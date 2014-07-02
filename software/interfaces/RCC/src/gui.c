/**
 * gui.c
 *
 * OpenGL (FreeGLUT) GUI initialization, display, and input handler functions
 */
#include "rcc.h"

/* The script template */
const char scriptTemplate[256] =
	"#$language = \"VBScript\"\r\n#$interface = \"1.0\"\r\n\r\nSub Main()\r\n\tcrt.Session.Connect \"/TELNET %s %d\"\r\n\tcrt.Screen.Synchronous = True\r\n\tcrt.Screen.WaitForString \"Enter the robot ID\"\r\n\tcrt.Screen.Send \"%d\" & Chr(13)\r\nEnd Sub\r\n";

struct textbox aprilTagURL;

/**
 * Display function, we don't want to do anything here
 */
void display()
{
}

/**
 * Handles the reshaping of the GUI
 */
void reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	aspectHandle(w, h);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/**
 * Handles mouse events on the GUI
 */
void mouse(int button, int state, int x, int y)
{
	GLuint selectBuf[GUI_BUFFER];
	GLint hits;
	GLint viewport[4];

	/* Until mouse has gone up, don't process input again. */
	if (button != GLUT_LEFT_BUTTON || state != GLUT_DOWN)
		return;

	glGetIntegerv(GL_VIEWPORT, viewport);
	GLint w = viewport[2];
	GLint h = viewport[3];

	glSelectBuffer(GUI_BUFFER, selectBuf);
	glRenderMode(GL_SELECT);

	glInitNames();
	glPushName(~0);

	glMatrixMode(GL_PROJECTION);

	glPushMatrix();
		glLoadIdentity();

		gluPickMatrix((GLdouble) x, (GLdouble) (h - y),
		PICK_DELTA, PICK_DELTA, viewport);

		/* Handle skewed aspect ratios. */
		aspectHandle(w, h);

		/* Redraw input elements of GUI. */
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
			drawRobots(GL_SELECT);
			drawAprilTagTextbox(GL_SELECT);
		glPopMatrix();

		glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glFlush();

	/* Process clicked elements. */
	hits = glRenderMode(GL_RENDER);
	processHits(hits, selectBuf);

	/* Update GUI. */
	glutPostRedisplay();
}

/**
 * Handles character input
 */
void keyboard(unsigned char key, int x, int y)
{
	/* Not using these variables. */
	x = (int) x;
	y = (int) y;

	/* Exit on escape key. */
	if (key == 27)
		exit (0);
	else
		readChar(key);

	/* Update GUI. */
	glutPostRedisplay();
}

/**
 * Handles keyboard input
 */
void readChar(char character)
{
	if (!aprilTagURL.isActive)
		return;

	switch (character)
	{
	case '\r':
	case '\n': {
		if (!aprilTagConnected)
			connectAprilTag();
		break;
	}
	/* Backspace handler */
	case '\b': {
		if (aprilTagURL.index > 0) {
			strdel(aprilTagURL.message, --aprilTagURL.index);
		}
		break;
	}
	/* Delete handler */
	case 127: {
		strdel(aprilTagURL.message, aprilTagURL.index);
		break;
	}
	/* Default: insert the character */
	default: {
		if (strlen(aprilTagURL.message) < aprilTagURL.length) {
			strins(aprilTagURL.message, character, aprilTagURL.index++);
		}
		break;
	}
	}
}

/**
 * Find out which robot was clicked on the GUI
 */
void processHits(GLint hits, GLuint buffer[])
{
	int i;
	unsigned int j;
	GLuint names, *ptr;
	int robotID = -1;

	if (hits == 0)
		return;

	ptr = (GLuint *) buffer;

	/* Iterate through all hits */
	for (i = 0; i < hits; i++) {
		names = *ptr;
		ptr += 3;

		if (names == 0)
			continue;

		for (j = 0; j < names; j++) {
			if (j == 0)
				robotID = *ptr;

			ptr++;
		}

		if (robotID == -1)
			continue;

		if (robotID == TEXTBOX_ID) {
			if (aprilTagConnected)
				openClientConnection(0);
			else
				aprilTagURL.isActive = 1;
			continue;
		} else {
			aprilTagURL.isActive = 0;
		}

		mutexLock(&robots[robotID].mutex);
		/* Do different things based on which mod keys are being held */
		switch (glutGetModifiers())
		{
		/* Ctrl-Click to Blacklist local robots */
		case (2): {
			if (robots[robotID].blacklisted) {
				robots[robotID].blacklisted = 0;
				if (initCommCommander(robots[robotID].port) < 0) {
					commToNum[robots[robotID].port] = 0;
					robots[robotID].type = UNKNOWN;
				}
			} else {
				robots[robotID].blacklisted = 1;
				if (robots[robotID].hSerial != NULL)
					CloseHandle(*robots[robotID].hSerial);
			}
			break;
		}
		/* Alt-Click to make robot into a host */
		case (4): {
			if (robots[robotID].type == LOCAL && robots[robotID].hSerial != NULL
				&& !robots[robotID].blacklisted)
				hprintf(robots[robotID].hSerial, "rt\n");
			break;
		}
		/* Ctrl-Alt-Click to open a direct connection to secureCRT */
		case (6): {
			if (robots[robotID].type != REMOTE &&
				robots[robotID].hSerial != NULL && !robots[robotID].blacklisted)
				directConnect(robotID);
			break;
		}
		/* Click to open a secureCRT connection */
		case (0):
		default: {
			if (!robots[robotID].blacklisted || robots[robotID].type == REMOTE)
				openClientConnection(robotID);
			break;
		}
		}
		mutexUnlock(&robots[robotID].mutex);
	}
}

/**
 * Opens a secureCRT window and connects to the server to the requested ID
 */
int openClientConnection(int robotID)
{
	HANDLE hTempFile = INVALID_HANDLE_VALUE;

	DWORD dwRetVal = 0;
	UINT uRetVal = 0;

	TCHAR szTempFileName[MAX_PATH];
	TCHAR lpTempPathBuffer[MAX_PATH];
	char buffer[1024];

	/* Create a temporary file */
	dwRetVal = GetTempPath(MAX_PATH, lpTempPathBuffer);

	if (dwRetVal > MAX_PATH || (dwRetVal == 0))
		return (-1);

	uRetVal = GetTempFileName(lpTempPathBuffer,
							  TEXT("SCRIPT"),
							  0,
							  szTempFileName);
	if (uRetVal == 0)
		return (-1);

	hTempFile = CreateFile((LPTSTR) szTempFileName,
						   GENERIC_WRITE, 0,
						   NULL,
						   CREATE_ALWAYS,
						   FILE_ATTRIBUTE_NORMAL,
						   NULL);
	if (hTempFile == INVALID_HANDLE_VALUE)
		return (-1);

	/* Output the script to the temporary file */
	hprintf(&hTempFile, scriptTemplate, ipAddress, port, robotID);

	if (!CloseHandle(hTempFile))
		return (-1);

	/* Open secureCRT with the script as an argument */
	if (sprintf(buffer, "/SCRIPT \"%s\"", szTempFileName) < 0)
		return (-1);

	ShellExecute(GetDesktopWindow(),
				 "open",
				 "securecrt.exe",
				 buffer,
				 "",
				 SW_SHOW);

	return (0);
}

/**
 * Opens a direct secureCRT connection to a local robot
 */
int directConnect(int robotID)
{
	char buffer[BUFFERSIZE];

	robots[robotID].blacklisted = 1;
	if (robots[robotID].hSerial != NULL)
		CloseHandle(*robots[robotID].hSerial);

	if (sprintf(buffer, "/SERIAL COM%d /BAUD 230400 /NOCTS",
		robots[robotID].port) < 0)
		return (-1);

	ShellExecute(GetDesktopWindow(),
				 "open",
				 "securecrt.exe",
				 buffer,
				 "",
				 SW_SHOW);

	return (0);
}

/**
 * Handles aspect ratio skew
 */
void aspectHandle(int w, int h)
{
	if (w <= h * ASPECT) {
		gluOrtho2D(-GUI_WIDTH / 2.,
				   GUI_WIDTH / 2.,
				   -GUI_HEIGHT * (GLfloat) h / (GLfloat) w / 2. * ASPECT,
				   GUI_HEIGHT * (GLfloat) h / (GLfloat) w / 2. * ASPECT);
	} else {
		gluOrtho2D(-GUI_WIDTH * (GLfloat) w / (GLfloat) h / 2. / ASPECT,
				   GUI_WIDTH * (GLfloat) w / (GLfloat) h / 2. / ASPECT,
				   -GUI_HEIGHT / 2.,
				   GUI_HEIGHT / 2.);
	}
}

/**
 * Draw all the robots current connected
 */
void drawRobots(GLenum mode)
{
	int i;

	GLfloat lx; // Local robot x position
	GLfloat ly; // Local robot y position
	GLfloat rx; // Remote robot x position
	GLfloat ry; // Remote robot y position
	GLfloat sx; // Starting x position

	GLfloat ls = SCALE_LARGE; // Local robot scale factor
	GLfloat rs = SCALE_LARGE; // Remote robot scale factor

	int numLocal = 0;
	int numRemote = 0;

	struct commCon *local[MAXROBOTID];
	struct commCon *remote[MAXROBOTID];

	/* Iterate through robot list and find all active robots */
	for (i = 0; i < MAXROBOTID; i++) {
		mutexLock(&robots[i].mutex);

		/* If the robot is active */
		if (robots[i].up != 0) {
			if (robots[i].type == LOCAL || robots[i].type == HOST) {
				local[numLocal++] = &robots[i];
			} else if (robots[i].type == REMOTE) {
				remote[numRemote++] = &robots[i];
			}
		}

		mutexUnlock(&robots[i].mutex);
	}

	/* Figure scale for local robot pane */
	if (numLocal <= 12) {
		ls = SCALE_LARGE;
		lx = ROBOT_START_LX;
		ly = ROBOT_START_LY;
	} else if (numLocal > 12) {
		ls = SCALE_MED;
		lx = ROBOT_START_LX + ROBOT_RADIUS / ls / 2;
		ly = ROBOT_START_LY + ROBOT_RADIUS / ls / 2;
	} else if (numLocal > 24) {
		ls = SCALE_SMALL;
		lx = ROBOT_START_LX;
		ly = ROBOT_START_LY + ROBOT_RADIUS;
	} else if (numLocal > 44) {
		ls = SCALE_TINY;
		lx = ROBOT_START_LX;
		ly = ROBOT_START_LY + ROBOT_RADIUS + ROBOT_RADIUS / ls;
	}
	sx = lx;

	/* Draw local robots */
	for (i = 0; i < numLocal; i++) {
		mutexLock(&local[i]->mutex);
		if (mode == GL_SELECT)
			glLoadName(local[i]->id);

		drawRobot(lx, ly, local[i], ls);
		mutexUnlock(&local[i]->mutex);

		lx += ROBOT_STEP_X / ls;
		if (lx > -sx) {
			lx = sx;
			ly -= ROBOT_STEP_Y / ls;
		}
	}

	/* Figure scale for remote robots */
	if (numRemote <= 12) {
		rs = SCALE_LARGE;
		rx = ROBOT_START_RX;
		ry = ROBOT_START_RY;
	} else if (numRemote > 12) {
		rs = SCALE_MED;
		rx = ROBOT_START_RX + ROBOT_RADIUS / rs / 2;
		ry = ROBOT_START_RY + ROBOT_RADIUS / rs / 2;
	} else if (numRemote > 24) {
		rs = SCALE_SMALL;
		rx = ROBOT_START_RX;
		ry = ROBOT_START_RY + ROBOT_RADIUS;
	} else if (numRemote > 44) {
		rs = SCALE_TINY;
		rx = ROBOT_START_RX;
		ry = ROBOT_START_RY + ROBOT_RADIUS + ROBOT_RADIUS / rs;
	}
	sx = rx;

	/* Draw remote robots */
	for (i = 0; i < numRemote; i++) {
		mutexLock(&remote[i]->mutex);
		if (mode == GL_SELECT)
			glLoadName(remote[i]->id);

		drawRobot(rx, ry, remote[i], rs);
		mutexUnlock(&remote[i]->mutex);

		rx += ROBOT_STEP_X / rs;
		if (rx > -sx) {
			rx = sx;
			ry -= ROBOT_STEP_Y / rs;
		}
	}
}

/**
 * Draw a robot at a given coordinate and scale
 */
void drawRobot(GLfloat x, GLfloat y, struct commCon *robot, GLfloat scale)
{
	glPushMatrix();
	glTranslatef(x, y, 0);

	/* Draw a local robot */
	if (robot->type == LOCAL || robot->type == HOST) {
		if (robot->type == HOST) {
			glPushMatrix();
				glColor3fv(color_red);
				glScalef(HOST_RADIUS / scale, HOST_RADIUS / scale, 0);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();
			glPushMatrix();
				glColor3fv(color_white);
				glScalef(OUTER_RADIUS / scale, OUTER_RADIUS / scale, 0);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();
		}

		glPushMatrix();
			glScalef(ROBOT_RADIUS / scale, ROBOT_RADIUS / scale, 0);
			glColor3fv(color_black);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();

		glPushMatrix();
		textSetAlignment(ALIGN_CENTER);
		glColor3fv(color_white);

		/* Draw different text for different sizes */
		if (scale == SCALE_LARGE) {
			textSetSize(TEXT_MED);

			glTranslatef(0, -ROBOT_RADIUS + 1, 0);
			textPrintf("COM%d", robot->port);

			glTranslatef(0, ROBOT_RADIUS, 0);
		} else if (scale == SCALE_MED) {
			textSetSize(TEXT_SMALL);

			glTranslatef(0, -ROBOT_RADIUS + 1.2, 0);
			textPrintf("COM%d", robot->port);

			glTranslatef(0, ROBOT_RADIUS / scale, 0);
			textSetSize(TEXT_MED);
		} else if (scale == SCALE_SMALL) {
			textSetSize(TEXT_SMALL);

			glTranslatef(0, 0.4, 0);
		} else if (scale == SCALE_TINY) {
			textSetSize(TEXT_SMALL);
		}
		textPrintf("%02d", robot->id);

		glPopMatrix();

	/* Draw a remote robot */
	} else {
		/* Make the robot hollow for remote robots */
		glPushMatrix();
			glScalef(ROBOT_RADIUS / scale, ROBOT_RADIUS / scale, 0);
			glColor3fv(color_black);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();
		glPushMatrix();
			glColor3fv(color_white);
			glScalef(INNER_RADIUS / scale, INNER_RADIUS / scale, 0);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();

		glPushMatrix();
		textSetAlignment(ALIGN_CENTER);
		glColor3fv(color_black);

		/* Draw different text for different sizes */
		if (scale == SCALE_LARGE) {
			textSetSize(TEXT_SMALL);

			glTranslatef(0, -ROBOT_RADIUS + 1, 0);
			textPrintf("Host:%02d", robot->host);

			glTranslatef(0, ROBOT_RADIUS, 0);
			textSetSize(TEXT_MED);
		} else if (scale == SCALE_MED) {
			textSetSize(TEXT_MED);

			glTranslatef(0, ROBOT_RADIUS * (1. / scale - 1) + 1.2, 0);
		} else if (scale == SCALE_SMALL) {
			textSetSize(TEXT_SMALL);

			glTranslatef(0, 0.4, 0);
		} else if (scale == SCALE_TINY) {
			textSetSize(TEXT_SMALL);
		}
		textPrintf("%02d", robot->id);

		glPopMatrix();
	}

	if (robot->blacklisted) {
		glPushMatrix();
			glColor3fv(color_red);
			glRotatef(45, 0, 0, 1);
			glBegin(GL_LINES);
				glVertex2f(-ROBOT_RADIUS / scale, 0);
				glVertex2f(ROBOT_RADIUS / scale, 0);
			glEnd();
			glBegin(GL_LINES);
				glVertex2f(0, -ROBOT_RADIUS / scale);
				glVertex2f(0, ROBOT_RADIUS / scale);
			glEnd();
		glPopMatrix();
	}
	glPopMatrix();
}

void drawAprilTagTextbox(GLenum mode)
{
	GLfloat textWidth = TEXT_MED * gmf[(int) 'm'].gmfCellIncX *
		aprilTagURL.length;

	glPushMatrix();
	glTranslatef(-TITLE_POS_X - textWidth, TITLE_POS_Y, 0);

	if (mode == GL_SELECT)
		glLoadName(TEXTBOX_ID);

	if (aprilTagConnected)
		glColor3fv(color_red);
	else
		glColor3fv(color_black);

	glRectf(-LINE_WIDTH_SMALL,
			-LINE_WIDTH_SMALL,
			textWidth + LINE_WIDTH_SMALL,
			TEXT_MED + LINE_WIDTH_SMALL);

	glColor3fv(color_black);
	glRectf(-LINE_WIDTH_SMALL / 2.,
			-LINE_WIDTH_SMALL / 2.,
			textWidth + LINE_WIDTH_SMALL / 2.,
			TEXT_MED + LINE_WIDTH_SMALL / 2.);

	textSetAlignment(ALIGN_LEFT);
	textSetSize(TEXT_MED);
	glColor3fv(color_white);
	textPrintf(aprilTagURL.message);

	if (aprilTagURL.isActive) {
		float length = 0;
		int loop;
		for (loop = 0; loop < (aprilTagURL.index); loop++)
			length += gmf[(int) aprilTagURL.message[loop]].gmfCellIncX;

		glPushMatrix();
			glTranslatef(length * TEXT_MED, 0, 0);
			glBegin(GL_LINES);
				glVertex2f(0.0, TEXT_MED);
				glVertex2f(0.0, 0.0);
			glEnd();
		glPopMatrix();
	}

	glPopMatrix();
}

/**
 * Draw the robots and GUI features on this timed function
 */
void timerEnableDraw(int value)
{
	GLfloat length;
	glClear(GL_COLOR_BUFFER_BIT);

	/* Draw title, IP, port, and dividing bar for local robots */
	glPushMatrix();
		glTranslatef(TITLE_POS_X, TITLE_POS_Y, 0);
		glColor3fv(color_black);
		textSetAlignment(ALIGN_LEFT);
		textSetSize(TEXT_LARGE);
		length = textPrintf("%s", NAME);
		glTranslatef(length + 2.5, 0, 0);
		textSetSize(TEXT_MED);
		textPrintf("[%s:%d]", ipAddress, port);
		glTranslatef(-length - 2.5, 0, 0);

		glTranslatef(-TITLE_POS_X, -TEXT_LARGE / 2, 0);
		glBegin(GL_LINES);
			glVertex2f(-GUI_WIDTH, 0);
			glVertex2f(GUI_WIDTH, 0);
		glEnd();

		glTranslatef(TITLE_POS_X, -TEXT_MED, 0);
		textSetSize(TEXT_MED);
		textPrintf("Local Robots");
	glPopMatrix();

	/* Draw dividing bar for remote robots */
	glPushMatrix();
		glTranslatef(0, -TEXT_LARGE, 0);
		glColor3fv(color_black);
		glBegin(GL_LINES);
			glVertex2f(-GUI_WIDTH, 0);
			glVertex2f(GUI_WIDTH, 0);
		glEnd();

		glTranslatef(TITLE_POS_X, -TEXT_MED, 0);
		textSetSize(TEXT_MED);
		textPrintf("Remote Robots");
	glPopMatrix();

	drawRobots(GL_RENDER);
	drawAprilTagTextbox(GL_RENDER);

	/* Update */
	glutSwapBuffers();
	glutPostRedisplay();

	/* Initialize timer loop. */
	glutTimerFunc(DRAW_DELAY, timerEnableDraw, value);
}

/**
 * Initialize and run the main loop of the GUI
 */
void guiInit()
{
	int argc = 0;
	glutInit(&argc, NULL);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow(NAME);

	textInit();
	drawInit();

	/* Initialize textbox */
	aprilTagURL.isActive = 0;
	aprilTagURL.index = 0;
	aprilTagURL.length = 21;

	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutTimerFunc(DRAW_DELAY, timerEnableDraw, 0);

	glutMainLoop();
}
