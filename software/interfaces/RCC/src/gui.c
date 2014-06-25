/**
 * gui.c
 *
 * OpenGL (FreeGLUT) GUI initialization, display, and input handler functions
 */
#include "rcc.h"

/* The script template */
const char scriptTemplate[256] = "#$language = \"VBScript\"\r\n#$interface = \"1.0\"\r\n\r\nSub Main()\r\n\tcrt.Session.Connect \"/TELNET %s %d\"\r\n\tcrt.Screen.Synchronous = True\r\n\tcrt.Screen.WaitForString \"Enter the robot ID you wish to view: \"\r\n\tcrt.Screen.Send \"%d\" & Chr(13)\r\nEnd Sub\r\n";

/**
 * Display function, we don't want to do anything here
 */
void
display()
{
}

/**
 * Handles the reshaping of the GUI
 */
void
reshape(int w, int h)
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
void
mouse(int button, int state, int x, int y)
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
 * Find out which robot was clicked on the GUI
 */
void
processHits(GLint hits, GLuint buffer[])
{
	int i;
	unsigned int j;
	GLuint names, *ptr;
	int robotID;

	if (hits == 0)
		return;

	ptr = (GLuint *)buffer;

	/* Iterate through all hits */
	for (i = 0; i < hits; i++) {
		names = *ptr;
		ptr += 3;

		if (names == 0)
			continue;

		for (j = 0; j < names; j++)	{
			if (j == 0)
				robotID = *ptr;

			ptr++;
		}

		/* If modifier keys are held down */
		switch (glutGetModifiers())
		{
		/* Blacklist local robots */
		case (2): {
			if (robots[robotID].type == LOCAL || robots[robotID].type == HOST) {
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
			}
			break;
		}
		/* Alt-click to make into a host */
		case (4): {
			if (robots[robotID].type == LOCAL &&
				robots[robotID].hSerial != NULL &&
				!robots[robotID].blacklisted)
				fcprintf(robots[robotID].hSerial, "rt\n");
			break;
		}
		case (0):
		default: {
			if (!robots[robotID].blacklisted)
				openClientConnection(robotID);
			break;
		}
		}

	}
}

/**
 * Opens a secureCRT window and connects to the server to the requested ID
 */
int
openClientConnection(int robotID)
{
    HANDLE hTempFile = INVALID_HANDLE_VALUE;

    DWORD dwRetVal = 0;
    UINT uRetVal   = 0;

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
						   GENERIC_WRITE,
						   0,
						   NULL,
						   CREATE_ALWAYS,
						   FILE_ATTRIBUTE_NORMAL,
						   NULL);
	if (hTempFile == INVALID_HANDLE_VALUE)
        return (-1);

	/* Output the script to the temporary file */
	fcprintf(&hTempFile, scriptTemplate, ipAddress, port, robotID);

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
 * Handles aspect ratio skew
 */
void
aspectHandle(int w, int h)
{
	if (w <= h * ASPECT) {
		gluOrtho2D(-GUI_WIDTH / 2.,
			GUI_WIDTH / 2.,
   			-GUI_HEIGHT * (GLfloat) h / (GLfloat) w / 2. * ASPECT,
   			GUI_HEIGHT * (GLfloat) h / (GLfloat) w / 2. * ASPECT);
	} else {
		gluOrtho2D (-GUI_WIDTH * (GLfloat) w / (GLfloat) h / 2. / ASPECT,
   			GUI_WIDTH * (GLfloat) w / (GLfloat) h / 2. / ASPECT,
   			-GUI_HEIGHT / 2.,
   			GUI_HEIGHT / 2.);
	}
}

/**
 * Draw all the robots current connected
 */
void
drawRobots(GLenum mode)
{
	int i;

	GLfloat lx;	/* Local robot x position */
	GLfloat ly;	/* Local robot y position */
	GLfloat rx;	/* Remote robot x position */
	GLfloat ry;	/* Remote robot y position */
	GLfloat sx;	/* Starting x position */

	GLfloat ls = SCALE_LARGE;	/* Local robot scale factor */
	GLfloat rs = SCALE_LARGE;	/* Remote robot scale factor */

	int numLocal = 0;
	int numRemote = 0;

	struct commCon *local[MAXROBOTID];
	struct commCon *remote[MAXROBOTID];

	/* Iterate through robot list and find all active robots */
	for (i = 0; i < MAXROBOTID; i++) {
		Pthread_mutex_lock(&robots[i].mutex);

		/* If the robot is active */
		if (robots[i].up != 0) {
			if (robots[i].type == LOCAL || robots[i].type == HOST) {
				local[numLocal++] = &robots[i];
			}
			else if (robots[i].type == REMOTE) {
				remote[numRemote++] = &robots[i];
			}
		}

		Pthread_mutex_unlock(&robots[i].mutex);
	}

	/* Figure scale for local robot pane */
	if (numLocal <= 12) {
		ls = SCALE_LARGE;
		lx = ROBOT_START_LX;
		ly = ROBOT_START_LY;
	}
	if (numLocal > 12) {
		ls = SCALE_MED;
		lx = ROBOT_START_LX + ROBOT_RADIUS / ls / 2;
		ly = ROBOT_START_LY + ROBOT_RADIUS / ls / 2;
	}
	if (numLocal > 24) {
		ls = SCALE_SMALL;
		lx = ROBOT_START_LX;
		ly = ROBOT_START_LY + ROBOT_RADIUS;
	}
	if (numLocal > 44) {
		ls = SCALE_TINY;
		lx = ROBOT_START_LX;
		ly = ROBOT_START_LY + ROBOT_RADIUS + ROBOT_RADIUS / ls;

	}
	sx = lx;

	/* Draw local robots */
	for (i = 0; i < numLocal; i++) {
		Pthread_mutex_lock(&local[i]->mutex);
		if (mode == GL_SELECT)
			glLoadName(local[i]->id);

		drawRobot(lx, ly, local[i], ls);
		Pthread_mutex_unlock(&local[i]->mutex);

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
	}
	if (numRemote > 12) {
		rs = SCALE_MED;
		rx = ROBOT_START_RX + ROBOT_RADIUS / rs / 2;
		ry = ROBOT_START_RY + ROBOT_RADIUS / rs / 2;
	}
	if (numRemote > 24) {
		rs = SCALE_SMALL;
		rx = ROBOT_START_RX;
		ry = ROBOT_START_RY + ROBOT_RADIUS;
	}
	if (numRemote > 44) {
		rs = SCALE_TINY;
		rx = ROBOT_START_RX;
		ry = ROBOT_START_RY + ROBOT_RADIUS + ROBOT_RADIUS / rs;

	}
	sx = rx;

	/* Draw remote robots */
	for (i = 0; i < numRemote; i++) {
		Pthread_mutex_lock(&remote[i]->mutex);
		if (mode == GL_SELECT)
			glLoadName(remote[i]->id);

		drawRobot(rx, ry, remote[i], rs);
		Pthread_mutex_unlock(&remote[i]->mutex);

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
void
drawRobot(GLfloat x, GLfloat y, struct commCon *robot, GLfloat scale)
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

/**
 * Draw the robots and GUI features on this timed function
 */
void
timerEnableDraw(int value)
{
	glClear(GL_COLOR_BUFFER_BIT);

	/* Draw title, IP, port, and dividing bar for local robots */
	glPushMatrix();
		glTranslatef(TITLE_POS_X, TITLE_POS_Y, 0);
		glColor3fv(color_black);
		textSetAlignment(ALIGN_LEFT);
		textSetSize(TEXT_LARGE);
		textPrintf("%s - %s:%d", NAME, ipAddress, port);

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

	/* Update */
	glutSwapBuffers();
	glutPostRedisplay();

	/* Initialize timer loop. */
	glutTimerFunc(DRAW_DELAY, timerEnableDraw, value);
}

/**
 * Initialize and run the main loop of the GUI
 */
void
guiInit()
{
	int argc = 0;
	glutInit(&argc, NULL);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow(NAME);

	textInit();
	drawInit();

	glutMouseFunc(mouse);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutTimerFunc(DRAW_DELAY, timerEnableDraw, 0);

	glutMainLoop();
}
