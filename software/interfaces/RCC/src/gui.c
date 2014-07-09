/**
 * gui.c
 *
 * OpenGL (FreeGLUT) GUI initialization, display, and input handler functions
 */
#include "rcc.h"

int clickMode;
int prevClick;
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

		gluPickMatrix((GLdouble) x, (GLdouble) (h - y), PICK_DELTA, PICK_DELTA,
			viewport);

		/* Handle skewed aspect ratios. */
		aspectHandle(w, h);

		/* Redraw input elements of GUI. */
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
			drawRobots(GL_SELECT);
			drawAprilTagTextbox(GL_SELECT);
			drawToolbar(GL_SELECT);
			drawAprilTags(GL_SELECT);
		glPopMatrix();

		glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glFlush();

	/* Process clicked elements. */
	aprilTagURL.isActive = 0;
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
	switch (glutGetModifiers())
	{
	case (2): {
		switch (character)
		{
		case (12): {
			openLocalConnections();
			return;
		}
		case (18): {
			openRemoteConnections();
			return;
		}
		case (23): {
			killSecureCRT();
			return;
		}
		default: {
			break;
		}
		}
		return;
	}
	default: {
		break;
	}
	}

	if (!aprilTagURL.isActive)
		return;

	switch (character)
	{
	case ('\r'):
	case ('\n'): {
		if (!aprilTagConnected)
			connectAprilTag();
		break;
	}
	/* Backspace handler */
	case ('\b'): {
		if (aprilTagURL.index > 0) {
			strdel(aprilTagURL.message, --aprilTagURL.index);
		}
		break;
	}
	/* Delete handler */
	case (127): {
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

		switch (robotID)
		{
		case (-1): {
			continue;
		}
		/* Textbox */
		case (TEXTBOX_ID): {
			if (aprilTagConnected)
				openClientConnection(0);
			else
				aprilTagURL.isActive = 1;
			continue;
		}
		/* Toolbar buttons */
		case (CONNECT_BUTTON): {
			clickMode = CONNECT;
			continue;
		}
		case (HOST_BUTTON): {
			clickMode = HOSTBOT;
			continue;
		}
		case (BLACKLIST_BUTTON): {
			clickMode = BLACKLIST;
			continue;
		}
		case (SCONNECT_BUTTON): {
			clickMode = SCONNECT;
			continue;
		}
		case (ATLINK_BUTTON): {
			clickMode = ATLINK;
			continue;
		}
		case (LOG_BUTTON): {
			clickMode = LOG;
			continue;
		}
		case (OPENLOCAL_BUTTON): {
			openLocalConnections();
			continue;
		}
		case (OPENREMOTE_BUTTON): {
			openRemoteConnections();
			continue;
		}
		case (KILLALL_BUTTON): {
			killSecureCRT();
			continue;
		}
		default: {
			break;
		}
		}

		int mod = glutGetModifiers();

		/* Clicked robots */
		if (robotID < MAXROBOTID) {
			mutexLock(&robots[robotID].mutex);
			/* Do different things based on which mod keys are being held */
			switch (mod)
			{
			/* Shift-Click to Open a connection to a robot */
			case (1): {
				if (!robots[robotID].blacklisted
					|| robots[robotID].type == REMOTE)
					openClientConnection(robotID);
				break;
			}
			/* Ctrl-Click to blacklist robots */
			case (2): {
				blacklist(robotID);
				break;
			}
			/* Shift-Ctrl-Click to Log robots */
			case (3): {
				beginLog(robotID);
				break;
			}
			/* Alt-Click to make robot into a host */
			case (4): {
				hostRobot(robotID);
				break;
			}
			/* Ctrl-Alt-Click to open a direct connection to secureCRT */
			case (6): {
				commConnect(robotID);
				break;
			}
			/* Do whatever tool is selected */
			case (0):
			default: {
				switch (clickMode)
				{
				case (CONNECT): {
					if (!robots[robotID].blacklisted
						|| robots[robotID].type == REMOTE)
						openClientConnection(robotID);
					break;
				}
				case (HOSTBOT): {
					hostRobot(robotID);
					break;
				}
				case (BLACKLIST): {
					blacklist(robotID);
					break;
				}
				case (SCONNECT): {
					commConnect(robotID);
					break;
				}
				case (ATLINK): {
					if (prevClick >= 2000 && prevClick < MAX_APRILTAG + 2000) {
						if (aprilTagData[prevClick - 2000].active)
							robots[robotID].aid = prevClick - 2000;
					} else {
						robots[robotID].aid = -1;
					}
					break;
				}
				case (LOG): {
					beginLog(robotID);
					break;
				}
				default: {
					break;
				}
				}
				break;
			}
			}
			mutexUnlock(&robots[robotID].mutex);
		}

		/* Clicked AprilTags */
		if (robotID >= 2000 && robotID < MAX_APRILTAG + 2000) {
			switch (mod)
			{
			/* Shift-Ctrl-Click to Log AprilTags */
			case (3): {
				beginAprilTagLog(robotID - 2000);
				break;
			}
			case (0):
			default: {
				switch (clickMode)
				{
				case (ATLINK): {
					if (prevClick > 0 && prevClick < MAXROBOTID) {
						if (robots[prevClick].up)
							robots[prevClick].aid = robotID - 2000;
					}
					break;
				}
				case (LOG): {
					beginAprilTagLog(robotID - 2000);
					break;
				}
				default: {
					if (aprilTagData[robotID - 2000].display)
						aprilTagData[robotID - 2000].display = 0;
					else
						aprilTagData[robotID - 2000].display = 1;
					break;
				}
				}
				break;
			}
			}
		}
		prevClick = robotID;
	}
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

	GLfloat x;		// Robot x position
	GLfloat y;		// Robot y position
	GLfloat scale;	// Robot scale factor
	GLfloat start;	// Robot starting x

	int numLocal = 0;
	int numRemote = 0;
	int numRobots;
	struct commCon *local[MAXROBOTID];
	struct commCon *remote[MAXROBOTID];

	/* Iterate through robot list and find all active robots */
	for (i = 0; i < MAXROBOTID; i++) {
		mutexLock(&robots[i].mutex);
		/* If the robot is active */
		if (robots[i].up != 0) {
			if (robots[i].type == LOCAL || robots[i].type == HOST)
				local[numLocal++] = &robots[i];
			else if (robots[i].type == REMOTE)
				remote[numRemote++] = &robots[i];
		}
		mutexUnlock(&robots[i].mutex);
	}

	/* Figure scale for local robot pane */
	numRobots = numLocal + numRemote;
	int maxScale = 1;
	GLfloat end = ROBOT_END_X;
	if (aprilTagConnected) {
		maxScale = 2;
		end = MAP_DIVIDE_X;
	}

	if (numRobots <= (30 / maxScale)) {
		scale = SCALE_LARGE;
		x = ROBOT_START_X;
		y = ROBOT_START_Y;
	} else if (numRobots > (110 / maxScale)) {
		scale = SCALE_TINY;
		x = ROBOT_START_X;
		y = ROBOT_START_Y + ROBOT_RADIUS - ROBOT_RADIUS / scale;
	} else if (numRobots > (64 / maxScale)) {
		scale = SCALE_SMALL;
		x = ROBOT_START_X;
		y = ROBOT_START_Y + ROBOT_RADIUS / scale;
	} else if (numRobots > (30 / maxScale)) {
		scale = SCALE_MED;
		x = ROBOT_START_X + ROBOT_RADIUS / scale / 2;
		y = ROBOT_START_Y + ROBOT_RADIUS / scale / 2;
	}
	start = x;

	/* Draw local robots */
	for (i = 0; i < numLocal; i++) {
		mutexLock(&local[i]->mutex);
		if (mode == GL_SELECT)
			glLoadName(local[i]->id);

		drawRobot(x, y, local[i], scale);
		mutexUnlock(&local[i]->mutex);

		x += ROBOT_STEP_X / scale;
		if (x > end) {
			x = start;
			y -= ROBOT_STEP_Y / scale;
		}
	}
	for (i = 0; i < numRemote; i++) {
		mutexLock(&remote[i]->mutex);
		if (mode == GL_SELECT)
			glLoadName(remote[i]->id);

		drawRobot(x, y, remote[i], scale);
		mutexUnlock(&remote[i]->mutex);

		x += ROBOT_STEP_X / scale;
		if (x > end) {
			x = start;
			y -= ROBOT_STEP_Y / scale;
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

	if (robot->aid != -1 && aprilTagConnected) {
		glPushMatrix();
			glTranslatef(ROBOT_RADIUS / scale, -ROBOT_RADIUS / scale, 0);
			glPushMatrix();
				glScalef(0.55, 0.55, 0);
				glColor3fv(color_white);
				glRectf(-1, -1, 1, 1);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.5, 0.5, 0);
				glColor3fv(color_black);
				glRectf(-1, -1, 1, 1);
			glPopMatrix();
			glTranslatef(0, -0.3, 0);
			glColor3fv(color_white);
			textPrintf("%d", robot->aid);
		glPopMatrix();
	}

	if (robot->log) {
		glPushMatrix();
			glTranslatef(-ROBOT_RADIUS / scale, -ROBOT_RADIUS / scale, 0);
			glPushMatrix();
				glScalef(0.55, 0.55, 0);
				glColor3fv(color_black);
				glRectf(-1, -1, 1, 1);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.5, 0.5, 0);
				glColor3fv(color_white);
				glRectf(-1, -1, 1, 1);
			glPopMatrix();
			glTranslatef(0, -0.3, 0);
			glColor3fv(color_black);
			textPrintf("L", robot->aid);
		glPopMatrix();
	}
	glPopMatrix();
}

void drawAprilTags(GLenum mode)
{
	int i;
	int numAprilTags = 0;
	struct aprilTag *activeTags[MAX_APRILTAG];
	GLfloat xi, yi;

	glPushMatrix();
	glTranslatef(APRILTAG_X, APRILTAG_Y, 0);

	glPushMatrix();
		glPushMatrix();
			glColor3fv(color_grey);
			for (i = 0; i < AT_SCALE_X; i++) {
				glTranslatef(1, 0, 0);
				glBegin(GL_LINES);
					glVertex2f(0, -GUI_HEIGHT / 2);
					glVertex2f(0, GUI_HEIGHT / 2 - 2.75);
				glEnd();
			}
		glPopMatrix();
		glPushMatrix();
			glColor3fv(color_grey);
			for (i = 0; i < AT_SCALE_X; i++) {
				glTranslatef(-1, 0, 0);
				glBegin(GL_LINES);
					glVertex2f(0, -GUI_HEIGHT / 2);
					glVertex2f(0, GUI_HEIGHT / 2 - 2.75);
				glEnd();
			}
		glPopMatrix();
		glPushMatrix();
			glColor3fv(color_grey);
			for (i = 0; i < (GUI_HEIGHT / 2 - 3); i++) {
				glTranslatef(0, 1, 0);
				glBegin(GL_LINES);
					glVertex2f(-AT_SCALE_X - 1, 0);
					glVertex2f(AT_SCALE_X + 1, 0);
				glEnd();
			}
		glPopMatrix();
		glPushMatrix();
			glColor3fv(color_grey);
			for (i = 0; i < (GUI_HEIGHT / 2); i++) {
				glTranslatef(0, -1, 0);
				glBegin(GL_LINES);
					glVertex2f(-AT_SCALE_X - 1, 0);
					glVertex2f(AT_SCALE_X + 1, 0);
				glEnd();
			}
		glPopMatrix();

		glColor3fv(color_darkgrey);
		glBegin(GL_LINES);
			glVertex2f(0, -GUI_HEIGHT / 2);
			glVertex2f(0, GUI_HEIGHT / 2 - 2.75);
		glEnd();
		glBegin(GL_LINES);
			glVertex2f(-AT_SCALE_X - 1, 0);
			glVertex2f(AT_SCALE_X + 1, 0);
		glEnd();
		glPushMatrix();
			glTranslatef(-AT_SCALE_X, -AT_SCALE_X * (aprilTagY / aprilTagX), 0);
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(1, 0);
			glEnd();
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(0, 1);
			glEnd();
		glPopMatrix();
		glPushMatrix();
			glTranslatef(AT_SCALE_X, AT_SCALE_X * (aprilTagY / aprilTagX), 0);
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(-1, 0);
			glEnd();
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(0, -1);
			glEnd();
		glPopMatrix();
	glPopMatrix();

	for (i = 0; i < maxAprilTag + 1; i++) {
		mutexLock(&aprilTagData[i].mutex);
		if (aprilTagData[i].active)
			activeTags[numAprilTags++] = &aprilTagData[i];
		else
			mutexUnlock(&aprilTagData[i].mutex);
	}

	for (i = 0; i < numAprilTags; i++) {
		glPushMatrix();
			if (mode == GL_SELECT)
				glLoadName(2000 + activeTags[i]->id);

			xi = (activeTags[i]->x - aprilTagX) / aprilTagX;
			yi = (activeTags[i]->y - aprilTagY) / aprilTagY;

			xi *= AT_SCALE_X;
			yi *= -AT_SCALE_X * (aprilTagY / aprilTagX);

			glTranslatef(xi, yi, 0);
			glPushMatrix();
				glRotatef(activeTags[i]->t, 0, 0, 1);
				glRotatef(-90, 0, 0, 1);

				glPushMatrix();
					glColor3fv(color_white);
					glScalef(0.5, 0.5, 0);
					glRectf(-1, -1, 1, 1);
				glPopMatrix();
				glPushMatrix();
					glColor3fv(color_black);
					glScalef(0.45, 0.45, 0);
					glRectf(-1, -1, 1, 1);
					glRectf(-1, -1, 1, 1);
				glPopMatrix();
				glPushMatrix();
					glColor3fv(color_white);
					glScalef(0.45, 0.45, 0);
					glBegin(GL_LINES);
						glVertex2f(0, 0);
						glVertex2f(0, -1);
					glEnd();
				glPopMatrix();
			glPopMatrix();
			glPushMatrix();
				glTranslatef(0.6, -0.6, 0);
				glColor3fv(color_black);
				textSetSize(TEXT_SMALL);

				textPrintf("%d%s", activeTags[i]->id,
					activeTags[i]->log ? "L" : "");
			glPopMatrix();
			if (activeTags[i]->display) {
				textSetAlignment(ALIGN_CENTER);
				glPushMatrix();
					glTranslatef(0, -0.6 - TEXT_SMALL, 0);
					glColor3fv(color_black);
					textSetSize(TEXT_SMALL);
					textPrintf("%.2f", activeTags[i]->x);
					glTranslatef(0, -TEXT_SMALL, 0);
					textPrintf("%.2f", activeTags[i]->y);
					glTranslatef(0, -TEXT_SMALL, 0);
					textPrintf("%.2f", activeTags[i]->t);
				glPopMatrix();
				textSetAlignment(ALIGN_LEFT);
			}
		glPopMatrix();
		mutexUnlock(&activeTags[i]->mutex);
	}

	glPopMatrix();
}

void drawAprilTagTextbox(GLenum mode)
{
	char name[17] = "AprilTag Server: ";

	/* Find the drawn width of the strings. */
	GLfloat textWidth = TEXT_MED * gmf[(int) 'm'].gmfCellIncX *
		aprilTagURL.length;
	GLfloat nameWidth = TEXT_MED * gmf[(int) 'm'].gmfCellIncX * 17;

	glPushMatrix();
	if (mode == GL_SELECT)
		glLoadName(TEXTBOX_ID);

	glColor3fv(color_black);
	textSetAlignment(ALIGN_LEFT);
	textSetSize(TEXT_MED);

	glTranslatef(-TITLE_POS_X - textWidth - nameWidth, TITLE_POS_Y, 0);

	textPrintf(name);

	glTranslatef(nameWidth, 0, 0);

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

void drawToolbar(GLenum mode)
{
	GLfloat textWidth = TEXT_LARGE * gmf[(int) 'm'].gmfCellIncX * 2;

	glPushMatrix();
		/* CT Button */
		glTranslatef(TITLE_POS_X - 0.1,
			ROBOT_START_Y + ROBOT_RADIUS - 0.75, 0);
		if (mode == GL_SELECT)
			glLoadName(CONNECT_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		if (clickMode == CONNECT)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("CT");

		/* RT Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);
		if (mode == GL_SELECT)
			glLoadName(HOST_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		if (clickMode == HOSTBOT)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("RT");

		/* BL Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);
		if (mode == GL_SELECT)
			glLoadName(BLACKLIST_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		if (clickMode == BLACKLIST)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("BL");

		/* ST Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);
		if (mode == GL_SELECT)
			glLoadName(SCONNECT_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		if (clickMode == SCONNECT)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("ST");

		/* LG Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);
		if (mode == GL_SELECT)
			glLoadName(LOG_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		if (clickMode == LOG)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("LG");

		/* AL Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);

		if (mode == GL_SELECT)
			glLoadName(ATLINK_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		if (clickMode == ATLINK)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("AL");

		glTranslatef(0, -12, 0);

		/* OL Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);

		if (mode == GL_SELECT)
			glLoadName(OPENLOCAL_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("OL");

		/* OR Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);

		if (mode == GL_SELECT)
			glLoadName(OPENREMOTE_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("OR");

		/* KO Button */
		glTranslatef(0, -TEXT_LARGE - 0.25, 0);

		if (mode == GL_SELECT)
			glLoadName(KILLALL_BUTTON);

		glColor3fv(color_white);
		glRectf(0,
				0.,
				textWidth,
				TEXT_LARGE);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("KO");

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
	glPopMatrix();

	glPushMatrix();
		glTranslatef(TOOLBAR_DIVIDE_X, 0, 0);
		glRotatef(90, 0, 0, 1);
		glBegin(GL_LINES);
			glVertex2f(-GUI_HEIGHT, 0);
			glVertex2f(GUI_HEIGHT / 2 - TEXT_LARGE * 2, 0);
		glEnd();
	glPopMatrix();

	drawToolbar(GL_RENDER);

	if (aprilTagConnected) {
		drawAprilTags(GL_RENDER);

		glPushMatrix();
			glTranslatef(MAP_DIVIDE_X, 0, 0);
			glRotatef(90, 0, 0, 1);
			glBegin(GL_LINES);
				glVertex2f(-GUI_HEIGHT, 0);
				glVertex2f(GUI_HEIGHT / 2 - TEXT_LARGE * 2, 0);
			glEnd();
		glPopMatrix();
	}

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
	aprilTagURL.index = strlen(defaultATServerIP);
	aprilTagURL.length = 21;
	sprintf(aprilTagURL.message, defaultATServerIP);

	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	clickMode = CONNECT;

	glutTimerFunc(DRAW_DELAY, timerEnableDraw, 0);

	glutMainLoop();
}
