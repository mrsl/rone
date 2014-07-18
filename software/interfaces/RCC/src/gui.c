/**
 * gui.c
 *
 * OpenGL (FreeGLUT) GUI initialization, display, and input handler functions
 */
#include "rcc.h"

int showHelp = 0;
int clickMode;
int prevClick;
struct textbox aprilTagURL;

char connectName[20] = 		"Connect via Telnet";
char blacklistName[20] =	"Blacklist";
char serialName[20] =		"Connect via Serial";
char aprilName[20] =		"AprilTag Linking";
char hostName[20] =			"Make Radio Host";
char infoName[20] =			"Extra Information";
char logName[20] =			"Log Data";

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
	showHelp = 0;
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
		case (1): {
			aprilTagURL.isActive = 1;
			return;
		}
		case (12): {
			openLocalConnections();
			return;
		}
		case (17): {
			if (showHelp)
				showHelp = 0;
			else
				showHelp = 1;
			return;
		}
		case (18): {
			openRemoteConnections();
			return;
		}
		case (20): {
			timestamps = (timestamps) ? 0 : 1;
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

	if (!aprilTagURL.isActive) {
		switch (character)
		{
		case ('1'): {
			clickMode = CONNECT;
			break;
		}
		case ('2'): {
			clickMode = HOSTBOT;
			break;
		}
		case ('3'): {
			clickMode = BLACKLIST;
			break;
		}
		case ('4'): {
			clickMode = SCONNECT;
			break;
		}
		case ('5'): {
			clickMode = LOG;
			break;
		}
		case ('6'): {
			clickMode = ATLINK;
			break;
		}
		case ('7'): {
			clickMode = DISPLAY;
			break;
		}
		default: {
			break;
		}
		}
		return;
	}

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
			if (aprilTagConnected) {
				if (clickMode == CONNECT)
					openClientConnection(0, -1);
			} else {
				aprilTagURL.isActive = 1;
			}
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
		case (INFO_BUTTON): {
			clickMode = DISPLAY;
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
		case (HELP_BUTTON): {
			showHelp = 1;
			continue;
		}
		case (TIME_BUTTON): {
			timestamps = (timestamps) ? 0 : 1;
			continue;
		}
		default: {
			break;
		}
		}

		int mod = glutGetModifiers();

		/* Clicked robots */
		if (robotID < MAXROBOTID) {
			/* Do different things based on which mod keys are being held */
			switch (mod)
			{
			/* Shift-Click to Open a connection to a robot */
			case (1): {
				if (!robots[robotID].blacklisted
					|| robots[robotID].type == REMOTE)
					openClientConnection(robotID, -1);
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
			/* Shift-Alt-Click to show info */
			case (5): {
				showRobotInfo(robotID);
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
						openClientConnection(robotID, -1);
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
						if (aprilTagData[prevClick - 2000].active) {
							if (robots[robotID].aid != -1) {
								aprilTagData[robots[robotID].aid].rid = -1;
							}
							if (aprilTagData[prevClick - 2000].rid != -1) {
								robots[aprilTagData[prevClick - 2000].rid].aid = -1;
							}
							robots[robotID].aid = prevClick - 2000;
							aprilTagData[prevClick - 2000].rid = robotID;
							robotID = -1;
						}
					} else if (robotID == prevClick){
						if (robots[robotID].aid != -1) {
							aprilTagData[robots[robotID].aid].rid = -1;
						}
						robots[robotID].aid = -1;
						robotID = -1;
					}
					break;
				}
				case (LOG): {
					beginLog(robotID);
					break;
				}
				case (DISPLAY): {
					showRobotInfo(robotID);
					break;
				}
				default: {
					break;
				}
				}
				break;
			}
			}
		}

		/* Clicked AprilTags */
		if (robotID >= 2000 && robotID < MAX_APRILTAG + 2000) {
			switch (mod)
			{
			case (1): {
				openClientConnection(0, robotID - 2000);
				break;
			}
			/* Shift-Ctrl-Click to Log AprilTags */
			case (3): {
				beginAprilTagLog(robotID - 2000);
				break;
			}
			/* Shift-Alt-Click to show info */
			case (5): {
				showAprilTagInfo(robotID);
				break;
			}
			case (0):
			default: {
				switch (clickMode)
				{
				case (CONNECT): {
					openClientConnection(0, robotID - 2000);
				}
				case (ATLINK): {
					if (prevClick > 0 && prevClick < MAXROBOTID) {
						if (robots[prevClick].up) {
							if (robots[prevClick].aid != -1) {
								aprilTagData[robots[prevClick].aid].rid = -1;
							}
							if (aprilTagData[robotID - 2000].rid != -1) {
								robots[aprilTagData[robotID - 2000].rid].aid = -1;
							}
							robots[prevClick].aid = robotID - 2000;
							aprilTagData[robotID - 2000].rid = prevClick;
							robotID = -1;
						}
						robotID = -1;
					} else if (robotID == prevClick){
						if (aprilTagData[robotID - 2000].rid != -1) {
							robots[aprilTagData[robotID - 2000].rid].aid = -1;
						}
						aprilTagData[robotID - 2000].rid = -1;
						robotID = -1;
					}
					break;
				}
				case (LOG): {
					beginAprilTagLog(robotID - 2000);
					break;
				}
				case (DISPLAY): {
					showAprilTagInfo(robotID);
					break;
				}
				default: {
					break;
				}
				}
				break;
			}
			}
		}
		if ((robotID >= 2000 && robotID < MAX_APRILTAG + 2000)
			|| robotID < MAXROBOTID) {
			prevClick = robotID;
		} else {
			prevClick = -1;
		}
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
	if (maxScale != 1)
		x -= 0.5;

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
	/* Draw remote robots */
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
				glTranslatef(0.05, -0.05, 0);
				glColor3fv(color_darkgrey);
				glScalef(HOST_RADIUS / scale, HOST_RADIUS / scale, 0);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();
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
			glTranslatef(0.05, -0.05, 0);
			glColor3fv(color_darkgrey);
			glScalef(ROBOT_RADIUS / scale, ROBOT_RADIUS / scale, 0);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();
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
			glTranslatef(0.05, -0.05, 0);
			glColor3fv(color_darkgrey);
			glScalef(ROBOT_RADIUS / scale, ROBOT_RADIUS / scale, 0);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();
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

	/* Draw a red X over a robot if it is blacklisted */
	if (robot->blacklisted) {
		glPushMatrix();
			if (robot->type == REMOTE)
				glColor3fv(color_lightred);
			else
				glColor3fv(color_red);
			glRotatef(45, 0, 0, 1);
			glBegin(GL_LINES);
				glVertex2f(-ROBOT_RADIUS / scale + DROPSHADOW_DIST, 0);
				glVertex2f(ROBOT_RADIUS / scale - DROPSHADOW_DIST, 0);
			glEnd();
			glBegin(GL_LINES);
				glVertex2f(0, -ROBOT_RADIUS / scale + DROPSHADOW_DIST);
				glVertex2f(0, ROBOT_RADIUS / scale - DROPSHADOW_DIST);
			glEnd();
		glPopMatrix();
	}

	/* Draw a small box with linked aprilTag ID if linked */
	if (robot->aid != -1 && aprilTagConnected) {
		glPushMatrix();
			glTranslatef(ROBOT_RADIUS / scale - 1, -ROBOT_RADIUS / scale, 0);
			glPushMatrix();
				glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
				glScalef(0.6, 0.6, 0);
				glColor3fv(color_darkgrey);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.6, 0.6, 0);
				glColor3fv(color_black);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.55, 0.55, 0);
				glColor3fv(color_white);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.45, 0.45, 0);
				glColor3fv(color_black);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glTranslatef(0, -0.3, 0);
			glColor3fv(color_white);
			textSetSize(TEXT_MED);
			textPrintf("%d", robot->aid);
		glPopMatrix();
	}

	/* Draw a L to designate that this robot is being logged */
	if (robot->log) {
		glPushMatrix();
			glTranslatef(-ROBOT_RADIUS / scale + 1, -ROBOT_RADIUS / scale, 0);
			glPushMatrix();
				glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
				glScalef(0.61, 0.6, 0);
				glColor3fv(color_darkgrey);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.61, 0.6, 0);
				glColor3fv(color_black);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glScalef(0.5, 0.5, 0);
				glColor3fv(color_white);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glTranslatef(0, -0.3, 0);
			glColor3fv(color_black);
			textSetSize(TEXT_MED);
			textPrintf("L", robot->aid);
		glPopMatrix();
	}

	if (robot->display && (!robot->blacklisted || robot->type == REMOTE)) {
		int i;
		float avg = 0;

		glPushMatrix();
			glPushMatrix();
				glTranslatef(0.05, -0.05, 0);
				glColor3fv(color_darkgrey);
				glScalef(1.4 * ROBOT_RADIUS / scale,
					1.2 * ROBOT_RADIUS / scale, 0);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glColor3fv(color_black);
				glScalef(1.4 * ROBOT_RADIUS / scale,
					1.2 * ROBOT_RADIUS / scale, 0);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glPushMatrix();
				glColor3fv(color_white);
				glScalef(1.4 * ROBOT_RADIUS / scale,
					1.2 * ROBOT_RADIUS / scale, 0);
				glScalef(0.95, 0.95, 0);
				glCallList(LIST_SQUARE);
			glPopMatrix();
			glTranslatef(-1.7 * ROBOT_RADIUS / scale + TEXT_MED,
				1.2 * ROBOT_RADIUS / scale - TEXT_MED, 0);
			glColor3fv(color_black);
			textSetSize(TEXT_SMALL);
			textSetAlignment(ALIGN_LEFT);
			if (robot->type != REMOTE)
				textPrintf("ID:%02d COM%d", robot->id, robot->port);
			else
				textPrintf("ID:%02d Host:%d", robot->id, robot->host);
			glTranslatef(0, -TEXT_SMALL, 0);
			textPrintf("Log:%s", (robot->log) ? "Yes" : "No");
			glTranslatef(0, -TEXT_SMALL, 0);
			if (robot->aid != -1)
				textPrintf("AprilTag:%d", robot->aid);
			else
				textPrintf("AprilTag:None");
			glTranslatef(0, -TEXT_SMALL, 0);
			if (robot->subnet != -1)
				textPrintf("Subnet:%d", robot->subnet);
			else
				textPrintf("Subnet:N/A");
			glTranslatef(0, -TEXT_SMALL, 0);
			if (robot->up + GRACETIME >= clock()) {
				for (i = 0; i < robot->count; i++)
					avg += robot->bps[i] / (float) robot->count;
			}
			textPrintf("B/s:%.3f", avg / 10.);
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
	GLfloat xs, ys;
	GLfloat xg, yg;

	/* Figure scale */
	if ((aprilTagX / aprilTagY) >= (AT_SCALE_X / AT_SCALE_Y)) {
		xs = AT_SCALE_X;
		ys = AT_SCALE_X * (aprilTagY / aprilTagX);
	} else {
		xs = AT_SCALE_Y * (aprilTagX / aprilTagY);
		ys = AT_SCALE_Y;
	}

	glPushMatrix();
	glTranslatef(APRILTAG_X, APRILTAG_Y, 0);

	/* Draw the grid */
	glPushMatrix();
		if (mode == GL_SELECT)
			glLoadName(APRILTAG_GRID);

		glPushMatrix();
			xg = 0;
			yg = 0;
			glColor3fv(color_grey);
			for (xg = 0; xg < 2* aprilTagX; xg += 100) {
				xi = xs * ((xg - aprilTagX) / aprilTagX);
				glPushMatrix();
					glTranslatef(xi, 0, 0);
					glBegin(GL_LINES);
						glVertex2f(0, -ys);
						glVertex2f(0, ys);
					glEnd();
				glPopMatrix();
			}
			glPushMatrix();
				glTranslatef(xs, 0, 0);
				glBegin(GL_LINES);
					glVertex2f(0, -ys);
					glVertex2f(0, ys);
				glEnd();
			glPopMatrix();

			for (yg = 0; yg < 2* aprilTagY; yg += 100) {
				yi = -ys * ((yg - aprilTagY) / aprilTagY);
				glPushMatrix();
					glTranslatef(0, yi, 0);
					glBegin(GL_LINES);
						glVertex2f(-xs, 0);
						glVertex2f(xs, 0);
					glEnd();
				glPopMatrix();
			}
			glPushMatrix();
				glTranslatef(0, -ys, 0);
				glBegin(GL_LINES);
					glVertex2f(-xs, 0);
					glVertex2f(xs, 0);
				glEnd();
			glPopMatrix();
		glPopMatrix();

		/* Draw the bounding corners */
		glColor3fv(color_darkgrey);
		glPushMatrix();
			glTranslatef(-xs, ys, 0);
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(2, 0);
			glEnd();
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(0, -2);
			glEnd();
			glTranslatef(0, TEXT_SMALL, 0);
			textSetSize(TEXT_SMALL);
			textPrintf("(0, 0)");
		glPopMatrix();
		glPushMatrix();
			glTranslatef(xs, -ys, 0);
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(-2, 0);
			glEnd();
			glBegin(GL_LINES);
				glVertex2f(0, 0);
				glVertex2f(0, 2);
			glEnd();
			textSetAlignment(ALIGN_RIGHT);
			glTranslatef(0, -2 * TEXT_SMALL, 0);
			textSetSize(TEXT_SMALL);
			textPrintf("(%.0f, %.0f)", 2 * aprilTagX, 2 * aprilTagY);
			textSetAlignment(ALIGN_LEFT);
		glPopMatrix();
	glPopMatrix();

	/* Find all active aprilTags */
	for (i = 0; i < maxAprilTag + 1; i++) {
		mutexLock(&aprilTagData[i].mutex);
		if (aprilTagData[i].active)
			activeTags[numAprilTags++] = &aprilTagData[i];
		else
			mutexUnlock(&aprilTagData[i].mutex);
	}

	/* Draw each aprilTag */
	for (i = 0; i < numAprilTags; i++) {
		glPushMatrix();
			/* Load aprilTag ID */
			if (mode == GL_SELECT)
				glLoadName(2000 + activeTags[i]->id);

			/* Scale coordinate */
			xi = xs * ((activeTags[i]->x - aprilTagX) / aprilTagX);
			yi = -ys * ((activeTags[i]->y - aprilTagY) / aprilTagY);

			glTranslatef(xi, yi, 0);
			glPushMatrix();
				glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
				glRotatef(activeTags[i]->t, 0, 0, 1);
				glRotatef(-90, 0, 0, 1);
				glScalef(0.7, 0.7, 0);
				glColor3fv(color_darkgrey);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();
			glPushMatrix();
				glRotatef(activeTags[i]->t, 0, 0, 1);
				glRotatef(-90, 0, 0, 1);
				glPushMatrix();
					glColor3fv(color_black);
					glScalef(0.7, 0.7, 0);
					glCallList(LIST_CIRCLE_FILLED);
				glPopMatrix();
				if (activeTags[i]->rid != -1) {
					glPushMatrix();
						glRotatef(180, 0, 0, 1);
						glTranslatef(0, 0.1, 0);
						textSetSize(TEXT_TINY);
						textSetAlignment(ALIGN_CENTER);
						glColor3fv(color_white);
						textPrintf("%02d", activeTags[i]->rid);
					glPopMatrix();
				} else {
					glPushMatrix();
						glColor3fv(color_white);
						glScalef(0.7, 0.7, 0);
						glBegin(GL_LINES);
							glVertex2f(0, 0);
							glVertex2f(0, -1);
						glEnd();
					glPopMatrix();
				}
			glPopMatrix();
			/* Draw the ID in the corner */
			glPushMatrix();
				glTranslatef(0.9, -0.6, 0);
				glPushMatrix();
					glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
					glColor3fv(color_darkgrey);
					glScalef(0.4, 0.4, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
				glPushMatrix();
					glColor3fv(color_black);
					glScalef(0.4, 0.4, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
				glPushMatrix();
					glColor3fv(color_white);
					glScalef(0.3, 0.3, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
				glTranslatef(0, -0.2, 0);
				glColor3fv(color_black);
				textSetSize(TEXT_SMALL);
				textSetAlignment(ALIGN_CENTER);
				textPrintf("%d", activeTags[i]->id);
				textSetAlignment(ALIGN_LEFT);
			glPopMatrix();

			if (activeTags[i]->log) {
				glPushMatrix();
					glTranslatef(0.9, 0.6, 0);
					glPushMatrix();
						glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
						glColor3fv(color_darkgrey);
						glScalef(0.4, 0.4, 0);
						glCallList(LIST_SQUARE);
					glPopMatrix();
					glPushMatrix();
						glColor3fv(color_black);
						glScalef(0.4, 0.4, 0);
						glCallList(LIST_SQUARE);
					glPopMatrix();
					glPushMatrix();
						glColor3fv(color_white);
						glScalef(0.3, 0.3, 0);
						glCallList(LIST_SQUARE);
					glPopMatrix();
					glTranslatef(0, -0.2, 0);
					glColor3fv(color_black);
					textSetSize(TEXT_SMALL);
					textSetAlignment(ALIGN_CENTER);
					textPrintf("L");
					textSetAlignment(ALIGN_LEFT);
				glPopMatrix();
			}

			/* Draw coordinate data if activated */
			if (activeTags[i]->display) {
				glPushMatrix();
					glTranslatef(-1.2, -1 - TEXT_TINY, 0);
					glColor3fv(color_black);
					textSetSize(TEXT_TINY);
					textPrintf("X:%.2f", activeTags[i]->x);
					glTranslatef(0, -TEXT_TINY, 0);
					textPrintf("Y:%.2f", activeTags[i]->y);
					glTranslatef(0, -TEXT_TINY, 0);
					textPrintf("T:%.2f", activeTags[i]->t);
				glPopMatrix();
			}
		glPopMatrix();
		mutexUnlock(&activeTags[i]->mutex);
	}
	glPopMatrix();

	/* Draw dividing bar */
	glPushMatrix();
		glColor3fv(color_darkgrey);
		glTranslatef(MAP_DIVIDE_X, 0, 0);
		glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
		glBegin(GL_LINES);
			glVertex2f(0, -GUI_HEIGHT);
			glVertex2f(0, GUI_HEIGHT / 2 - TEXT_LARGE * 2);
		glEnd();
		glColor3fv(color_black);
		glTranslatef(-DROPSHADOW_DIST, DROPSHADOW_DIST, 0);
		glBegin(GL_LINES);
			glVertex2f(0, -GUI_HEIGHT);
			glVertex2f(0, GUI_HEIGHT / 2 - TEXT_LARGE * 2);
		glEnd();
	glPopMatrix();

}

void drawAprilTagTextbox(GLenum mode)
{
	char name[18] = "AprilTag Server: ";

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

	glPushMatrix();
		glTranslatef(textWidth / 2, TEXT_MED / 2, 0);

		glPushMatrix();
			glColor3fv(color_darkgrey);
			glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
			glScalef(textWidth / 2 + 0.3, TEXT_MED / 2 + 0.3, 0);
			glCallList(LIST_SQUARE);
		glPopMatrix();

		glScalef(textWidth / 2 + 0.3, TEXT_MED / 2 + 0.3, 0);

		if (aprilTagConnected)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		glCallList(LIST_SQUARE);
		glScalef(0.95, 0.95, 0);
		glColor3fv(color_black);
		glCallList(LIST_SQUARE);
	glPopMatrix();

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

void drawButtonBox(GLfloat width)
{
	glPushMatrix();
		glTranslatef(width / 2, 0.45, 0);
		glPushMatrix();
			glColor3fv(color_white);
			glScalef(width / 2 + 0.1, TEXT_LARGE / 2, 0);
			glCallList(LIST_SQUARE);
		glPopMatrix();
	glPopMatrix();
}
void drawToolbar(GLenum mode)
{
	int i;
	char *name;
	GLfloat textWidth = TEXT_LARGE * gmf[(int) 'm'].gmfCellIncX * 2;

	glPushMatrix();
		glTranslatef(TITLE_POS_X - 0.1,
			ROBOT_START_Y + ROBOT_RADIUS - 0.75, 0);

		/* CT Button */
		if (mode == GL_SELECT)
			glLoadName(CONNECT_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == CONNECT)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("CT");

		/* RT Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);
		if (mode == GL_SELECT)
			glLoadName(HOST_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == HOSTBOT)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("RH");

		/* BL Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);
		if (mode == GL_SELECT)
			glLoadName(BLACKLIST_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == BLACKLIST)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("BL");

		/* ST Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);
		if (mode == GL_SELECT)
			glLoadName(SCONNECT_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == SCONNECT)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("SC");

		/* LG Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);
		if (mode == GL_SELECT)
			glLoadName(LOG_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == LOG)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("LG");

		/* AL Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(ATLINK_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == ATLINK)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("AL");

		/* AL Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(INFO_BUTTON);

		drawButtonBox(textWidth);

		if (clickMode == DISPLAY)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("IN");

		glTranslatef(0, -2.5, 0);

		/* TS Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(TIME_BUTTON);

		drawButtonBox(textWidth);

		if (timestamps)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("TS");

		/* OL Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(OPENLOCAL_BUTTON);

		drawButtonBox(textWidth);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("OL");

		/* OR Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(OPENREMOTE_BUTTON);

		drawButtonBox(textWidth);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("OR");

		/* KO Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(KILLALL_BUTTON);

		drawButtonBox(textWidth);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("KO");

		glTranslatef(0, -2.5, 0);

		/* ? Button */
		glTranslatef(0, -TEXT_LARGE - 0.5, 0);

		if (mode == GL_SELECT)
			glLoadName(HELP_BUTTON);

		drawButtonBox(textWidth);

		glColor3fv(color_black);
		textSetSize(TEXT_LARGE);
		textPrintf("??");

		switch (clickMode) {
		case (CONNECT): {
			name = connectName;
			break;
		}
		case (HOSTBOT): {
			name = hostName;
			break;
		}
		case (BLACKLIST): {
			name = blacklistName;
			break;
		}
		case (SCONNECT): {
			name = serialName;
			break;
		}
		case (LOG): {
			name = logName;
			break;
		}
		case (ATLINK): {
			name = aprilName;
			break;
		}
		case (DISPLAY): {
			name = infoName;
			break;
		}
		default: {
			break;
		}
		}

		textWidth = 0;
		for (i = 0; i < strlen(name); i++)
			textWidth += gmf[(int) name[i]].gmfCellIncX;

		if (aprilTagConnected)
			glTranslatef(GUI_WIDTH / 2 - textWidth, 0, 0);
		else
			glTranslatef(GUI_WIDTH - textWidth, 0, 0);

		glPushMatrix();
			glTranslatef(textWidth / 3 - 1, -0.5, 0);
			glPushMatrix();
				glTranslatef(textWidth / 3, 0.45, 0);
				glPushMatrix();
					glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
					glColor3fv(color_darkgrey);
					glScalef(textWidth / 3 + 0.1, TEXT_SMALL / 2 + 0.1, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
				glPushMatrix();
					glColor3fv(color_black);
					glScalef(textWidth / 3 + 0.1, TEXT_SMALL / 2 + 0.1, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
				glPushMatrix();
					glColor3fv(color_white);
					glScalef(textWidth / 3, TEXT_SMALL / 2, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
			glPopMatrix();
			glTranslatef(0.1, 0.25, 0);
			glColor3fv(color_black);
			textSetSize(TEXT_SMALL);
			textSetAlignment(ALIGN_LEFT);
			textPrintf(name);
		glPopMatrix();

	glPopMatrix();
}

void drawHelp()
{
	glPushMatrix();
		/* Draw bounding box */
		glPushMatrix();
			glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
			glColor3fv(color_darkgrey);
			glRectf(-GUI_WIDTH / 2 + 3,
					-GUI_HEIGHT / 2 + 3,
					GUI_WIDTH / 2 - 3,
					GUI_HEIGHT / 2 - 3);
		glPopMatrix();
		glColor3fv(color_black);
		glRectf(-GUI_WIDTH / 2 + 3,
				-GUI_HEIGHT / 2 + 3,
				GUI_WIDTH / 2 - 3,
				GUI_HEIGHT / 2 - 3);
		glColor3fv(color_white);
		glRectf(-GUI_WIDTH / 2 + 3.2,
				-GUI_HEIGHT / 2 + 3.2,
				GUI_WIDTH / 2 - 3.2,
				GUI_HEIGHT / 2 - 3.2);
		glColor3fv(color_black);
		glRectf(-GUI_WIDTH / 2 + 3.5,
				-GUI_HEIGHT / 2 + 3.5,
				GUI_WIDTH / 2 - 3.5,
				GUI_HEIGHT / 2 - 3.5);

		glTranslatef(-GUI_WIDTH / 2 + 4.5, GUI_HEIGHT / 2 - 5.5, 0);
		/* Draw help text */
		glColor3fv(color_white);
		textSetSize(TEXT_LARGE);
		textPrintf("RCC HELP");
		glTranslatef(0, -TEXT_LARGE / 2, 0);
		glBegin(GL_LINES);
			glVertex2f(0, 0);
			glVertex2f(GUI_WIDTH - 9, 0);
		glEnd();
		glTranslatef(0, -TEXT_LARGE, 0);
		textSetSize(TEXT_MED);
		textPrintf("Command List:");
		glTranslatef(TEXT_MED, -TEXT_MED, 0);
		textPrintf("CT - Connect to robot via telnet. Opens a SecureCRT window.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("RH - Make robot a radio host.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("BL - Blacklist a robot.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("SC - Connect to a robot via serial. Blacklists the robot.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("LG - Begins to log all robot data to a time-stamped file.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("AL - Click a robot and AprilTag or vice versa to pair them.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("IN - Displays additional robot data. Click on a robot or");
		glTranslatef(0, -2 * TEXT_MED, 0);
		textPrintf("TS - Toggles time-stamping of incoming robot data.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("OL - Opens a connection to all local robots.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("OR - Opens a connection to all remote robots.");
		glTranslatef(0, -TEXT_MED - 0.25, 0);
		textPrintf("KO - Kills all open Secure CRT windows.");
		glTranslatef(-TEXT_MED, -2 * TEXT_MED, 0);
		textPrintf("For more in-depth help, visit the wiki page!");
	glPopMatrix();
}
/**
 * Draw the robots and GUI features on this timed function
 */
void timerEnableDraw(int value)
{
	GLfloat length;
	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(color_white[0], color_white[1], color_white[2], 1.);

	/* Draw title, IP, port, and dividing bar */
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
		glPushMatrix();
			glColor3fv(color_darkgrey);
			glTranslatef(0, -DROPSHADOW_DIST, 0);
			glBegin(GL_LINES);
				glVertex2f(-GUI_WIDTH, 0);
				glVertex2f(GUI_WIDTH, 0);
			glEnd();
		glPopMatrix();

		glColor3fv(color_black);
		glBegin(GL_LINES);
			glVertex2f(-GUI_WIDTH, 0);
			glVertex2f(GUI_WIDTH, 0);
		glEnd();
	glPopMatrix();

	/* Draw toolbar dividing line */
	glPushMatrix();
		glTranslatef(TOOLBAR_DIVIDE_X, 0, 0);
		glPushMatrix();
			glColor3fv(color_darkgrey);
			glTranslatef(DROPSHADOW_DIST, -DROPSHADOW_DIST, 0);
			glBegin(GL_LINES);
				glVertex2f(0, -GUI_HEIGHT);
				glVertex2f(0, GUI_HEIGHT / 2 - TEXT_LARGE * 2);
			glEnd();
		glPopMatrix();

		glColor3fv(color_black);
		glBegin(GL_LINES);
			glVertex2f(0, -GUI_HEIGHT);
			glVertex2f(0, GUI_HEIGHT / 2 - TEXT_LARGE * 2);
		glEnd();
	glPopMatrix();

	drawToolbar(GL_RENDER);

	if (aprilTagConnected)
		drawAprilTags(GL_RENDER);

	drawRobots(GL_RENDER);
	drawAprilTagTextbox(GL_RENDER);

	if (showHelp)
		drawHelp();

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
