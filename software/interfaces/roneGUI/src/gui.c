/*
 * gui.c
 *
 * The main file to be executed.
 *
 *  Created on: May 16, 2011
 *      Author: Nathan Alison
 *      Updated by: Zachary Kingston
 */

#include "gui.h"

/* Data Values */
outputData outData = {0}; /* data to store output values from the robot. */
HANDLE hSerial;           /* handle to the serial port connected to robot. */
bool serialConnected = 0;

/* Timers */
int MotorTimer = 0;
int IRtimer = 0;

/* Robot COM port. */
int comNumber = 0;

/* Redraw the GUI? */
int redraw = 0;

/* File variables. */
FILE *out;
FILE *in;
FILE *data;

/**
 * Initialize one time only drawing resources
 */
void
guiPreInit(void)
{
	textInit();
	drawInit();
}

/**
 * Initialize the necessary components of the GUI.
 */
void
guiInit(void)
{
	inputInit();
	outputInit(&outData);
	serialConnected = serialConnect(&hSerial, comNumber);
}

/**
 * Draws the GUI given current data.
 */
void
display(void)
{
	/* Clear the GUI. */
	glClear(GL_COLOR_BUFFER_BIT);

	/* Draw input and output. */
	drawOutput(&outData);
	drawInput(GL_RENDER);

	/* Draw the title. */
	glPushMatrix();
		glTranslatef(TITLE_POS_X, TITLE_POS_Y, 0);
		glColor3fv(color_black);
		textSetAlignment(ALIGN_CENTER);
		textSetSize(TEXT_LARGE);
		textPrintf("Rone GUI");
	glPopMatrix();

	/* Update */
	glutSwapBuffers();
	glutPostRedisplay();
}

/**
 * Standard keyboard input handler.
 */
void
keyboard(unsigned char key, int x, int y)
{
	/* Exit on escape key. */
	if (key == 27)
		exit(0);
	else
		readChar(key);

	/* Update GUI. */
	glutPostRedisplay();
}

/**
 * Special key input handler.
 */
void
special(int key, int x, int y)
{
	switch (key) {
	/* Close and reestablish serial connection on F5. */
	case GLUT_KEY_F5: {
		CloseHandle(hSerial);
		inputInit();
		outputInit(&outData);
		serialConnected = serialConnect(&hSerial, comNumber);
		break;
	}
	default: {
		readSpecialChar(key);
		break;
	}
	}

	/* Update GUI. */
	glutPostRedisplay();
}

/**
 * Mouse input handler.
 */
void
mouse(int button, int state, int x, int y)
{
	GLuint selectBuf[BUFSIZE];
	GLint hits;
	GLint viewport[4];

	/* Until mouse has gone up, don't process input again. */
	if (button != GLUT_LEFT_BUTTON || state != GLUT_DOWN)
		return;

	glGetIntegerv(GL_VIEWPORT, viewport);
	GLint w = viewport[2];
	GLint h = viewport[3];

	glSelectBuffer(BUFSIZE, selectBuf);
	glRenderMode(GL_SELECT);

	glInitNames();
	glPushName(~0);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
		glLoadIdentity();

		gluPickMatrix((GLdouble) x, (GLdouble) (h - y),
			PICK_DELTA, PICK_DELTA, viewport);

		/* Handle skewed aspect ratios. */
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

		/* Redraw input elements of GUI. */
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
			drawInput(GL_SELECT);
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
 * Reshape handler.
 *
 * Keep the same aspect ratio.
 */
void
reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

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

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/**
 * Query the robot every 16ms.
 *
 * Query the robot at a rate approximately equal to 60HZ.
 */
void
timerQueryRobot(int value)
{
	if (redraw == 1)
		glutPostRedisplay();

	if (serialConnected)
		cprintf("sv\r");
	
	/* Sets up the GUI to query the robot every 16ms (QUERY_DELAY). */
	glutTimerFunc(QUERY_DELAY, timerQueryRobot, value);
}

/**
 * Query the robot for nbr data
 *
 * Query the robot at a rate approximately equal to 30HRZ
 */
void
timerNBRQueryRobot(int value)
{
	if (redraw == 1)
		glutPostRedisplay();

	if (serialConnected)
		cprintf("sn\r");

	/* Sets up the GUI to query the robot every NBR_QUERY_DELAY. */
	glutTimerFunc(NBR_QUERY_DELAY, timerNBRQueryRobot, value);
}

/**
 * Query the robot for gripper data
 *
 * Query the robot at a rate approximately equal to 30HRZ
 */
void
timerGQueryRobot(int value)
{
	if (redraw == 1)
		glutPostRedisplay();

	if (serialConnected)
		cprintf("sg\r");

	/* Sets up the GUI to query the robot every NBR_QUERY_DELAY. */
	glutTimerFunc(G_QUERY_DELAY, timerGQueryRobot, value);
}

/*
 * Transmit the motor command every ______ms. Do this constantly
 * because otherwise it will stop the motors.
 */
void
timerMotorCommands(int value)
{
	if (serialConnected) {
		if (velOrPwm == PWM) {
			cprintf("rcp l,%02X\r", (uint8)(int8)motorLeft);
			cprintf("rcp r,%02X\r", (uint8)(int8)motorRight);
		} else {
			/* TODO: Make a four digit hex number rather than 8. */
			cprintf("rcv l,%04X\r", (uint16)(int16)motorLeft);
			cprintf("rcv r,%04X\r", (uint16)(int16)motorRight);
		}
	}

	/* Initialize timer loop. */
	glutTimerFunc(MOTOR_DELAY, timerMotorCommands, value);
}

void
timerReadData(int value)
{
	if (serialConnected) {
		serialDataHandler(hSerial);
		redraw = updateOutput(&outData);
	}
	/* Initialize timer loop. */
	glutTimerFunc(READ_DELAY, timerReadData, value);
}

void
timerAutoSelect(int value)
{
	int i;
	struct regData data;

	if (autoselect && !serialConnected) {
		enumCommNames(&data);

		for (i = 0; i < data.n; i++) {
			HANDLE tSerial;
			int tCom = data.ports[i];

			if (serialConnect(&tSerial, tCom)) {
				if (serialDataHandler(tSerial) != 0) {
					CloseHandle(tSerial);
					CloseHandle(hSerial);
					comNumber = tCom;
					guiInit();
				}
			}
		}
	}

	/* Initialize timer loop. */
	glutTimerFunc(AUTOSELECT_DELAY, timerAutoSelect, value);
}

/**
 * Main function
 *
 * Initializes OpenGL and runs the main loop.
 */
int
main(int argc, char* argv[])
{
	int i;
	for (i = 1; i < argc; i++) {
		if (i + 1 != argc) {
			if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
				if (sscanf(argv[i + 1], "%d", &comNumber) != 1) {
					break;
				}
				i++;
				continue;
			}
		}
	}

	argc = 0;
	glutInit(&argc, NULL);

	/* Remove the terminal that would spawn behind the GUI. */
	FreeConsole();
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("Rone GUI");

	/* Initialize the GUI. */
	guiPreInit();
	guiInit();

	/* Bind functions to OpenGL functions. */
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutSpecialFunc(special);
	glutReshapeFunc(reshape);

	/* Initialize query timer loop. */
	glutTimerFunc(QUERY_DELAY, timerQueryRobot, 0);
	/* Initialize the nbr query time loop. */
	glutTimerFunc(NBR_QUERY_DELAY, timerNBRQueryRobot, 0);
	/* Initialize the gripper query time loop */
	glutTimerFunc(G_QUERY_DELAY, timerGQueryRobot, 0);
	/* Initialize data read timer loop. */
	glutTimerFunc(READ_DELAY, timerReadData, 0);
	/* Initialize motor timer loop. */
	glutTimerFunc(MOTOR_DELAY, timerMotorCommands, 0);
	/* Initialize auto-select loop. */
	glutTimerFunc(AUTOSELECT_DELAY, timerAutoSelect, 0);

	glutMainLoop();
	return (0);
}
