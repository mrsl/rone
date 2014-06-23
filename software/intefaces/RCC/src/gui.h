#ifndef GUI_H_
#define GUI_H_

#include "guiText.h"
#include "guiBase.h"

#define NAME 			"RCC Server"

#define WINDOW_WIDTH	800
#define WINDOW_HEIGHT 	600

#define GUI_WIDTH 		40.
#define GUI_HEIGHT 		30.
#define ASPECT 			(GUI_WIDTH / GUI_HEIGHT)

#define GUI_BUFFER		512

#define PICK_DELTA		3.0

#define DRAW_DELAY		100

#define ROBOT_RADIUS	2.
#define OUTER_RADIUS	2.1
#define INNER_RADIUS	1.9
#define HOST_RADIUS		2.3

#define TITLE_POS_X		-19.5
#define TITLE_POS_Y		13.5

#define ROBOT_START_LX	-15.
#define ROBOT_START_LY	8.
#define ROBOT_START_RX	ROBOT_START_LX
#define ROBOT_START_RY	-6.

#define ROBOT_STEP_X	6.
#define ROBOT_STEP_Y	5.

#define SCALE_LARGE		1.
#define SCALE_MED		1.5
#define SCALE_SMALL		2.
#define SCALE_TINY		3.


#define PI				3.14159

extern int redraw;

void display();
void reshape(int w, int h);
void mouse(int button, int state, int x, int y);
void processHits(GLint hits, GLuint buffer[]);
void aspectHandle(int w, int h);
void drawRobots(GLenum mode);
void drawRobot(GLfloat x, GLfloat y, struct commCon *robot, GLfloat scale);
void timerEnableDraw(int value);
void guiInit(int argc, char* argv[]);

#endif
