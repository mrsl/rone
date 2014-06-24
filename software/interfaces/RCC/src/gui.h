#ifndef GUI_H_
#define GUI_H_

/* GUI and drawing defines */
#define NAME 			"RCC Server"

#define WINDOW_WIDTH	800
#define WINDOW_HEIGHT 	600

#define GUI_WIDTH 		40.
#define GUI_HEIGHT 		30.
#define ASPECT 			(GUI_WIDTH / GUI_HEIGHT)

#define GUI_BUFFER		512

#define PICK_DELTA		3.0

#define DRAW_DELAY		100

#define TITLE_POS_X		-19.5
#define TITLE_POS_Y		13.5

/* Robot drawing defines */
#define ROBOT_RADIUS	2.
#define OUTER_RADIUS	2.1
#define INNER_RADIUS	1.9
#define HOST_RADIUS		2.3

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

/* Text defines */
#define TEXT_SMALL 0.65
#define TEXT_MED 	0.85
#define TEXT_LARGE	1.35

#define ALIGN_CENTER 	1
#define ALIGN_LEFT 		0
#define ALIGN_RIGHT 	2

#define LINE_WIDTH 			2
#define LINE_WIDTH_MEDIUM	1
#define LINE_WIDTH_SMALL 	0.25

/* Resource defines */
#define COLOR_SIZE			3
#define NUM_DRAWING_LISTS	1
#define LIST_CIRCLE_FILLED	(drawingListBase + 0)

#define POINT_SIZE 	10
#define DISK_SLICES 60
#define DISK_LOOPS	3

#define PI				3.14159

extern GLYPHMETRICSFLOAT gmf[256];

extern GLint drawingListBase;

extern const float color_red[COLOR_SIZE];
extern const float color_black[COLOR_SIZE];
extern const float color_white[COLOR_SIZE];

/* GUI functions and drawing functions */
void display();
void reshape(int w, int h);
void mouse(int button, int state, int x, int y);
void processHits(GLint hits, GLuint buffer[]);
void aspectHandle(int w, int h);
void drawRobots(GLenum mode);
void drawRobot(GLfloat x, GLfloat y, struct commCon *robot, GLfloat scale);
void timerEnableDraw(int value);
void guiInit();

/* Text functions */
GLvoid textInit(GLvoid);
GLvoid textPrintf(const char *fmt, ...);
void textSetAlignment(int ta);
void textSetSize(GLfloat size);

/* Resource functions */
void drawInit();

#endif
