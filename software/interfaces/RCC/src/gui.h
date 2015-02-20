/**
 * gui.h
 *
 * Header file for gui.c and guiResources.c
 */
#ifndef GUI_H_
#define GUI_H_

/* GUI and drawing defines */
#define NAME 				"RCC Server"

#define WINDOW_WIDTH		800
#define WINDOW_HEIGHT 		600

#define GUI_WIDTH 			40.
#define GUI_HEIGHT 			30.
#define ASPECT 				(GUI_WIDTH / GUI_HEIGHT)

#define GUI_BUFFER			512

#define PICK_DELTA			3.0

#define DROPSHADOW_DIST		0.04

#define DRAW_DELAY			16
#define AT_DRAW_DELAY		32

#define TITLE_POS_X			-19.5
#define TITLE_POS_Y			13.5

#define TOOLBAR_DIVIDE_X	(TITLE_POS_X + 2)
#define MAP_DIVIDE_X		0

/* Robot drawing defines */
#define ROBOT_RADIUS		2.
#define BOX_RADIUS			2.5
#define OUTER_RADIUS		2.1
#define INNER_RADIUS		1.9
#define HOST_RADIUS			2.3

#define ROBOT_START_X		(TOOLBAR_DIVIDE_X + ROBOT_RADIUS + 1)
#define ROBOT_START_Y		10.

#define ROBOT_END_X			(-ROBOT_START_X + 1)

#define ROBOT_STEP_X		6.
#define ROBOT_STEP_Y		5.

#define SCALE_LARGE			1.
#define SCALE_MED			1.5
#define SCALE_SMALL			2.
#define SCALE_TINY			3.

#define BUTTONSPACE			-TEXT_LARGE - 0.2
#define BUTTONBLOCKSPACE	-2.0

#define TOASTER_POP_TIME	750

/* AprilTag drawing defines */
#define APRILTAG_X			10
#define APRILTAG_Y			-1

#define AT_SCALE_X			9.
#define AT_SCALE_Y			12.

/* Text defines */
#define TEXT_TINY			0.5
#define TEXT_SMALL			0.65
#define TEXT_MED 			0.85
#define TEXT_NORMAL			1.0
#define TEXT_LARGE			1.35

#define ALIGN_CENTER 		1
#define ALIGN_LEFT 			0
#define ALIGN_RIGHT 		2

#define LINE_WIDTH 			2
#define LINE_WIDTH_MEDIUM	1
#define LINE_WIDTH_SMALL 	0.25

/* Resource defines */
#define COLOR_SIZE			3
#define NUM_DRAWING_LISTS	2
#define LIST_CIRCLE_FILLED	(drawingListBase + 0)
#define LIST_SQUARE			(drawingListBase + 1)

#define POINT_SIZE 			10
#define DISK_SLICES			60
#define DISK_LOOPS			3

#define PI					3.14159
#define MAX_PROCESSES		1024

/* Textbox defines */
#define MAX_TEXTBOX_LENGTH	21
#define TEXTBOX_ID			10000
#define TEXTBOX_NAME		10001

/* Click modes */
#define CONNECT		0
#define BLACKLIST	1
#define SCONNECT	2
#define ATLINK		3
#define HOSTBOT		4
#define LOG			5
#define DISPLAY		6
#define ATSAT		7
#define GUI			8

#define CONNECT_BUTTON		1001
#define BLACKLIST_BUTTON	1002
#define SCONNECT_BUTTON		1003
#define ATLINK_BUTTON		1004
#define OPENLOCAL_BUTTON	1005
#define OPENREMOTE_BUTTON	1006
#define KILLALL_BUTTON		1007
#define HOST_BUTTON			1008
#define LOG_BUTTON			1009
#define HELP_BUTTON			1010
#define INFO_BUTTON			1011
#define TIME_BUTTON			1012
#define LOGST_BUTTON		1013
#define GUI_BUTTON			1014
#define HDATA_BUTTON		1015
#define SAT_BUTTON			1016

#define APRILTAG_GRID		1501

/**
 * Textbox buffer and state information
 */
struct textbox {
	int isActive;
	unsigned int length;
	int index;
	CRITICAL_SECTION mutex;									// Mutex
	char message[MAX_TEXTBOX_LENGTH];
};

extern char tbuffer[64];

extern struct textbox aprilTagURL;	// AprilTag IP textbox

extern GLYPHMETRICSFLOAT gmf[256];	// Character information
extern GLint drawingListBase;		// Drawing primitives

/* Drawing colors */
extern const float color_red[COLOR_SIZE];
extern const float color_lightred[COLOR_SIZE];
extern const float color_black[COLOR_SIZE];
extern const float color_grey[COLOR_SIZE];
extern const float color_darkgrey[COLOR_SIZE];
extern const float color_white[COLOR_SIZE];

extern const GLfloat *color_array[NUMROBOT_POINTS];
extern const GLfloat *lightcolor_array[NUMROBOT_POINTS];

/* GUI functions and drawing functions */
void display();
void reshape(int w, int h);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void readChar(char character);
void processHits(GLint hits, GLuint buffer[]);
int openClientConnection(int robotID, int aprilTagID);
int directConnect(int robotID);
void killSecureCRT();
void aspectHandle(int w, int h);
void drawRobots(GLenum mode);
void drawRobot(GLfloat x, GLfloat y, struct commCon *robot, GLfloat scale);
void drawAprilTags(GLenum mode);
void drawAprilTagTextbox(GLenum mode);
void drawToolbar(GLenum mode);
void drawHelp();
void timerEnableDraw(int value);
void special(int key, int x, int y);
void setToaster(char *text);
void drawToaster();

void guiInit();

/* Text functions */
GLvoid textInit(GLvoid);
GLfloat textPrintf(const char *fmt, ...);
void textSetAlignment(int ta);
void textSetSize(GLfloat size);

/* Resource functions */
void drawInit();

/* Command functions */
void telnetConnect(int robotID);
void hostRobot(int robotID);
void blacklist(int robotID);
void commConnect(int robotID);
void logRobot(int robotID);
void logAprilTag(int aprilTagID);
void openLocalConnections();
void openRemoteConnections();
void showRobotInfo(int robotID);
void showAprilTagInfo(int robotID);
int guiConnect(int robotID);

#endif
