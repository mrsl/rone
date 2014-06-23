#include "rcc.h"

const GLfloat color_red[COLOR_SIZE] = {1.0, 0.0, 0.0};
const GLfloat color_green[COLOR_SIZE] = {0.0, 1.0, 0.0};
const GLfloat color_blue[COLOR_SIZE] = {0.0, 0.0, 1.0};

const GLfloat color_cyan[COLOR_SIZE] = {0.0, 1.0, 1.0};
const GLfloat color_magenta[COLOR_SIZE] = {1.0, 0.0, 1.0};
const GLfloat color_yellow[COLOR_SIZE] = {1.0, 1.0, 0.0};

const GLfloat color_black[COLOR_SIZE] = {0.0, 0.0, 0.0};
const GLfloat color_lightgray[COLOR_SIZE] = {0.75, 0.75, 0.75};
const GLfloat color_gray[COLOR_SIZE] = {0.5, 0.5, 0.5};
const GLfloat color_darkgray[COLOR_SIZE] = {0.25, 0.25, 0.25};
const GLfloat color_white[COLOR_SIZE] = {1.0, 1.0, 1.0};

GLint drawingListBase;

void
drawInit()
{
	glPointSize(POINT_SIZE);
	glLineWidth(LINE_WIDTH);
	glClearColor(1.0, 1.0, 1.0, 1.0);

	/* Enable antialias */
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	/* Create quadratic object */
	GLUquadricObj *qobj;
	qobj = gluNewQuadric();

	drawingListBase = glGenLists(NUM_DRAWING_LISTS);

	glNewList(LIST_CIRCLE_FILLED, GL_COMPILE);
		gluQuadricDrawStyle(qobj, GLU_FILL);
		gluDisk(qobj, 0.0, 1.0, DISK_SLICES, DISK_LOOPS);
		gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);
		gluDisk(qobj, 0.0, 1.0, DISK_SLICES, DISK_LOOPS);
	glEndList();

	gluDeleteQuadric(qobj);
}
