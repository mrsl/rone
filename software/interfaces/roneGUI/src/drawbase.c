/*
 * drawbase.c
 *
 * OpenGL drawing functions.
 *
 *  Created on: May 23, 2011
 *      Author: nathan
 */

#include "gui.h"


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

/* Initialize stuff for drawing */
void
drawInit(void)
{
	glPointSize(POINT_SIZE);
	glLineWidth(LINE_WIDTH);
	glClearColor(1.0, 1.0, 1.0, 1.0);

	/* Enable antialias */
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	//TODO: Polygon antialias sucks! Use multisampling?
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

	glNewList(LIST_SEMICIRCLE, GL_COMPILE);
		gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);
		gluPartialDisk(qobj, 0.0, 1.0, DISK_SLICES, DISK_LOOPS, 0, ANGLE_180);
	glEndList();

	glNewList(LIST_ANNULUS, GL_COMPILE);
		gluQuadricDrawStyle(qobj, GLU_FILL);
		gluDisk(qobj, 1.0 - ANNULUS_THICK, 1.0, DISK_SLICES, DISK_LOOPS);
		gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);
		gluDisk(qobj, 1.0 - ANNULUS_THICK, 1.0, DISK_SLICES, DISK_LOOPS);

	glEndList();

	glNewList(LIST_ANNULUS_SECT, GL_COMPILE);
		gluQuadricDrawStyle(qobj, GLU_FILL);
		gluPartialDisk(qobj, 1.0 - ANNULUS_THIN, 1.0 + ANNULUS_THIN,
			DISK_SLICES, DISK_LOOPS, -BUMP_ANGLE / 2, BUMP_ANGLE);
		gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);
		gluPartialDisk(qobj, 1.0 - ANNULUS_THIN, 1.0 + ANNULUS_THIN,
			DISK_SLICES, DISK_LOOPS, -BUMP_ANGLE / 2, BUMP_ANGLE);
	glEndList();

	glNewList(LIST_AXES2D, GL_COMPILE);
		glBegin(GL_LINES);
		glVertex2f(-1, 0);
		glVertex2f(1, 0);
		glVertex2f(0, -1);
		glVertex2f(0, 1);
		glEnd();
	glEndList();

	glNewList(LIST_SQUARE, GL_COMPILE);
		glRecti(-1, -1, 1, 1);
		glBegin(GL_LINE_LOOP);
		glVertex2f(-1, -1);
		glVertex2f(1, -1);
		glVertex2f(1, 1);
		glVertex2f(-1, 1);
		glEnd();
	glEndList();

	glNewList(LIST_TRIANGLE, GL_COMPILE);
		glBegin(GL_TRIANGLES);
			glVertex2i(1, -1);
			glVertex2i(-1, -1);
			glVertex2i(0, 1);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2i(1, -1);
			glVertex2i(-1, -1);
			glVertex2i(0, 1);
		glEnd();
	glEndList();

	glNewList(LIST_TRIANGLE_HALF, GL_COMPILE);
		glBegin(GL_TRIANGLES);
		glVertex2i(1, 0);
		glVertex2i(-1, 0);
		glVertex2i(0, 1);
		glEnd();

		glBegin(GL_LINE_LOOP);
		glVertex2i(1, 0);
		glVertex2i(-1, 0);
		glVertex2i(0, 1);
		glEnd();
	glEndList();

	glNewList(LIST_PUSHBUTTON_CENTRE, GL_COMPILE);
		/* First draw the initial disk. */
		gluQuadricDrawStyle(qobj, GLU_FILL);
		gluDisk(qobj, 0.0, PUSHBUTTON_RADIUS, DISK_SLICES, DISK_LOOPS);

		/* Then draw a silhouette to remove the jagged edges. */
		gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);
		gluDisk(qobj, 0.0, PUSHBUTTON_RADIUS, DISK_SLICES, DISK_LOOPS);
	glEndList();

	glNewList(LIST_PUSHBUTTON_OUTLINE, GL_COMPILE);
		/* First draw the initial disk. */
		gluQuadricDrawStyle(qobj, GLU_FILL);
		gluDisk(qobj, PUSHBUTTON_RADIUS - PUSHBUTTON_RING_WIDTH,
			PUSHBUTTON_RADIUS, DISK_SLICES, DISK_LOOPS);

		/* Then draw a silhouette to remove the jagged edges. */
		gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);
		gluDisk(qobj, PUSHBUTTON_RADIUS - PUSHBUTTON_RING_WIDTH,
			PUSHBUTTON_RADIUS, DISK_SLICES, DISK_LOOPS);
	glEndList();

	glNewList(LIST_IR_COMPONENTS, GL_COMPILE);
		/* Draw twice to smooth jagged edges */
		glBegin(GL_TRIANGLES);
			glVertex2f(1.2, -1.2);
			glVertex2f(-1.2, -1.2);
			glVertex2f(0, 1.8);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2f(1, -1);
			glVertex2f(-1, -1);
			glVertex2f(0, 1.6);
		glEnd();
		glBegin(GL_TRIANGLES);
			glVertex2f(1.2, -1.2);
			glVertex2f(-1.2, -1.2);
			glVertex2f(0, 1.8);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2f(1, -1);
			glVertex2f(-1, -1);
			glVertex2f(0, 1.6);
		glEnd();
	glEndList();

	glNewList(LIST_GRIPPER_CLAWS, GL_COMPILE);
		glBegin(GL_TRIANGLES);
			glVertex2i(1, 0);
			glVertex2i(-1, 0);
			glVertex2i(2, 3);
		glEnd();
		glBegin(GL_TRIANGLES);
			glVertex2i(-1, 0);
			glVertex2i(2, 3);
			glVertex2i(-2, 3);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2f(0.8, 0);
			glVertex2f(-0.8,0);
			glVertex2f(1.8, 2.8);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2f(-0.8, 0);
			glVertex2f(1.8, 2.8);
			glVertex2f(-1.8, 2.8);
		glEnd();
	glEndList();

	glNewList(LIST_GRIPPER_CLAWS, GL_COMPILE);
		glBegin(GL_TRIANGLES);
			glVertex2i(1, 0);
			glVertex2i(-1, 0);
			glVertex2i(2, 3);
		glEnd();
		glBegin(GL_TRIANGLES);
			glVertex2i(-1, 0);
			glVertex2i(2, 3);
			glVertex2i(-2, 3);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2f(0.8, 0);
			glVertex2f(-0.8,0);
			glVertex2f(1.8, 2.8);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex2f(-0.8, 0);
			glVertex2f(1.8, 2.8);
			glVertex2f(-1.8, 2.8);
		glEnd();
	glEndList();

	gluDeleteQuadric(qobj);
}


