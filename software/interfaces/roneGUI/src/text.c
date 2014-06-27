/*
 * text.c
 *
 * OpenGL outline fonts using WGL.
 *
 *  Created on: May 26, 2011
 *      Author: nathan
 */

#include "gui.h"

GLuint base, baseOut;
GLYPHMETRICSFLOAT gmf[256];
GLYPHMETRICSFLOAT gmfOut[256];

static int alignment = ALIGN_LEFT;
static GLfloat textWidth = TEXT_MED, textHeight = TEXT_MED;

GLvoid textInit(GLvoid)
{
	HDC hdc = wglGetCurrentDC();
	HFONT font;

	font = CreateFont(-12,
					  0,
					  0,
					  0,
					  FW_ULTRALIGHT,
					  FALSE,
					  FALSE,
					  FALSE,
					  ANSI_CHARSET,
					  OUT_TT_PRECIS,
					  CLIP_DEFAULT_PRECIS,
					  ANTIALIASED_QUALITY,
					  FF_DONTCARE|DEFAULT_PITCH,
					  "Lucida Console");

	SelectObject(hdc, font);

	base = glGenLists(256);
	baseOut = glGenLists(256);

	/* Create the font and font outline (for antialias) */
	wglUseFontOutlines(hdc,
					   0,
					   255,
					   base,
					   0.0,
					   0.0,
					   WGL_FONT_POLYGONS,
					   gmf);
	wglUseFontOutlines(hdc,
					   0,
					   255,
					   baseOut,
					   0.0,
					   0.0,
					   WGL_FONT_LINES,
					   gmfOut);

	DeleteObject(font);
}

GLvoid textPrintf(const char *fmt, ...)
{
	GLfloat length = 0;
	char text[256] = {0};
	va_list ap;

	if (fmt == NULL) {
		return;
	}

	va_start(ap, fmt);
		vsprintf(text, fmt, ap);
	va_end(ap);

	unsigned int loop;
	for (loop=0; loop<(strlen(text)); loop++)
	{
		length += gmf[(int)text[loop]].gmfCellIncX;
	}
	glPushMatrix();
		glScalef(textWidth, textHeight, 0);
		switch(alignment)
		{
		case ALIGN_RIGHT:
			glTranslatef(-length, 0.0, 0.0);
			break;
		case ALIGN_CENTER:
			glTranslatef(-length/2, 0.0, 0.0);
			break;
		case ALIGN_LEFT:
			break;
		}

		glPushAttrib(GL_LIST_BIT);
			glPushMatrix();
				glListBase(base);
				glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);
			glPopMatrix();

			if (textWidth != TEXT_LARGE) {
				glLineWidth(LINE_WIDTH_SMALL);
			}

			glPushMatrix();
				glListBase(baseOut);
				glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);
			glPopMatrix();

			glLineWidth(LINE_WIDTH);
		glPopAttrib();
	glPopMatrix();
}

void textSetAlignment(int ta) {
	alignment = ta;
}

void textSetSize(GLfloat size) {
	textWidth = size;
	textHeight = size;
}

