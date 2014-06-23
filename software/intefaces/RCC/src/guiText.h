#ifndef TEXT_H_
#define TEXT_H_

#define TEXT_SMALL 0.65
#define TEXT_MED 	0.85
#define TEXT_LARGE	1.35

#define ALIGN_CENTER 	1
#define ALIGN_LEFT 	0
#define ALIGN_RIGHT 	2

#define LINE_WIDTH 					2
#define LINE_WIDTH_MEDIUM				1
#define LINE_WIDTH_SMALL 				0.25

extern GLYPHMETRICSFLOAT gmf[256];

GLvoid textInit(GLvoid);
GLvoid textPrintf(const char *fmt, ...);
void textSetAlignment(int ta);
void textSetSize(GLfloat size);

#endif
