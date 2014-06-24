/*
 * gui.h
 *
 *  Created on: May 23, 2011
 *      Author: nathan
 */

#ifndef GUI_H_
#define GUI_H_

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <windows.h>

//#include <pthread.h>

/*
 * NOTE: IMPORTANT!!!
 *
 * Changed the include to GL/freeglut.h from freeglut.h
 * to put freeglut.h with the other OpenGL imports.
 *
 * jmclurkin: I have no idea what this means.  freeglut.h is in the
 * OpenGLDemo project (not a good location, where is the freeglut project?)
 * We don't want to use the system version of this file, we want the one for
 * our static lib. changing this back so it builds.
 */
//#include <GL/freeglut.h>
#include "GL/freeglut.h"

#include "params.h"
#include "io.h"
#include "drawing.h"
#include "registry.h"

/**
 * gui.c
 */
#define MSG_SIZE       256
#define CALLBACK_DELAY 30

/* Variables. */
extern FILE *in;
extern FILE *data;
extern int IRtimer;
extern bool serialConnected;

/* Function prototypes. */
void guiPreInit(void);
void guiInit(void);
void display(void);
void keyboard(unsigned char key, int x, int y);
void special(int key, int x, int y);
void mouse(int button, int state, int x, int y);
void reshape(int w, int h);
void idle(void);
void timerQueryRobot(int value);
void timerTransmitIR(int value);
void timerMotorCommands(int value);
void timerReadData(int value);
void writeToINFile();
void writeToOUTFile(char *msg, int size);

/**
 * util.c
 */
#define PI 3.14159
float clamp(float num, float min, float max);
void strins(char* str, char c, int index);
void strdel(char* str, int index);
void differential(int newData, int *oldDataPtr, float alpha);
void differentialf(float newData, float *oldDataPtr, float alpha);
void differentiala(float newData, float *oldDataPtr);

/* serial.c */
int serialConnect(HANDLE *hSerialPtr, int comPort);
int setCommStateTimeouts(HANDLE hSerial);
//int getData(HANDLE file, outputData *data);
int serialSendData(HANDLE file, char *);

int serialDataHandler(HANDLE file);
int serialMessageGet(char *buffer, int buflen);
GLvoid cprintf(const char *fmt, ...);

/* text.c */
#define TEXT_SMALL 0.65
#define TEXT_MED 	0.85
#define TEXT_LARGE	1.35

extern GLYPHMETRICSFLOAT gmf[256];
#define ALIGN_CENTER 	1
#define ALIGN_LEFT 	0
#define ALIGN_RIGHT 	2

GLvoid textInit(GLvoid);
GLvoid textPrintf(const char *fmt, ...);
void textSetAlignment(int ta);
void textSetSize(GLfloat size);

/* input.c */
#define	 BUFSIZE 512

void inputInit();
void readChar(char character);
void readSpecialChar(char character);
void processHits(GLint hits, GLuint buffer[]);


/* output.c */
void outputInit(outputData *out);
int updateOutput(outputData *output);

/* drawing.c */
//TODO: Move to drawing.c / output.c

void drawInit(void);
void drawOutput(outputData *data);
void drawInput(GLenum mode);

// TODO: Make so that only needed in gui.c?
extern HANDLE hSerial;
extern int comNumber;

/*
 * Mutex to prevent multiple threads from sending to the serial
 * port at the same time.
 */
//extern pthread_mutex_t serialPortMutex;

/* IR transmit timer function */
void timerTransmitIR(int value);

void guiInit(void);
#endif /* GUI_H_ */



