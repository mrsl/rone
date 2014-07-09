/**
 * rcc.h
 *
 * Main header file for all files
 */
#ifndef RCC_H_
#define RCC_H_

#define _WIN32_WINNT 	0x0501

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Windows includes */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
#include <psapi.h>

/* OpenGL */
#include "GL/freeglut.h"

/* File includes */
#include "util.h"
#include "server.h"
#include "serial.h"
#include "registry.h"
#include "robotLink.h"
#include "gui.h"

extern int port;				// Port we are listening on
extern int verbose;				// Verbose output?
extern GLfloat aprilTagX;		// AprilTag window parameters
extern GLfloat aprilTagY;
extern char logDir[MAX_PATH];	// Logfile directory

#endif
