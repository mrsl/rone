/**
 * rcc.h
 */
#ifndef RCC_H_
#define RCC_H_

/* Standard includes */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <strings.h>
#include <stddef.h>
#include <math.h>

/* Windows includes */
#include <winsock.h>
#include <windows.h>

/* Open GL */
#include "GL/freeglut.h"

/* File includes */
#include "server.h"
#include "util.h"
#include "serial.h"
#include "registry.h"
#include "robotLink.h"
#include "file.h"
#include "gui.h"

extern int port;
extern int verbose;

#endif
