/**
 * rcc.h
 *
 * Main header file for all files
 */
#ifndef RCC_H_
#define RCC_H_

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

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
#include "gui.h"

extern int port;
extern int verbose;

#endif
