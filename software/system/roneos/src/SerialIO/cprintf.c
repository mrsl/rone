/*
 * @file cprintf.c
 * @brief Light-weight formatted printing to a remote print buffer that is routed through the
 * robot network.
 *
 * @since 2001
 * @author James McLurkin
 * @copyright iRobot 2001
 */

/******** Include Files ********/
//#include <string.h>
//#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "roneos.h"
#include "snprintf.h"

/******** Variables ********/
static osSemaphoreHandle cprintfMutex;
static boolean cprintfOSInit = FALSE;

static char cfprintfBuffer[CPRINTF_TEXT_STRING_SIZE];
static char* cfprintfBufferPtr = cfprintfBuffer;
//static uint8 cfprintfBufferLength = 0;

//TODO this shold be static, but the logger uses it too.
uint8 CRLFMode = CPRINTF_CRLF_CRLF;

//80 bytes of text:
//000000000011111111112222222222333333333344444444445555555555666666666677777777778888888888

/******** Functions ********/

/*
 *	@brief Serves the same purpose as formatted output string for the robot.
 *
 *	Processes the input string into an output string rone understands.
 *	If the input string is too large, cfprintfOverRunError is set to TRUE.
 *
 *	@returns void
 */
void cprintf(char *format, ...) {
	va_list arguments;

	va_start(arguments, format);

	int32 potentialChars;
	boolean cfprintfOverRunError = FALSE;
	char* charPtr;

	if (cprintfOSInit) {
		osSemaphoreTake(cprintfMutex, portMAX_DELAY);
	}
	/* Process the input string and store the output in the 'outStringFormatted' */
	/* note that vsnprintf returns the number of characters that would have been
	 * written to the buffer if it were large enough. */
	potentialChars = vsnprintf(cfprintfBufferPtr, CPRINTF_TEXT_STRING_SIZE - 1, format, arguments);
	if (potentialChars >= (CPRINTF_TEXT_STRING_SIZE - 1)) {
		cfprintfOverRunError = TRUE;
	}

	charPtr = cfprintfBufferPtr;
	while (*charPtr != '\0') {
		if (*charPtr == '\n') {
			// change '\n' into the current CRLF mode
			switch (CRLFMode) {
			case CPRINTF_CRLF_CR:
				sputchar('\r');
				break;
			case CPRINTF_CRLF_CRLF:
				sputchar('\r');
				sputchar('\n');
				break;
			case CPRINTF_CRLF_LFCR:
				sputchar('\n');
				sputchar('\r');
				break;
			case CPRINTF_CRLF_LF:
			default:
				sputchar('\n');
				break;
			}
		} else {
			sputchar(*charPtr);
		}
		charPtr++;
	}
	sputcharFlush();

	if (cprintfOSInit) {
		osSemaphoreGive(cprintfMutex);
	}
	// clean up the argument list
	va_end(arguments);
}


/**
 * @brief Sets the CRLF mode for cprintf. One of:
 * CPRINTF_CRLF_LF, CPRINTF_CRLF_CR
 * CPRINTF_CRLF_CRLF, CPRINTF_CRLF_LFCR
 * @returns void
 */
void cprintfCRLFMode(uint8 CRLFModeArg) {
	CRLFMode = CRLFModeArg;
}


/*
 * @brief Initializes cprintf for serial printing
 *
 * @returns void
 */
void cprintfInit(void) {
	cprintfMutex = osSemaphoreCreateMutex();
	cprintfOSInit = TRUE;
}

