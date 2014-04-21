/**
 * @file cprintf.h
 *
 * @brief Light-weight formatted printing to a remote print buffer that is routed through the
 * robot network.
 *
 * @since 2001
 * @author James McLurkin
 * @copyright iRobot 2001
 */

#ifndef CPRINTF_H_
#define CPRINTF_H_

/******** Defines ********/

#define CPRINTF_TEXT_STRING_SIZE		(RADIO_COMMAND_MESSAGE_DATA_LENGTH * RPRINTF_MAX_PACKETS)

#define CPRINTF_CRLF_LF					0
#define CPRINTF_CRLF_CR					1
#define CPRINTF_CRLF_CRLF				2
#define CPRINTF_CRLF_LFCR				3


/******** Functions ********/

/**
 * @brief Initializes cprintf for serial printing
 *
 * @returns void
 */
void cprintfInit(void);


/**
 * @brief Sets the CRLF mode for cprintf. One of:
 * CPRINTF_CRLF_LF, CPRINTF_CRLF_CR
 * CPRINTF_CRLF_CRLF, CPRINTF_CRLF_LFCR
 * @returns void
 */
void cprintfCRLFMode(uint8 CRLFMode);


/**
 *	@brief Serves the same purpose as formatted output string for the robot.
 *
 *	Processes the input string into an output string rone understands.
 *	If the input string is too large, cfprintfOverRunError is set to TRUE.
 *
 *	@returns void
 */
void cprintf(char* format, ...);

#endif /* CPRINTF_H_ */
