/**
 * @file rprintf.h
 *
 * @brief Light-weight formatted printing to a remote print buffer that is routed through a
 * host robot.
 *
 * @since 2001
 * @author James McLurkin
 * @copyright iRobot 2001
 */

#ifndef RPRINTF_H_
#define RPRINTF_H_

/******** Defines ********/

#define RPRINTF_REMOTE					0
#define RPRINTF_HOST					1

#define RPRINTF_MAX_PACKETS				RADIO_COMMS_QUEUE_RECV_SIZE

/******** Functions ********/

/**
 * @brief Initializes rprintf and the remote print buffer.
 *
 * @returns void
 */
void rprintfInit(void);


/**
 *	@brief Prints formatted output to the remote terminal
 *
 *	Processes the input string into an output string rone understands.
 *	If the input string is too large, rprintfOverRunError is set to TRUE.
 *
 *	@returns void
 */
void rprintf(const char* format, ...);

void rprintfFlush();

void rprintfStringOutput(const char *buffer, int length, int robotID);

//TODO: Should this be public?
void rprintfSetHostMode(uint8 val);


/**
 * @brief Returns the current rprintf terminal mode.
 *
 * Returns the current rprintf terminal mode.
 * @returns uint8 or either RPRINTF_HOST or RPRINTF_REMOTE
 */
boolean rprintfIsHost(void);


/**
 * @brief Enables or disables a particular robot for rprintf hosting
 *
 * @argument robotID The ID of the robot
 * @argument enable TRUE/FALSE to enable/disable query of this robot
 */
void rprintfEnableRobot(uint8 robotID, boolean enable);

/**
 * @brief Measure the rptintf Radio usage
 *
 * @returns the total number of bytes received
 */
uint32 rprintfGetTotalBytesReceived(void);



/**
 * @brief set the time that host waits before the next robot to avoid collisions with local robot comms,
 *  it is 25 ms by default
 *
 * @returns void
 */
void rprintfSetSleepTime(uint32 val);





/**
 * @brief Measure the rprintf Radio usage
 *
 * @returns the total number of bytes received
 */
uint32 rprintfGetActiveRobotsNum(void);


/**
 *
 *
 *
 * @brief Measure the rprintf Radio usage
 *
 * prints the list of active robots.  This starts with a leading comma, which is useful for how it's used.
 */
void rprintfPrintActiveRobotList(void);



#endif /* RPRINTF_H_ */
