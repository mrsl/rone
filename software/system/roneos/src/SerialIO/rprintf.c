/*
 * @file rprintf.c
 *
 * @brief Light-weight formatted printing to a remote print buffer that is routed through a
 * host robot.
 * @since 2001
 * @author James McLurkin
 * @copyright iRobot 2001
 */

/******** Include Files ********/
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "roneos.h"
#include "snprintf.h"

#define RPRINTF_HOST_RESPONSE_TIMEOUT			4
#define RPRINTF_HOST_REQUESTS_MAX				5

#define RPRINTF_REMOTE_ROBOT_STATE_INACTIVE		0
#define RPRINTF_REMOTE_ROBOT_STATE_ACTIVE		1
#define RPRINTF_REMOTE_ROBOT_STATE_DISABLED		2

#define RPRINTF_QUERY_ACTIVE					0
#define RPRINTF_QUERY_INACTIVE					1

// host radio message
#define RPRINTF_MSG_PACKET_REQ_BITS_IDX 		0

// remote radio message
#define RPRINTF_MSG_BUFFER_LENGTH_IDX 			0
#define RPRINTF_MSG_PACKET_IDX 					1
#define RPRINTF_MSG_DATA_PAYLOAD_START			2
#define RPRINTF_MSG_DATA_PAYLOAD_LENGTH 		(RADIO_COMMAND_MESSAGE_DATA_LENGTH - RPRINTF_MSG_DATA_PAYLOAD_START)

/******** Variables ********/

static osSemaphoreHandle rprintfMutex;
boolean rprintfOSInit = FALSE;
static char rprintfBuffer[RPRINTF_TEXT_STRING_SIZE];
static char* rprintfBufferPtr = rprintfBuffer;
static uint8 rprintfBufferLength = 0;

static char rprintfRadioBuffer[RPRINTF_TEXT_STRING_SIZE];
static char rprintfRadioBufferPrev[RPRINTF_TEXT_STRING_SIZE];
static char* rprintfRadioBufferPrevPtr = rprintfRadioBufferPrev;
static char* rprintfRadioBufferPtr = rprintfRadioBuffer;
static boolean rprintfRadioBufferDataReady = FALSE;
static boolean rprintfBufferCallbackLock = FALSE;
static uint32 rprintfRemoteRequestTime = 0;

static uint8 rprintfMode = RPRINTF_REMOTE;
static uint8 currentRobotIndex = ROBOT_ID_MIN;
static boolean radioCmdQueryAll = FALSE;
static RadioCmd radioCmdRprintfHostToRemote;
static RadioCmd radioCmdRprintfRemoteToHost;

static uint8 rprintfRemoteRobotState[ROBOT_ID_MAX + 1] = {0};
static uint32 rprintfTotalBytesReceived = 0;
static uint16 rprintfActiveRobotNum = 0;
static uint32 roundNum = 0;
static uint32 rprintfHostInterRobotSleepTime = 25;


//128 bytes of text:
//00000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000111111111122222222

/******** Functions ********/

/*
 *	@brief Prints formatted output to the remote terminal
 *
 *	Processes the input string into an output string rone understands.
 *	If the input string is too large, rprintfOverRunError is set to TRUE.
 *
 *	@returns void
 */
void rprintf(char *format, ...) {
	va_list arguments;

	va_start(arguments, format);

	int32 potentialChars;
	char* charPtr;
	if (rprintfOSInit) {
		// wait until the callback function clears the flag, or the buffer times out
		if (rprintfBufferCallbackLock) {
			if (osTaskGetTickCount() > (rprintfRemoteRequestTime + RPRINTF_HOST_RESPONSE_TIMEOUT)) {
				// the previous radio request has timed out.  release the buffer lock
				rprintfBufferCallbackLock = FALSE;
			} else {
				// wait a bit for the callback function to release the lock
				osTaskDelay(1);
			}
		}

		// get the mutex to write to the rptintf flags and buffer
		osSemaphoreTake(rprintfMutex, portMAX_DELAY);

		// since we are writing new data, the buffer is not ready to be accessed by the rprintf thread
		rprintfRadioBufferDataReady = FALSE;

		/* Process the input string and store the output in the 'outStringFormatted' */
		/* note that vsnprintf returns the number of characters that would have been
		 * written to the buffer if it were large enough. */
		potentialChars = vsnprintf(rprintfBufferPtr, RPRINTF_TEXT_STRING_SIZE - 1, format, arguments);
		if (potentialChars >= (RPRINTF_TEXT_STRING_SIZE - 1)) {
			error("rprintf buffer overflow");
		}

		charPtr = rprintfBufferPtr;
		while (*charPtr != '\0') {
			// buffer the char on the radio buffer
			//if((!rprintfBufferCallbackLock) && (rprintfRadioBufferPtr < (rprintfRadioBuffer + RPRINTF_TEXT_STRING_SIZE - 1))) {
			if(rprintfRadioBufferPtr < (rprintfRadioBuffer + RPRINTF_TEXT_STRING_SIZE - 1)) {
				*rprintfRadioBufferPtr++ = *charPtr;
			}
			if (*charPtr == '\n') {
				//if(!rprintfBufferCallbackLock) {
				// the radio buffer is ready for xmit
				*rprintfRadioBufferPtr = '\0';
				rprintfRadioBufferDataReady = TRUE;
				rprintfRadioBufferPtr = rprintfRadioBuffer;
				//TODO replace these fake battery values and radio signal quality with real numbers
				cprintf("rtd, %d, ", roneID);
				cprintf(rprintfRadioBuffer);
				//}
			}
			charPtr++;
		}
		osSemaphoreGive(rprintfMutex);
	}
	// clean up the argument list
	va_end(arguments);
}

/*
 * Specifies a robot to include in the radio query.
 */
void rprintfEnableRobot(uint8 robotID, boolean enable) {
	uint16 i, startID, endID;

	if (robotID == ROBOT_ID_ALL) {
		startID = ROBOT_ID_MIN;
		endID = ROBOT_ID_MAX;
	} else if (robotID == ROBOT_ID_NULL) {
		// do nothing
		return;
	} else {
		startID = robotID;
		endID = robotID;
	}

	for (i = startID; i <= endID; ++i) {
		if (enable) {
			// leave active robots active.  Set disabled robots to inactive.
			if (rprintfRemoteRobotState[i] == RPRINTF_REMOTE_ROBOT_STATE_DISABLED) {
				rprintfRemoteRobotState[i] = RPRINTF_REMOTE_ROBOT_STATE_INACTIVE;
			}
		} else {
			rprintfRemoteRobotState[i] = RPRINTF_REMOTE_ROBOT_STATE_DISABLED;
		}
	}
}

/*
 * Returns the number of packets needed to transmit length
 * amount of data.
 */
static uint8 computeNumPackets(uint8 length) {
	uint8 packetNumMax = length / RPRINTF_MSG_DATA_PAYLOAD_LENGTH;
	// add one to deal with the loss of precision of integer division.
	// packetNumMax = ceiling(length / PAYLOAD_LENGTH)
	if ((packetNumMax * RPRINTF_MSG_DATA_PAYLOAD_LENGTH) < length) {
		packetNumMax++;
	}
	return packetNumMax;
}

static void queryRobot(uint8 remoteRobotID, uint8 queryMode) {
	RadioMessage radioMsg;
	uint8 i, length, requestAttempts;
	uint8 msgBitsReceived, msgBitsExpected, msgBitsRequest;
	uint8 packetNum, packetNumMax, remoteRobotState;
	boolean queryComplete, val;
	uint8 temp;
	uint8 cfprintfRadioActiveRobotCountTemp;

	// Query the selected robot.
	queryComplete = FALSE;
	remoteRobotState = RPRINTF_REMOTE_ROBOT_STATE_INACTIVE;
	requestAttempts = 0;
	msgBitsRequest = 0;
	msgBitsExpected = 0;
	msgBitsReceived = 0;
	packetNumMax = 0;

	length = 0;
	while ((requestAttempts++ < RPRINTF_HOST_REQUESTS_MAX) && (!queryComplete)) {
		// Send an initial rprintf request to the remote robot.
		radioMsg.command.data[RPRINTF_MSG_PACKET_REQ_BITS_IDX] = msgBitsRequest;
		//print . if robot is being queried but no message yet
		//if (queryMode) cprintf(".");
		//ledSetGroup(2, 120);
		radioCommandXmit(&radioCmdRprintfHostToRemote, remoteRobotID, &radioMsg);
		//ledSetGroup(2, 0);

		// Wait for the remote robot to respond.
		//ledSetGroup(1, 120);
		val = radioCommandReceive(&radioCmdRprintfRemoteToHost, &radioMsg, RPRINTF_HOST_RESPONSE_TIMEOUT);
		//ledSetGroup(1, 0);
		if (val) {
			//print ! if host receives a message from the queried robot
			//if (queryMode) cprintf("!");
			// Assemble the received message.
			do {
				if (remoteRobotState == RPRINTF_REMOTE_ROBOT_STATE_INACTIVE) {
					// If we received a message from an inactive robot, label it active.
					remoteRobotState = RPRINTF_REMOTE_ROBOT_STATE_ACTIVE;
					length = radioMsg.command.data[RPRINTF_MSG_BUFFER_LENGTH_IDX];
					packetNumMax = computeNumPackets(length);
					for (i = 0; i < packetNumMax; ++i) {
						msgBitsExpected |= (1 << i);
					}
				}
				if (length > 0) {
					// If we received part of a message, copy it over.
					packetNum = radioMsg.command.data[RPRINTF_MSG_PACKET_IDX];
					msgBitsReceived |= (1 << packetNum);
					uint16 packetIdx;
					uint16 msgIdx = packetNum * RPRINTF_MSG_DATA_PAYLOAD_LENGTH;
					for (packetIdx = 0; packetIdx < RPRINTF_MSG_DATA_PAYLOAD_LENGTH; packetIdx++) {
						if (msgIdx < RPRINTF_TEXT_STRING_SIZE) {
							rprintfRadioBuffer[msgIdx++] = radioMsg.command.data[RPRINTF_MSG_DATA_PAYLOAD_START + packetIdx];
						}
					}
				}
				if (msgBitsExpected == msgBitsReceived) {
					// Either we've gotten all the bits, or length = 0 and there is no data to receive: we're done.
					queryComplete = TRUE;
					if(msgBitsExpected>0){
					//cprintf("$");
					}
					break;
				}
				// Request any bits that were missed.
				msgBitsRequest = msgBitsExpected & ~msgBitsReceived;
			} while (radioCommandReceive(&radioCmdRprintfRemoteToHost, &radioMsg, RPRINTF_HOST_RESPONSE_TIMEOUT));
		}
	}
	rprintfRemoteRobotState[remoteRobotID] = remoteRobotState;

	float batteryLevel = 3.7;
	uint8 signalQuality = 80;

	// Print the message.
	if (queryComplete && (length > 0)) {
		rprintfTotalBytesReceived += length;
		//rprintfRadioBufferPrevPtr = rprintfRadioBufferPtr;
		//cprintf("rtd,%d,3.7,80 %s", remoteRobotID, batteryLevel, signalQuality, cfprintfRadioBuffer);
		//cprintf("red,%d,3.7,80 %s,rprintfRadioBuffer);
		cprintf("rtd,%d %s", remoteRobotID, rprintfRadioBuffer);
		//cprintf(";%d%s,%d;", remoteRobotID, rprintfRadioBuffer, remoteRobotID);	//###
	}
	//else if (remoteRobotState){
		//if we didnt get a new message, print the previous message - used for nav demo
	//	cprintf("rtd,%d,time%d %s", remoteRobotID, roundNum, rprintfRadioBufferPrev);
	//}
}

/*
 * If the robot is in host mode, this will query other robots remotely
 * for the contents of their rprintf buffer. If the robot is in
 * query-all mode, it will query all robots it can find; if not, it will
 * only include robots specified by the remote-include command.
 */
static void rprintfHostTask(void* parameters) {
	uint8 remoteRobotIDInactiveOld = ROBOT_ID_MIN;
	uint8 remoteRobotIDInactive;
	uint8 remoteRobotID;
	uint8 status;
	uint8 activeTemp;

	while (TRUE) {
		// This task only runs if we are the host.
		if (rprintfMode == RPRINTF_HOST) {
			activeTemp = 0;
			roundNum+=1;
			for (remoteRobotID = ROBOT_ID_MIN; remoteRobotID <= ROBOT_ID_MAX; remoteRobotID++) {
				status = rprintfRemoteRobotState[remoteRobotID];
				if (status == RPRINTF_REMOTE_ROBOT_STATE_ACTIVE) {
					activeTemp++;
					//querying active robot
					//cprintf("qa %d\n", remoteRobotID);
					queryRobot(remoteRobotID, TRUE);
				}
			}
			// query inactive robots
			remoteRobotIDInactive = remoteRobotIDInactiveOld + 1;
			while (remoteRobotIDInactive != remoteRobotIDInactiveOld) {
				status = rprintfRemoteRobotState[remoteRobotIDInactive];
				if (remoteRobotIDInactive == ROBOT_ID_MAX) {
					remoteRobotIDInactive = ROBOT_ID_MIN;
				}
				if (status == RPRINTF_REMOTE_ROBOT_STATE_INACTIVE) {
					//cprintf("qi %d\n",remoteRobotIDInactive);
					queryRobot(remoteRobotIDInactive, FALSE);
					break;
				}
				remoteRobotIDInactive++;
			}
			remoteRobotIDInactiveOld = remoteRobotIDInactive;
			rprintfActiveRobotNum = activeTemp;
			// Wait before the next robot to avoid collisions with local robot comms
			// this will make it take a long time to query a large number of robots.  We need a better plan...
			osTaskDelay(rprintfHostInterRobotSleepTime);
		} else {
			// Wait before we check again.  Minimize CPU load for remote robots.
			osTaskDelay(200);
		}
	}
}

/*
 * When a robot receives a radio query, it transmits
 * the contents of its rprintf buffer back.
 *
 * msgPtr is the pointer to the radio message containing
 * the callback request.
 */
static void rprintfRemoteCallback(RadioCmd* radioCmdPtr, RadioMessage* msgPtr) {
	uint8 destinationRobotID, length, requestedBits, requestedBitsToXmit;
	uint8 packetNum, packetNumMax;
	RadioMessage radioResponseMsg;

	// We are remote. Respond to the host with the contents of the rprintf buffer.
	// Parse the request.
	requestedBits = msgPtr->command.data[RPRINTF_MSG_PACKET_REQ_BITS_IDX];

	requestedBitsToXmit = requestedBits;
	rprintfRemoteRequestTime = osTaskGetTickCount();
	if (requestedBits == 0x00) {
		// This is a new data request.
		requestedBitsToXmit = 0xFF;
	}

	// Lock the radio buffer to check if there is something to transmit.
	osSemaphoreTake(rprintfMutex, portMAX_DELAY);

	// If the buffer is ready, determine the length of the message.
	if (rprintfRadioBufferDataReady) {
		length = strlen(rprintfRadioBuffer);
		rprintfBufferCallbackLock = TRUE;
		rprintfRadioBufferDataReady = FALSE;
		//Robot is being asked if it has a message
		//cprintf("*");
	} else {
		length = 0;
	}

	osSemaphoreGive(rprintfMutex);

	// Build the response and request a retransmit if needed.
	packetNumMax = computeNumPackets(length);
	radioResponseMsg.command.data[RPRINTF_MSG_BUFFER_LENGTH_IDX] = length;

	// do we have anything to transmit at all?
	//TODO data race!  split the writer buffer and the sender buffer
	if (packetNumMax == 0) {
		// You have nothing new to transmit. Send an ack back to the host.
		radioCommandXmit(&radioCmdRprintfRemoteToHost, ROBOT_ID_ALL, &radioResponseMsg);
		rprintfBufferCallbackLock = FALSE;
	}

	// Transmit the message.
	for (packetNum = 0; packetNum < packetNumMax; packetNum++) {
		if (requestedBitsToXmit & 0x01) {
			radioResponseMsg.command.data[RPRINTF_MSG_PACKET_IDX] = packetNum;

			uint16 packetIdx;
			uint16 msgIdx = packetNum * RPRINTF_MSG_DATA_PAYLOAD_LENGTH;
			char c;
			for (packetIdx = 0; packetIdx < RPRINTF_MSG_DATA_PAYLOAD_LENGTH;
					packetIdx++) {
				if (msgIdx < RPRINTF_TEXT_STRING_SIZE) {
					c = rprintfRadioBuffer[msgIdx++];
				} else {
					c = 0;
				}
				radioResponseMsg.command.data[RPRINTF_MSG_DATA_PAYLOAD_START
						+ packetIdx] = c;
			}
			radioCommandXmit(&radioCmdRprintfRemoteToHost, ROBOT_ID_ALL, &radioResponseMsg);
			//TODO this delay is here because there is a bug in the radio drivers
			//systemDelay(5000);
			//osTaskDelay(1);
		}
		requestedBitsToXmit >>= 1;
	}

}


/*
 * If the robot is set to host mode, it will
 * run the host task below, starting remote callback
 * among the robots. Host mode is set when the rt command
 * is used.
 */
void rprintfSetHostMode(uint8 val) {
	if (val) {
		rprintfMode = RPRINTF_HOST;
	} else {
		rprintfMode = RPRINTF_REMOTE;
	}
}


/*
 * @brief Returns the current rprintf terminal mode.
 *
 * Returns the current rprintf terminal mode.
 * @returns uint8 or either RPRINTF_HOST or RPRINTF_REMOTE
 */
boolean rprintfIsHost(void) {
	return (rprintfMode == RPRINTF_HOST ? TRUE : FALSE);
}


/*
 * @brief Measure the rptintf Radio usage
 *
 * @returns the total number of bytes received
 */
uint32 rprintfGetTotalBytesReceived(void) {
	return rprintfTotalBytesReceived;
}


/*
 * @brief Measure the rptintf Radio usage
 *
 * @returns the total number of bytes received
 */
uint32 rprintfGetActiveRobotsNum(void) {
	return rprintfActiveRobotNum;
}


/*
 * @brief Measure the rprintf Radio usage
 *
 * prints the list of active robots.  This starts with a leading comma, which is useful for how it's used.
 */
void rprintfPrintActiveRobotList(void) {
	uint16 i;
	for (i = ROBOT_ID_MIN; i <= ROBOT_ID_MAX; ++i) {
		if (rprintfRemoteRobotState[i] == RPRINTF_REMOTE_ROBOT_STATE_ACTIVE) {
			cprintf(",%d", i);
		}
	}
}



/**
 * @brief set the time that host waits before the next robot to avoid collisions with local robot comms,
 *  it is 25 ms by default
 *
 * @returns void
 */
void rprintfSetSleepTime(uint32 val) {
	rprintfHostInterRobotSleepTime = val;
}


/*
 * @brief Initializes rprintf and the remote print buffer.
 *
 * @returns void
 *
 *
 *
 *
 */

void rprintfInit(void) {
	uint32 val;
	uint16 i;

	rprintfMutex = osSemaphoreCreateMutex();
	rprintfOSInit = TRUE;

	// init robot state to all be inactive (which is enabled)
	for (i = ROBOT_ID_MIN; i <= ROBOT_ID_MAX; ++i) {
		rprintfRemoteRobotState[i] = RPRINTF_REMOTE_ROBOT_STATE_INACTIVE;
	}

	// Initialize the radio command queue and callback behavior.
	radioCommandAddCallback(&radioCmdRprintfHostToRemote, "rprintf-func", rprintfRemoteCallback);
	radioCommandAddQueue(&radioCmdRprintfRemoteToHost, "rprintf-q", RPRINTF_MAX_PACKETS);

	// enable all the robots for remote query
	rprintfEnableRobot(ROBOT_ID_ALL, TRUE);

	// Put the robot in remote task by default.
	rprintfSetHostMode(RPRINTF_REMOTE);

	// Create the host task, which will run if the robot is in host mode.
	osTaskCreate(rprintfHostTask, "rprintfRemote", 2048, NULL, RPRINTFTERMINAL_TASK_PRIORITY);
}

