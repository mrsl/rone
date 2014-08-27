/*
 * @file rprintf.c
 *
 * @brief Light-weight formatted printing to a remote print buffer that is routed through a
 * host robot.
 * @since 2001
 * @author James McLurkin
 * @copyright iRobot 2001
 */

#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "roneos.h"
#include "snprintf.h"

#define RPRINTF_HOST_RESPONSE_TIMEOUT			10
#define RPRINTF_HOST_REQUESTS_MAX				5

#define RPRINTF_REMOTE_ROBOT_STATE_INACTIVE		0
#define RPRINTF_REMOTE_ROBOT_STATE_ACTIVE		1
#define RPRINTF_REMOTE_ROBOT_STATE_DISABLED		2

#define RPRINTF_QUERY_ACTIVE					0
#define RPRINTF_QUERY_INACTIVE					1

// host radio message
#define RPRINTF_MSG_PACKET_REQ_BITS_IDX 		0
#define RPRINTF_MSG_PACKET_DATA_IDX	 			1

// remote radio message
#define RPRINTF_MSG_BUFFER_LENGTH_IDX 			0
#define RPRINTF_MSG_PACKET_IDX 					1
#define RPRINTF_MSG_DATA_IDX					2
#define RPRINTF_MSG_DATA_PAYLOAD_START			3
#define RPRINTF_MSG_DATA_PAYLOAD_LENGTH 		(RADIO_COMMAND_MESSAGE_DATA_LENGTH - RPRINTF_MSG_DATA_PAYLOAD_START)

#define RPRINTF_TEXT_STRING_SIZE		(RPRINTF_MSG_DATA_PAYLOAD_LENGTH * RPRINTF_MAX_PACKETS)

boolean rprintfOSInit = FALSE;
static uint32 rprintfRemoteRequestTime = 0;

static osSemaphoreHandle rprintfWriteMutex;	// Local write buffer mutex
static osSemaphoreHandle rprintfSendMutex;		// Radio send buffer mutex

static char rprintfWriteBuffer[RPRINTF_TEXT_STRING_SIZE];	// Local write buffer
static char rprintfSendBuffer[RPRINTF_TEXT_STRING_SIZE];	// Radio send buffer
static char rprintfRecvBuffer[RPRINTF_TEXT_STRING_SIZE];	// Radio receive buffer
static char rprintfTempBuffer[RPRINTF_TEXT_STRING_SIZE];

static int rprintfWriteBufferLength;	// Length of string in write buffer
static int rprintfSendBufferLength;		// Length of string in send buffer

static uint8 rprintfMode = RPRINTF_REMOTE;
static RadioCmd radioCmdRprintfHostToRemote;
static RadioCmd radioCmdRprintfRemoteToHost;

static uint8 rprintfRemoteRobotState[ROBOT_ID_MAX + 1] = {0};
static uint8 rprintfRemoteRobotMessageID[ROBOT_ID_MAX + 1] = {0};
static uint32 rprintfTotalBytesReceived = 0;
static uint16 rprintfActiveRobotNum = 0;
static uint32 rprintfRoundNum = 0;
static uint32 rprintfHostInterRobotSleepTime = 25;
static int rprintfSendNumTransmitsRemaining; // Number of retransmits for this data
static uint8 rprintfMessageID;

/*
 *	@brief Prints formatted output to the remote terminal
 *
 *	Processes the input string into an output string rone understands.
 *	If the input string is too large, rprintfOverRunError is set to TRUE.
 *
 *	@returns void
 */
void rprintf(const char *format, ...) {
	int32 i, potentialChars;
	char tempBuffer[RPRINTF_TEXT_STRING_SIZE];

	va_list arguments;

	// If not initialized, return out
	if (!rprintfOSInit) {
		return;
	}

	// Safely lock the write buffer
	osSemaphoreTake(rprintfWriteMutex, portMAX_DELAY);

	va_start(arguments, format);

	// Write string with argument formatting into write buffer
	potentialChars = vsnprintf(tempBuffer, RPRINTF_TEXT_STRING_SIZE - 1, format, arguments);

	// Check for overflow
	if (potentialChars >= (RPRINTF_TEXT_STRING_SIZE - 1)) {
	} else {
		for (i = 0; i < potentialChars; i++) {
			if (rprintfWriteBufferLength < RPRINTF_TEXT_STRING_SIZE - 1) {
				rprintfWriteBuffer[rprintfWriteBufferLength++] = tempBuffer[i];
			} else {
				break;
			}
		}
	}
	rprintfWriteBuffer[rprintfWriteBufferLength] = '\0';

	// Clean up arguments.
	va_end(arguments);

	// Unlock the write buffer
	osSemaphoreGive(rprintfWriteMutex);
}

/*
 *	@brief Prints rprintf string to serial
 *
 *	Inserts newlines for multi-lined messages or improperly formatted strings
 *
 *	@returns void
 */
void rprintfStringOutput(const char *buffer, int length, int robotID) {
	cprintf("rtd,%d %s", robotID, buffer);
//	uint8 i;
//	char *bufp;
//	char bufferCopy[length + 1];
//
//	strncpy(bufferCopy, buffer, length + 1);
//
//	bufp = bufferCopy;
//	cprintf("rtd,%d %s", robotID, bufp);
//
//	bufp = bufferCopy;
//	// Iterate through each string line and output with prefix
//	for (i = 0; i <= length; i++) {
//		if (bufferCopy[i] == '\n') {
//			bufferCopy[i] = '\0';
//		}
//		if (bufferCopy[i] == '\0') {
//			if (bufp != &bufferCopy[i]) {
//				// rtd,ID Message...
//				cprintf("rtd,%d %s\n", robotID, bufp);
//				bufp = &bufferCopy[++i];
//			}
//		}
//	}
}

/*
 *	@brief Flushes the rprintf write buffer to be sent out
 *
 *	@returns void
 */
void rprintfFlush() {
	int i;

	// Safely lock the write buffer
	osSemaphoreTake(rprintfWriteMutex, portMAX_DELAY);
	// Safely lock the send buffer
	osSemaphoreTake(rprintfSendMutex, portMAX_DELAY);

	// Print over serial the buffer that will be sent out
	rprintfStringOutput(rprintfWriteBuffer, rprintfWriteBufferLength, roneID);

	// Copy the buffer over
	strncpy(rprintfSendBuffer, rprintfWriteBuffer, RPRINTF_TEXT_STRING_SIZE);

	for (i = 0; i < RPRINTF_TEXT_STRING_SIZE; i++) {
		rprintfWriteBuffer[i] = '\0';
	}

	// Set the new buffer lengths
	rprintfSendBufferLength = rprintfWriteBufferLength;
	rprintfWriteBufferLength = 0;

	// Reset number of retransmits remaining
	rprintfSendNumTransmitsRemaining = RPRINTF_HOST_REQUESTS_MAX;

	// Set new message ID number
	rprintfMessageID = (rprintfMessageID + 1) % 100 + 1;

	// Unlock the send buffer
	osSemaphoreGive(rprintfSendMutex);
	// Unlock the write buffer
	osSemaphoreGive(rprintfWriteMutex);
}

/*
 * Specifies a robot to include in the radio query.
 */
void rprintfEnableRobot(uint8 robotID, boolean enable) {
	uint16 i, startID, endID;

	// Enable all robots
	if (robotID == ROBOT_ID_ALL) {
		startID = ROBOT_ID_MIN;
		endID = ROBOT_ID_MAX;
	// Do nothing
	} else if (robotID == ROBOT_ID_NULL) {
		return;
	// Set range
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
	if ((packetNumMax * RPRINTF_MSG_DATA_PAYLOAD_LENGTH) < length) {
		packetNumMax++;
	}
	return packetNumMax;
}

static void queryRobot(uint8 remoteRobotID, uint8 queryMode) {
	RadioMessage radioMsg;
	uint8 i, length, requestAttempts;
	uint8 msgBitsReceived, msgBitsExpected, msgBitsRequest;
	uint8 messageID, packetNum, packetNumMax, remoteRobotState;
	boolean queryComplete, val;

	// Initialization
	queryComplete = FALSE;
	remoteRobotState = RPRINTF_REMOTE_ROBOT_STATE_INACTIVE;
	requestAttempts = 0;
	msgBitsRequest = 0;
	msgBitsExpected = 0;
	msgBitsReceived = 0;
	packetNumMax = 0;
	messageID = 0;
	radioMsg.command.destinationID = remoteRobotID;

	// Query the selected robot.
	length = 0;
	while ((requestAttempts++ < RPRINTF_HOST_REQUESTS_MAX) && (!queryComplete)) {
		// Send an initial rprintf request to the remote robot.
		radioMsg.command.data[RPRINTF_MSG_PACKET_REQ_BITS_IDX] = msgBitsRequest;
		radioMsg.command.data[RPRINTF_MSG_PACKET_DATA_IDX] = messageID;

		radioCommandXmit(&radioCmdRprintfHostToRemote, remoteRobotID, &radioMsg);

		// Wait for the remote robot to respond.
		val = radioCommandReceive(&radioCmdRprintfRemoteToHost, &radioMsg, RPRINTF_HOST_RESPONSE_TIMEOUT);
		if (val) {
			do {
				// First message received, obtain metadata
				if (remoteRobotState == RPRINTF_REMOTE_ROBOT_STATE_INACTIVE) {
					// If we received a message from an inactive robot, label it active.
					remoteRobotState = RPRINTF_REMOTE_ROBOT_STATE_ACTIVE;
					// Obtain length and message ID
					length = (uint8) radioMsg.command.data[RPRINTF_MSG_BUFFER_LENGTH_IDX];
					messageID = (uint8) radioMsg.command.data[RPRINTF_MSG_DATA_IDX];
					// Message ID is the same as before, this is a retransmission so ignore
					if (messageID == rprintfRemoteRobotMessageID[remoteRobotID]) {
						queryComplete = TRUE;
						length = 0;
						break;
					}
					// Compute number of packets needed to transmit message
					packetNumMax = computeNumPackets(length);
					for (i = 0; i < packetNumMax; ++i) {
						msgBitsExpected |= (1 << i);
					}
				}
				// Part of a message, fill in data
				if (length > 0) {
					// Message ID doesn't match, drop this query
					if (messageID != (uint8) radioMsg.command.data[RPRINTF_MSG_DATA_IDX]) {
						queryComplete = TRUE;
						length = 0;
						break;
					}
					// If we received part of a message, copy it over.
					packetNum = (uint8) radioMsg.command.data[RPRINTF_MSG_PACKET_IDX];

					msgBitsReceived |= (1 << packetNum);
					uint16 packetIdx;
					uint16 msgIdx = packetNum * RPRINTF_MSG_DATA_PAYLOAD_LENGTH;
					for (packetIdx = 0; packetIdx < RPRINTF_MSG_DATA_PAYLOAD_LENGTH; packetIdx++) {
						if (msgIdx < RPRINTF_TEXT_STRING_SIZE) {
							rprintfRecvBuffer[msgIdx++] = radioMsg.command.data[RPRINTF_MSG_DATA_PAYLOAD_START + packetIdx];
						}
					}
				}
				// Either we've gotten all the bits, or length = 0 and there is no data to receive: we're done.
				if (msgBitsExpected == msgBitsReceived) {
					queryComplete = TRUE;
					if(msgBitsExpected>0){
						// do something??
					}
					break;
				}
				// Request any bits that were missed.
				msgBitsRequest = msgBitsExpected & ~msgBitsReceived;
			} while (radioCommandReceive(&radioCmdRprintfRemoteToHost, &radioMsg, RPRINTF_HOST_RESPONSE_TIMEOUT));
		}
	}
	// Set state
	rprintfRemoteRobotState[remoteRobotID] = remoteRobotState;
	rprintfRemoteRobotMessageID[remoteRobotID] = messageID;

	// Print the message.
	if (queryComplete && (length > 0)) {
		rprintfTotalBytesReceived += length;
		rprintfRecvBuffer[length] = '\0';

		rprintfStringOutput(rprintfRecvBuffer, length, remoteRobotID);

		for (i = 0; i < RPRINTF_TEXT_STRING_SIZE; i++) {
			rprintfRecvBuffer[i] = '\0';
		}
	}
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
			rprintfRoundNum += 1;
			// Query all active robots
			for (remoteRobotID = ROBOT_ID_MIN; remoteRobotID <= ROBOT_ID_MAX; remoteRobotID++) {
				status = rprintfRemoteRobotState[remoteRobotID];
				if (status == RPRINTF_REMOTE_ROBOT_STATE_ACTIVE) {
					activeTemp++;
					// Querying active robot
					queryRobot(remoteRobotID, TRUE);
				}
			}
			// Search through robot list for queryable inactive robot
			remoteRobotIDInactive = remoteRobotIDInactiveOld + 1;
			// Stop if we have wrapped around
			while (remoteRobotIDInactive != remoteRobotIDInactiveOld) {
				status = rprintfRemoteRobotState[remoteRobotIDInactive];
				// Wrap around
				if (remoteRobotIDInactive == ROBOT_ID_MAX) {
					remoteRobotIDInactive = ROBOT_ID_MIN;
				}
				// Query if inactive
				if (status == RPRINTF_REMOTE_ROBOT_STATE_INACTIVE) {
					queryRobot(remoteRobotIDInactive, FALSE);
					break;
				}
				remoteRobotIDInactive++;
			}
			remoteRobotIDInactiveOld = remoteRobotIDInactive;
			rprintfActiveRobotNum = activeTemp;
			// Wait before the next robot to avoid collisions
			osTaskDelay(rprintfHostInterRobotSleepTime);

		// Wait before we check again.  Minimize CPU load for remote robots.
		} else {
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
void rprintfRemoteCallback(RadioCmd* radioCmdPtr, RadioMessage* msgPtr) {
	uint8 requestedBits, requestedBitsToXmit;
	uint8 packetNum, packetNumMax;
	uint8 bufferLength;
	uint8 requestedMessageID;
	RadioMessage radioResponseMsg;

	osSemaphoreTake(rprintfSendMutex, portMAX_DELAY);

	cprintf("\ncallback!\n");

	// We are remote. Respond to the host with the contents of the buffer.
	requestedBits = msgPtr->command.data[RPRINTF_MSG_PACKET_REQ_BITS_IDX];
	requestedMessageID = msgPtr->command.data[RPRINTF_MSG_PACKET_DATA_IDX];

	requestedBitsToXmit = requestedBits;
	rprintfRemoteRequestTime = osTaskGetTickCount();
	if (requestedBits == 0x00) {
		// This is a new data request.
		requestedBitsToXmit = 0xFF;
	}

	// If max retransmits reached, get rid of buffer.
	if (rprintfSendNumTransmitsRemaining-- > 0) {
		bufferLength = (uint8) strlen(rprintfSendBuffer);
		strncpy(rprintfTempBuffer, rprintfSendBuffer, RPRINTF_TEXT_STRING_SIZE);
	} else {
		bufferLength = 0;
	}

	if (!(requestedMessageID == 0 || requestedMessageID == rprintfMessageID)) {
		bufferLength = 0;
	}

	// Build the response and request a retransmit if needed.
	packetNumMax = computeNumPackets(bufferLength);
	radioResponseMsg.command.data[RPRINTF_MSG_BUFFER_LENGTH_IDX] = bufferLength;

	// Do we have anything to transmit at all?
	if (packetNumMax == 0) {
		// You have nothing new to transmit. Send an ack back to the host.
		radioCommandXmit(&radioCmdRprintfRemoteToHost, ROBOT_ID_ALL, &radioResponseMsg);
	}

	uint16 packetIdx;
	uint16 msgIdx;
	char c;
	// Transmit the message.
	for (packetNum = 0; packetNum < packetNumMax; packetNum++) {
		if (requestedBitsToXmit & 0x01) {
			radioResponseMsg.command.data[RPRINTF_MSG_PACKET_IDX] = packetNum;
			radioResponseMsg.command.data[RPRINTF_MSG_DATA_IDX] = rprintfMessageID;

			msgIdx = packetNum * RPRINTF_MSG_DATA_PAYLOAD_LENGTH;

			for (packetIdx = 0; packetIdx < RPRINTF_MSG_DATA_PAYLOAD_LENGTH; packetIdx++) {
				if (msgIdx < bufferLength) {
					c = rprintfTempBuffer[msgIdx++];
				} else {
					c = '\0';
				}
				radioResponseMsg.command.data[RPRINTF_MSG_DATA_PAYLOAD_START + packetIdx] = c;
			}
			radioCommandXmit(&radioCmdRprintfRemoteToHost, ROBOT_ID_ALL, &radioResponseMsg);
		}
		requestedBitsToXmit >>= 1;
	}

	osSemaphoreGive(rprintfSendMutex);
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
 */
void rprintfInit(void) {
	// Initialize variables
	rprintfWriteMutex = osSemaphoreCreateMutex();
	rprintfSendMutex = osSemaphoreCreateMutex();
	rprintfOSInit = TRUE;
	rprintfMessageID = 1;
	rprintfWriteBufferLength = 0;
	rprintfSendBufferLength = 0;
	rprintfSendNumTransmitsRemaining = 0;

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

