/*
 * mrmInputCallbacks.c
 * Radio message and serial message callbacks for functions in the MRM
 *
 *  Created on: Sep 16, 2014
 *      Author: Zak
 */

#include "globalTreeCOM.h"

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 messageType;
	uint8 myId;
	uint8 theirId;
	int16 distance;
	uint8 state;
	int32 rv;
	int32 alpha;
	uint32 nbtime;
	char pad[8];
} typedef controlMsg;

// Callback variables
SerialCmd scLT;
SerialCmd scPT;
SerialCmd scGD;
SerialCmd scRV;
SerialCmd scST;
SerialCmd scNB;
SerialCmd scAL;
SerialCmd scTV;
RadioMessage rmSend;
RadioCmd rcSend;

/**
 * Serial input function, changes values of a lookup table and then broadcasts
 * out to other robots.
 *
 * Format is:
 *     lt ID1 ID2 DIS
 */
void scLTFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u %u %d", (unsigned int *)&newMessage->myId,
		(unsigned int *)&newMessage->theirId,
		(int *)&newMessage->distance) != 3) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_LT;

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

/**
 * Format is:
 *     pt ID
 */
void scPTFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->myId) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_PT;

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);
	setGRLpivot(newMessage->myId);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scGDFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->myId) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_GD;

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);
	setGRLguide(newMessage->myId);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scALFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (int *)&newMessage->alpha) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_AL;

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	setBehFilter(newMessage->alpha);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scNBFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->nbtime) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_NB;

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);
	neighborsSetPeriod(newMessage->nbtime);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

/**
 * Serial input function, changes values of a lookup table and then broadcasts
 * out to other robots.
 *
 * Format is:
 *     rv rv
 */
void scRVFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (int *)&newMessage->rv) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_RV;
	setRVGain(newMessage->rv);

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}
void scTVFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (int *)&newMessage->rv) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_TV;
	setTVGain(newMessage->rv);

	ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

/**
 * Serial input function, changes state of robot
 *
 * Format is:
 *     st STATEID
 */
void scSTFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->state) != 1) {
		cprintf("Invalid Command.");
		return;
	}

	if (newMessage->state > STATE_MAX) {
		cprintf("Invalid State.");
		return;
	}

	newMessage->messageType = MSG_TYPE_ST;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	setState(newMessage->state);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}

//	uint8 nextState = 0;
//
//	if (newMessage->state == STATE_ROTATE) {
//		newMessage->state = STATE_RALIGN;
//		nextState = STATE_ROTATE;
//	}
//	if (newMessage->state == STATE_PIVOT) {
//		newMessage->state = STATE_PALIGN;
//		nextState = STATE_PIVOT;
//	}
//
//
//	setState(newMessage->state);
//
//	// Spam out to be heard
//	for (i = 0; i < 4; i++) {
//		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
//		osTaskDelay(25);
//	}
//
//	if (nextState) {
//		osTaskDelay(MRM_ALIGNMENT_TIME);
//		setState((newMessage->state = nextState));
//
//		// Spam out to be heard
//		for (i = 0; i < 4; i++) {
//			radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
//			osTaskDelay(25);
//		}
//	}
}

/**
 * Callback for lookup table adjustment, triggers when a new message is received
 */
void rcCallback(RadioCmd* radioCmdPtr, RadioMessage* msgPtr) {
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(msgPtr);

	// If not a valid message, throw it out
	if (newMessage->check != CHECKVAL) {
		return;
	}

	// Set the lookup table
	if (newMessage->messageType == MSG_TYPE_LT) {
		ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);
	}

	if (newMessage->messageType == MSG_TYPE_ST) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		setState(newMessage->state);
	}

	if (newMessage->messageType == MSG_TYPE_PT) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		setGRLpivot(newMessage->myId);
	}

	if (newMessage->messageType == MSG_TYPE_GD) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		setGRLguide(newMessage->myId);
	}

	if (newMessage->messageType == MSG_TYPE_RV) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		setRVGain(newMessage->rv);
	}

	if (newMessage->messageType == MSG_TYPE_TV) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		setTVGain(newMessage->rv);
	}

	if (newMessage->messageType == MSG_TYPE_NB) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		neighborsSetPeriod(newMessage->nbtime);
	}

	if (newMessage->messageType == MSG_TYPE_AL) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		setBehFilter(newMessage->alpha);
	}
}

/**
 * Initialize callbacks
 */
void mrmInitCallbacks() {
	serialCommandAdd(&scLT, "lt", scLTFunc);
	serialCommandAdd(&scST, "st", scSTFunc);
	serialCommandAdd(&scPT, "pt", scPTFunc);
	serialCommandAdd(&scGD, "gd", scGDFunc);
	serialCommandAdd(&scRV, "rv", scRVFunc);
	serialCommandAdd(&scTV, "tv", scTVFunc);
	serialCommandAdd(&scAL, "al", scALFunc);
	serialCommandAdd(&scNB, "nb", scNBFunc);
	radioCommandAddCallback(&rcSend, "RC", rcCallback);
}
