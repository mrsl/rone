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
	char pad[20];
} typedef controlMsg;

// Callback variables
SerialCmd scLT;
SerialCmd scST;
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
	for (i = 0; i < 7; i++) {
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
	for (i = 0; i < 7; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
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
}

/**
 * Initialize callbacks
 */
void mrmInitCallbacks() {
	serialCommandAdd(&scLT, "lt", scLTFunc);
	serialCommandAdd(&scST, "st", scSTFunc);
	radioCommandAddCallback(&rcSend, "RC", rcCallback);
}
