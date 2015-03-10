
#include "roneos.h"

#define CHECKVAL 0xDADADADA

#define MSG_TYPE_OR 0
#define MSG_TYPE_OT 1
#define MSG_TYPE_RR 2
#define MSG_TYPE_RT 3
#define MSG_TYPE_AD 4
#define MSG_TYPE_AA 5

int32 objectRV = 40;
int32 objectTV = 10;
int32 robotTVGain = 100;
int32 robotRVGain = 50;
int32 avoidDist = 300;
int32 avoidAngle = MILLIRAD_HALF_PI;


struct __attribute__((__packed__)) {
	uint32 check;
	uint8 messageType;
	int32 value;
	char pad[21];
} typedef controlMsg;

// Callback variables
SerialCmd scOR;
SerialCmd scOT;
SerialCmd scRR;
SerialCmd scRT;
SerialCmd scAD;
SerialCmd scAA;

RadioMessage rmSend;
RadioCmd rcSend;

void scORFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (unsigned int *)&newMessage->value) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_OR;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scOTFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (unsigned int *)&newMessage->value) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_OT;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scRRFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (unsigned int *)&newMessage->value) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_RR;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scRTFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (unsigned int *)&newMessage->value) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_RT;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scADFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (unsigned int *)&newMessage->value) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_AD;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scAAFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%d", (unsigned int *)&newMessage->value) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_TYPE_AA;

	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	osTaskDelay(50);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void testingCallback(RadioCmd* radioCmdPtr, RadioMessage* msgPtr) {
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(msgPtr);

	// If not a valid message, throw it out
	if (newMessage->check != CHECKVAL) {
		return;
	}

	// Set the lookup table
	if (newMessage->messageType == MSG_TYPE_OR) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		objectRV = newMessage->value;
	}

	if (newMessage->messageType == MSG_TYPE_OT) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		objectTV = newMessage->value;
	}

	if (newMessage->messageType == MSG_TYPE_RR) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		robotRVGain = newMessage->value;
	}

	if (newMessage->messageType == MSG_TYPE_RT) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		robotTVGain = newMessage->value;
	}

	if (newMessage->messageType == MSG_TYPE_AD) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		avoidDist = newMessage->value;
	}

	if (newMessage->messageType == MSG_TYPE_AA) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(50);
		avoidAngle = newMessage->value;
	}
}

/**
 * Initialize callbacks
 */
void initCallbacks() {
	serialCommandAdd(&scOR, "or", scORFunc);
	serialCommandAdd(&scOT, "ot", scOTFunc);
	serialCommandAdd(&scRR, "rr", scRRFunc);
	serialCommandAdd(&scRT, "rt", scRTFunc);
	serialCommandAdd(&scAD, "ad", scADFunc);
	serialCommandAdd(&scAA, "aa", scAAFunc);
	radioCommandAddCallback(&rcSend, "RC", testingCallback);
}
