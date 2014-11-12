/*
 * consensusCallbacks.c
 *
 *  Created on: Nov 5, 2014
 *      Author: Zak
 */

#include "consensus.h"
#include "consensusPipeline.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECKVAL	0xBADA
#define MSG_CDS		0
#define MSG_CEN		1
#define MSG_PIP		2
#define MSG_PRO		3

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 messageType;
	uint16 val;
	char pad[23];
} typedef controlMsg;

SerialCmd scCS;	// Consensus
SerialCmd scPS;	// Pipeline size
SerialCmd scPR;	// Request probability
RadioMessage rmSend;
RadioCmd rcSend;

void scCSFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->messageType) != 1) {
		cprintf("Invalid command.");
		return;
	}

	/* Spam out to be heard */
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scPSFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->val) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_PIP;

	/* Spam out to be heard */
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void scPRFunc(char* command) {
	int i;
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(&rmSend);

	command += 2;
	newMessage->check = CHECKVAL;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->val) != 1) {
		cprintf("Invalid command.");
		return;
	}

	newMessage->messageType = MSG_PRO;

	/* Spam out to be heard */
	for (i = 0; i < 4; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
		osTaskDelay(25);
	}
}

void rcCallback(RadioCmd* radioCmdPtr, RadioMessage* msgPtr) {
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(msgPtr);

	/* If not a valid message, throw it out */
	if (newMessage->check != CHECKVAL) {
		return;
	}

	/* Blink some lights */
	uint8 i;
	for (i = 0; i < 4; i++) {
		ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(10);
		ledsClear(LED_ALL);
		osTaskDelay(10);
	}

	switch (newMessage->messageType) {
	/* Disable consensus */
	case (MSG_CDS): {
		consensusDisable();
		break;
	}
	/* Enable consensus */
	case (MSG_CEN): {
		consensusEnable();
		break;
	}
	/* Set the pipeline size */
	case (MSG_PIP): {
		consensusPipelineSetSize((uint8) newMessage->val);
		break;
	}
	/* Set the probability of request */
	case (MSG_PRO): {
		consensusSetReqProbability(newMessage->val);
		break;
	}
	}
}

void consensusCallbackInit(void) {
	serialCommandAdd(&scCS, "cs", scCSFunc);
	serialCommandAdd(&scPS, "ps", scPSFunc);
	serialCommandAdd(&scPR, "pr", scPRFunc);
	radioCommandAddCallback(&rcSend, "RC", rcCallback);
}
