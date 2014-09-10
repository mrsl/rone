/*
 * COMandTreeTesting.c
 *
 *  Created on: Aug 27, 2014
 *      Author: Zak, Golnaz
 */

#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"
#include "globalTreeCOM.h"

#define BEHAVIOR_TASK_PERIOD			50
#define NEIGHBOR_ROUND_PERIOD			1000

#define CHECKVAL		0xDADA

#define MSG_TYPE_LT		0
#define MSG_TYPE_ST		1

#define STATE_IDLE		0
#define STATE_CGUESS	1
#define STATE_ROTATE	2

#define STATE_MAX		2

#define CENTROID_ALPHA	60

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 messageType;
	uint8 myId;
	uint8 theirId;
	int16 distance;
	uint8 state;
	char pad[20];
} typedef controlMsg;

SerialCmd scLT;
SerialCmd scST;
RadioMessage rmSend;
RadioCmd rcSend;

uint32 neighborRound;
uint32 startNbrRound = 0;

uint8 state = STATE_IDLE;


// Set up centroid and GRL
scaleCoordinate GRLcentroidCooridates[GLOBAL_ROBOTLIST_MAX_SIZE + 2];
GlobalRobotList globalRobotList;
boolean GRLinit = FALSE;

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
	osTaskDelay(100);

	//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);

	// Spam out to be heard
	for (i = 0; i < 5; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
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
	osTaskDelay(100);

	setState(newMessage->state);

	// Spam out to be heard
	for (i = 0; i < 5; i++) {
		radioCommandXmit(&rcSend, ROBOT_ID_ALL, &rmSend);
	}
}

/**
 * Callback for lookup table adjustment, triggers when a new message is received
 */
void rcCallback(RadioCmd* radioCmdPtr, RadioMessage* msgPtr) {
	controlMsg *newMessage = (controlMsg *) radioCommandGetDataPtr(msgPtr);

	if (newMessage->check != CHECKVAL) {
		return;
	}

	// Set the lookup table
	if (newMessage->messageType == MSG_TYPE_LT) {
		ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(100);
		//setLookup(newMessage->myId, newMessage->theirId, newMessage->distance);
	}

	if (newMessage->messageType == MSG_TYPE_ST) {
		ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		osTaskDelay(100);
		setState(newMessage->state);
	}
}

void setState(uint8 newState) {
	state = newState;

	if (newState == STATE_IDLE) {
		startNbrRound = 0;
	}
}

void filterIIRNavData(navigationData *old, navigationData *new) {
	new->centroidX = (int16) filterIIR(old->centroidX, new->centroidX, CENTROID_ALPHA);
	new->centroidY = (int16) filterIIR(old->centroidY, new->centroidY, CENTROID_ALPHA);
	new->guideX = (int16) filterIIR(old->guideX, new->guideX, CENTROID_ALPHA);
	new->guideY = (int16) filterIIR(old->guideY, new->guideY, CENTROID_ALPHA);
	new->pivotX = (int16) filterIIR(old->pivotX, new->pivotX, CENTROID_ALPHA);
	new->pivotY = (int16) filterIIR(old->pivotY, new->pivotY, CENTROID_ALPHA);
}

void navDataInit(navigationData *navData) {
	navData->centroidX = 0;
	navData->centroidY = 0;
	navData->guideX = 0;
	navData->guideY = 0;
	navData->pivotX = 0;
	navData->pivotY = 0;
}

void behaviorTask(void* parameters) {
	uint32 lastWakeTime;

	Beh behOutput;
	boolean printNow;
	NbrList nbrList;

	navigationData navData;
	navigationData navDataOld;

	navDataInit(&navData);
	navDataInit(&navDataOld);

	// Initialization steps
	systemPrintStartup();
	systemPrintMemUsage();

	radioCommandSetSubnet(1);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	serialCommandAdd(&scLT, "lt", scLTFunc);
	serialCommandAdd(&scST, "st", scSTFunc);
	radioCommandAddCallback(&rcSend, "RC", rcCallback);

	createGRLscaleCoordinates(GRLcentroidCooridates);
	globalRobotListCreate(&globalRobotList);

<<<<<<< HEAD
	setLookup(102, 98, 2300);
	setLookup(102, 128, 4600);
	setLookup(102, 106, 6440);
	setLookup(102, 110, 8800);
	setLookup(102, 118, 8100);
	setLookup(102, 112, 6600);
	setLookup(102, 97, 4700);
	setLookup(102, 121, 5200);
	setLookup(98, 128, 2550);
	setLookup(98, 106, 4200);
	setLookup(98, 110, 6450);
	setLookup(98, 118, 5910);
	setLookup(98, 112, 4650);
	setLookup(98, 97, 4180);
	setLookup(98, 121, 5700);
	setLookup(128, 106, 2550);
	setLookup(128, 110, 4420);
	setLookup(128, 118, 3600);
	setLookup(128, 112, 2800);
	setLookup(128, 97, 3600);
	setLookup(128, 121, 4780);
	setLookup(106, 110, 2750);
	setLookup(106, 118, 3920);
	setLookup(106, 112, 4500);
	setLookup(106, 97, 5700);
	setLookup(106, 121, 7330);
	setLookup(110, 118, 2900);
	setLookup(110, 112, 4700);
	setLookup(110, 97, 6500);
	setLookup(110, 121, 8450);
	setLookup(118, 112, 2150);
	setLookup(118, 97, 4040);
	setLookup(118, 121, 6050);
	setLookup(112, 97, 1900);
	setLookup(112, 121, 3930);
	setLookup(97, 121, 2000);


=======
>>>>>>> bae412c54070db9489e6dcee177143ce26bcdea2
	gripperBoardInit();

	externalPoseInit();

	for (;;) {
		// Default behavior is inactive
		behOutput = behInactive;
		lastWakeTime = osTaskGetTickCount();

		gripperBoardSetServo(90);

		// If host, do not do anything
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		} else if (state == STATE_IDLE) {
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
		} else {

			neighborsGetMutex();

			printNow = neighborsNewRoundCheck(&neighborRound);

			// If neighbor data has updated, print out new centroid estimate
			if (printNow) {
				nbrListCreate(&nbrList);

				//navDataOld = navData;
				globalRobotListUpdate(&globalRobotList, &nbrList);
				centroidGRLUpdate(&navData, globalRobotList, &nbrList, GRLcentroidCooridates);

				if (startNbrRound == 0) {
					startNbrRound = neighborRound;
				}

				rprintf("%d, %d, %d, %u\n", navData.centroidX, navData.centroidY, navData.childCountSum, neighborRound - startNbrRound);
				rprintfFlush();
			}

			neighborsPutMutex();

//			filterIIRNavData(&navDataOld, &navData);
//			rprintf("    %d, %d\n", navData.centroidX, navData.centroidY);
//			rprintfFlush();

			// Set LEDs based on state
			if (isPivot) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else if (roneID == GUIDE_ROBOT_ID) {
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else {
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			}

			// Set to pivot if green button pressed
			if (buttonsGet(BUTTON_GREEN)) {
				isPivot = (isPivot) ? FALSE : TRUE;
			}

			 if (state == STATE_CGUESS) {
			 } else if (state == STATE_ROTATE) {
				 mrmRotateCW(&navData, &behOutput, 8);
			 }
		}

		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

int main(void) {
	systemInit();
	behaviorSystemInit(behaviorTask, 4096);
	osTaskStartScheduler();
	return 0;
}
