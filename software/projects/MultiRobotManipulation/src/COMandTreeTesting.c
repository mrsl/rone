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

#define CENTROID_ALPHA	90

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 messageType;
	uint8 myId;
	uint8 theirId;
	int16 distance;
	uint8 state;
	char pad[20];
} typedef controlMsg;

void setState(uint8 newState);

SerialCmd scLT;
SerialCmd scST;
RadioMessage rmSend;
RadioCmd rcSend;

uint32 neighborRound;
uint32 startNbrRound = 0;

uint8 state = STATE_IDLE;

//#define RAVG_SIZE		10
//
//navigationData ravg[RAVG_SIZE];
//int ravg_ind = 0;
//int ravg_size = 0;
//
//void copyNavData(navigationData *toCopy, navigationData *toMe) {
//	toMe->centroidX = toCopy->centroidX;
//	toMe->centroidY = toCopy->centroidY;
//	toMe->guideX = toCopy->guideX;
//	toMe->guideY = toCopy->guideY;
//	toMe->pivotX = toCopy->pivotX;
//	toMe->pivotY = toCopy->pivotY;
//	toMe->childCountSum = toCopy->childCountSum;
//}
//
//void rollingAverageNavData(navigationData *new, navigationData *avg) {
//	int i;
//	int32 x, y;
//
//	copyNavData(new, &ravg[ravg_ind]);
//
//	ravg_ind = (ravg_ind + 1) % RAVG_SIZE;
//
//	if (ravg_size < RAVG_SIZE) {
//		ravg_size++;
//	}
//
//	x = 0;
//	y = 0;
//	for (i = 0; i < ravg_size; i++) {
//		x += (int32) ravg[i].centroidX;
//		y += (int32) ravg[i].centroidY;
//	}
//	avg->centroidX = (int16) (x / ravg_size);
//	avg->centroidY = (int16) (y / ravg_size);
//
//	x = 0;
//	y = 0;
//	for (i = 0; i < ravg_size; i++) {
//		x += (int32) ravg[i].pivotX;
//		y += (int32) ravg[i].pivotY;
//	}
//	avg->pivotX = (int16) (x / ravg_size);
//	avg->pivotY = (int16) (y / ravg_size);
//
//	x = 0;
//	y = 0;
//	for (i = 0; i < ravg_size; i++) {
//		x += (int32) ravg[i].guideX;
//		y += (int32) ravg[i].guideY;
//	}
//	avg->guideX = (int16) (x / ravg_size);
//	avg->guideY = (int16) (y / ravg_size);
//}


// Set up centroid and GRL
scaleCoordinate GRLcentroidCooridates[GLOBAL_ROBOTLIST_MAX_SIZE];
scaleCoordinate GRLpivotCoordinate;
scaleCoordinate GRLguideCoordinate;

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

	navigationData navDataAvg;
	navigationData navDataRead;

	cprintf("1\n");

	navDataInit(&navDataAvg);
	navDataInit(&navDataRead);

	cprintf("2\n");

	radioCommandSetSubnet(1);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	cprintf("3\n");

	serialCommandAdd(&scLT, "lt", scLTFunc);
	serialCommandAdd(&scST, "st", scSTFunc);
	radioCommandAddCallback(&rcSend, "RC", rcCallback);

	cprintf("4\n");


	createGRLscaleCoordinates(GRLcentroidCooridates);

	cprintf("5\n");
	createGRLpivotCoordinate(&GRLpivotCoordinate);

	cprintf("6\n");
	createGRLguideCoordinate(&GRLguideCoordinate);

	cprintf("7\n");
	globalRobotListCreate(&globalRobotList);

	cprintf("8\n");
	externalPoseInit();

	ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

	// Initialization steps
	systemPrintStartup();
	systemPrintMemUsage();

	for (;;) {
		// Default behavior is inactive
		lastWakeTime = osTaskGetTickCount();

		// Set to pivot if green button pressed
		if (buttonsGet(BUTTON_GREEN)) {
			setGRLpivot(roneID);
		} else if (buttonsGet(BUTTON_BLUE)) {
			setGRLguide(roneID);
		}

		// If host, do not do anything
		if (rprintfIsHost() || externalPoseIsHost()) {
			behOutput = behInactive;
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		} else if (state == STATE_IDLE) {
			behOutput = behInactive;
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
		} else {
			neighborsGetMutex();

			printNow = neighborsNewRoundCheck(&neighborRound);

			// If neighbor data has updated, print out new centroid estimate
			if (printNow) {
				nbrListCreate(&nbrList);

				updatePivotandGuide(&nbrList);

				globalRobotListUpdate(&globalRobotList, &nbrList);
				centroidGRLUpdate(&navDataRead, &globalRobotList, &nbrList, GRLcentroidCooridates);
				pivotGRLUpdate(&navDataRead, &globalRobotList, &nbrList, &GRLpivotCoordinate);
				guideGRLUpdate(&navDataRead, &globalRobotList, &nbrList, &GRLguideCoordinate);

				if (startNbrRound == 0) {
					startNbrRound = neighborRound;
				}

				rprintf("%d, %d, %d, %d, %d, %d\n", navDataRead.centroidX,
													navDataRead.centroidY,
													navDataAvg.centroidX,
													navDataAvg.centroidY,
													navDataRead.childCountSum,
													neighborRound - startNbrRound);
				rprintfFlush();

				//cprintf("pt %d,%d\n", navDataAvg.centroidX / 10, navDataAvg.centroidY / 10);
				cprintf("pt %d,%d\n", navDataRead.pivotX / 10, navDataRead.pivotY / 10);
				//cprintf("pt %d,%d\n", navDataAvg.guideX / 10, navDataAvg.guideY / 10);
			}

			neighborsPutMutex();

			//rollingAverageNavData(&navDataRead, &navDataAvg);

			// Set LEDs based on state
			if (roneID == getPivotRobot()) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else if (roneID == getGuideRobot()) {
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else {
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			}

			 if (state == STATE_CGUESS) {
			 } else if (state == STATE_ROTATE) {
				 mrmRotateCentroid(&navDataAvg, &behOutput, 8);
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
