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

SerialCmd scLT;
SerialCmd scST;
RadioMessage rmSend;
RadioCmd rcSend;

uint32 neighborRound;
uint32 startNbrRound = 0;

uint8 state = STATE_IDLE;

#define RAVG_SIZE		50

struct {
	int16 x;
	int16 y;
} typedef coorAvg;

coorAvg ravg[RAVG_SIZE];
int ravg_ind = 0;
int ravg_size = 0;

void rollingAverageCentroid(navigationData *new, navigationData *avg) {
	int i;
	int32 x, y;

	ravg[ravg_ind].x = new->centroidX;
	ravg[ravg_ind].y = new->centroidY;

	ravg_ind = (ravg_ind + 1) % RAVG_SIZE;

	if (ravg_size < RAVG_SIZE) {
		ravg_size++;
	}

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].x;
		y += (int32) ravg[i].y;
	}
	avg->centroidX = (int16) (x / ravg_size);
	avg->centroidY = (int16) (y / ravg_size);
}


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

	navDataInit(&navDataAvg);
	navDataInit(&navDataRead);

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

	gripperBoardInit();

	externalPoseInit();

	for (;;) {
		// Default behavior is inactive
		lastWakeTime = osTaskGetTickCount();

		gripperBoardSetServo(90);

		// If host, do not do anything
		if (rprintfIsHost()) {
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
				globalRobotListUpdate(&globalRobotList, &nbrList);
				centroidGRLUpdate(&navDataRead, globalRobotList, &nbrList, GRLcentroidCooridates);

				if (startNbrRound == 0) {
					startNbrRound = neighborRound;
				}

				rprintf("%d, %d, %d, %d, %d, %u\n", navDataRead.centroidX,
													navDataRead.centroidY,
													navDataAvg.centroidX,
													navDataAvg.centroidY,
													navDataRead.childCountSum,
													neighborRound - startNbrRound);
				rprintfFlush();
				cprintf("pt %d,%d\n", navDataAvg.centroidX / 10, navDataAvg.centroidY / 10);
			}

			neighborsPutMutex();

			rollingAverageCentroid(&navDataRead, &navDataAvg);

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
				 mrmRotateCW(&navDataAvg, &behOutput, 8);
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
