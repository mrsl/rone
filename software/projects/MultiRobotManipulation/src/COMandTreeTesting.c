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

void setState(uint8 newState);

// Callback variables
SerialCmd scLT;
SerialCmd scST;
RadioMessage rmSend;
RadioCmd rcSend;

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 messageType;
	uint8 myId;
	uint8 theirId;
	int16 distance;
	uint8 state;
	char pad[20];
} typedef controlMsg;

// State machine stuff
uint32 startNbrRound = 0;
uint8 state = STATE_IDLE;

// Position estimation coordinates
scaleCoordinate GRLcentroidCooridates[GLOBAL_ROBOTLIST_MAX_SIZE];
scaleCoordinate GRLpivotCoordinate;
scaleCoordinate GRLguideCoordinate;

// Global robot list (GRL)
GlobalRobotList globalRobotList;
boolean GRLinit = FALSE;

#define RAVG_SIZE		50

navigationData ravg[RAVG_SIZE];
int ravg_ind = 0;
int ravg_size = 0;

void copyNavData(navigationData *toCopy, navigationData *toMe) {
	toMe->centroidX = toCopy->centroidX;
	toMe->centroidY = toCopy->centroidY;
	toMe->guideX = toCopy->guideX;
	toMe->guideY = toCopy->guideY;
	toMe->pivotX = toCopy->pivotX;
	toMe->pivotY = toCopy->pivotY;
	toMe->childCountSum = toCopy->childCountSum;
}

void rollingAverageNavData(navigationData *new, navigationData *avg) {
	int i;
	int32 x, y;

	copyNavData(new, &ravg[ravg_ind]);

	ravg_ind = (ravg_ind + 1) % RAVG_SIZE;

	if (ravg_size < RAVG_SIZE) {
		ravg_size++;
	}

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].centroidX;
		y += (int32) ravg[i].centroidY;
	}
	avg->centroidX = (int16) (x / ravg_size);
	avg->centroidY = (int16) (y / ravg_size);

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].pivotX;
		y += (int32) ravg[i].pivotY;
	}
	avg->pivotX = (int16) (x / ravg_size);
	avg->pivotY = (int16) (y / ravg_size);

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].guideX;
		y += (int32) ravg[i].guideY;
	}
	avg->guideX = (int16) (x / ravg_size);
	avg->guideY = (int16) (y / ravg_size);
}

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

	// If not a valid message, throw it out
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

/**
 * Sets our state for the FSM
 */
void setState(uint8 newState) {
	state = newState;

	if (newState == STATE_IDLE) {
		startNbrRound = 0;
	}
}

/**
 * Initialize a navigationData struct
 */
void navDataInit(navigationData *navData) {
	navData->centroidX = 0;
	navData->centroidY = 0;
	navData->guideX = 0;
	navData->guideY = 0;
	navData->pivotX = 0;
	navData->pivotY = 0;
}

void behaviorTask(void* parameters) {
	uint32 lastWakeTime;	// The last time this task was woken

	Beh behOutput;			// Output motion behavior

	boolean nbrUpdate;		// Has the neighbor system updated?
	NbrList nbrList;		// The neighbor list
	uint32 neighborRound;	// The current neighbor round

	// Initialize point coordinates
	navigationData navDataAvg;
	navigationData navDataRead;

	navDataInit(&navDataAvg);
	navDataInit(&navDataRead);

	// Initialize neighbor subsystem
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	// Initialize callback hooks for serial and radio messages
	serialCommandAdd(&scLT, "lt", scLTFunc);
	serialCommandAdd(&scST, "st", scSTFunc);
	radioCommandAddCallback(&rcSend, "RC", rcCallback);

	// Initialize GRL and centroid, pivot, and guide robot position estimations
	createGRLscaleCoordinates(GRLcentroidCooridates);
	createGRLpivotCoordinate(&GRLpivotCoordinate);
	createGRLguideCoordinate(&GRLguideCoordinate);

	globalRobotListCreate(&globalRobotList);

	// Initialize the external pose subsystem for location
	externalPoseInit();

	// Status check
	systemPrintStartup();
	systemPrintMemUsage();

	for (;;) {
		lastWakeTime = osTaskGetTickCount();	// We have woken

		// Set robot to pivot robot if green button pressed
		if (buttonsGet(BUTTON_GREEN)) {
			setGRLpivot(roneID);
		// Set robot to guide robot if blue button pressed
		} else if (buttonsGet(BUTTON_BLUE)) {
			setGRLguide(roneID);
		}

		// If host, don't do anything
		if (rprintfIsHost() || externalPoseIsHost()) {
			behOutput = behInactive;
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		// If in idle state, also don't do anything
		} else if (state == STATE_IDLE) {
			behOutput = behInactive;
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
		// If we are active
		} else {
			// Lock the neighbor list
			neighborsGetMutex();

			// Check for update
			nbrUpdate = neighborsNewRoundCheck(&neighborRound);

			// If neighbor data has updated, update our guesses
			if (nbrUpdate) {
				nbrListCreate(&nbrList);

				// Update pivot and guide robot IDs
				updatePivotandGuide(&nbrList);

				// Update the GRL
				globalRobotListUpdate(&globalRobotList, &nbrList);

				// Update our position estimations
				centroidGRLUpdate(&navDataRead, &globalRobotList, &nbrList, GRLcentroidCooridates);
				pivotGRLUpdate(&navDataRead, &globalRobotList, &nbrList, &GRLpivotCoordinate);
				guideGRLUpdate(&navDataRead, &globalRobotList, &nbrList, &GRLguideCoordinate);

				// If this is the first neighbor round we are active, set our start
				if (startNbrRound == 0) {
					startNbrRound = neighborRound;
				}

				// Print out some data
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

			// Unlock the neighbor list
			neighborsPutMutex();

			// Calculate rolling average of estimates
			rollingAverageNavData(&navDataRead, &navDataAvg);

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

			// Set motion based on state
			if (state == STATE_CGUESS) {
			} else if (state == STATE_ROTATE) {
				mrmRotateCentroid(&navDataAvg, &behOutput, 8);
		 	}
		}

		// Set motion output
		motorSetBeh(&behOutput);
		// Delay task until next time
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

int main(void) {
  	systemInit();
	behaviorSystemInit(behaviorTask, 4096);
	osTaskStartScheduler();
	return 0;
}
