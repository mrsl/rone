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
#define NEIGHBOR_ROUND_PERIOD			300

void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();

	Beh behOutput;
	boolean printNow;
	NbrList nbrList;
	uint32 neighborRound;

	navigationData navData;

	// Initialization steps
	systemPrintStartup();
	systemPrintMemUsage();

	radioCommandSetSubnet(1);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	// Set up COM
	scaleCoordinate GRLcentroidCooridates[GLOBAL_ROBOTLIST_MAX_SIZE + 2];
	createGRLscaleCoordinates(GRLcentroidCooridates);

	// Set up GRL
	GlobalRobotList globalRobotList;
	globalRobotListCreate(&globalRobotList);

//	setLookup(8, 17, 4000, PI, -PI / 2);
//	setLookup(8, 31, 4000, -PI / 2, 0);
//	setLookup(8, 59, 5657, PI / 4 - PI, PI / 4);
//	setLookup(17, 31, 5657, PI / 4 - PI, PI / 4);
//	setLookup(17, 59, 4000, PI, PI / 2);
//	setLookup(31, 59, 4000, PI / 2, 0);


	setLookup(25, 10, 4000, -PI / 2, PI / 2);
	setLookup(25, 31, 4000, 0, PI);
	setLookup(10, 02, 4000, 0, PI);
	setLookup(31, 02, 4000, -PI / 2, PI / 2);
	setLookup(02, 25, 5657, -PI / 2, PI / 2);
	setLookup(31, 10, 5657, -PI / 2, PI / 2);

	for (;;) {
		// Default behavior is inactive
		behOutput = behInactive;

		// If host, do not do anything
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		} else {
			// Set LEDs based on state
			if (isPivot) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else if (roneID == GUIDE_ROBOT_ID) {
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else {
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			}

			// Set to pivot if green button pressed
			if (buttonsGet(BUTTON_GREEN)) {
				isPivot = (isPivot) ? FALSE : TRUE;
			}

			neighborsGetMutex();

			printNow = neighborsNewRoundCheck(&neighborRound);

			// If neighbor data has updated, print out new centroid estimate
			if (printNow) {
				nbrListCreate(&nbrList);

				globalRobotListUpdate(&globalRobotList, &nbrList);
				centroidGRLUpdate(&navData, globalRobotList, &nbrList, GRLcentroidCooridates);

				rprintf("(%d, %d) - %d\n", navData.centroidX, navData.centroidY, navData.childCountSum);
				rprintfFlush();
			}

			neighborsPutMutex();
		}

		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		lastWakeTime = osTaskGetTickCount();
	}
}

int main(void) {
	systemInit();
	behaviorSystemInit(behaviorTask, 4096);
	osTaskStartScheduler();
	return 0;
}
