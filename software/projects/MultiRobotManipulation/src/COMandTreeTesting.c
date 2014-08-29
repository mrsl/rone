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

	boolean buttonRed,		// Buttons
			buttonGreen,
			buttonBlue;

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

	for (;;) {
		// Default behavior is inactive
		behOutput = behInactive;

		// If host, do not do anything
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		} else {
			neighborsGetMutex();

			printNow = neighborsNewRoundCheck(&neighborRound);
			nbrListCreate(&nbrList);

			globalRobotListUpdate(&globalRobotList, &nbrList);
			centroidGRLUpdate(globalRobotList, nbrList, GRLcentroidCooridates);

			//CentroidGRLPrintAllTrees(&globalRobotList, &nbrList, &GRLcentroidCooridates);
			//CentroidGRLPrintSelfTree(&globalRobotList, &GRLcentroidCooridates);

//			int16 x, y;
//			int16 xCoor, yCoor;
//			int32 orientation, bearing;
//			uint8 childCount;
//
//			cprintf(" xResult | yResult |   xCoor |   yCoor |    xNbr |    yNbr |  orient. | bearing | childC. \n");
//
//			xCoor = 0, yCoor = 0;
//			orientation = 0;
//			bearing = 0;
//			childCount = 1;
//			applyTransformationMatrix(&x, &y, xCoor, yCoor, orientation, bearing, childCount);
//			xCoor = 100;
//			childCount = 2;
//			applyTransformationMatrix(&x, &y, xCoor, yCoor, orientation, bearing, childCount);
//			xCoor = -100;
//			orientation = PI;
//			bearing = PI;
//			childCount = 2;
//			applyTransformationMatrix(&x, &y, xCoor, yCoor, orientation, bearing, childCount);


			//cprintf("\n");

			rprintfFlush();

			//CentroidGRLPrintEstimate(&globalRobotList, &GRLcentroidCooridates);
			//rprintfFlush();


			// Get buttons
			buttonBlue = buttonsGet(BUTTON_BLUE);
			buttonRed = buttonsGet(BUTTON_RED);
			buttonGreen = buttonsGet(BUTTON_GREEN);

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
