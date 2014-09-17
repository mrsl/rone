/*
 * COMandTreeTesting.c
 *
 *  Created on: Aug 27, 2014
 *      Author: Zak, Golnaz
 */

#include "globalTreeCOM.h"

// Position estimation coordinates
scaleCoordinate GRLcentroidCooridates[GLOBAL_ROBOTLIST_MAX_SIZE];
scaleCoordinate GRLpivotCoordinate;
scaleCoordinate GRLguideCoordinate;

// Global robot list (GRL)
GlobalRobotList globalRobotList;
boolean GRLinit = FALSE;

// Point coordinates
navigationData navDataAvg;
navigationData navDataRead;

void mrmBehaviorInit() {
	navDataInit(&navDataAvg);
	navDataInit(&navDataRead);

	// Initialize neighbor subsystem
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	mrmInitCallbacks();

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
}

void behaviorTask(void* parameters) {
	uint32 lastWakeTime;	// The last time this task was woken

	Beh behOutput;			// Output motion behavior

	boolean nbrUpdate;		// Has the neighbor system updated?
	NbrList nbrList;		// The neighbor list
	uint32 neighborRound;	// The current neighbor round

	// Initialize variables and subsystems
	mrmBehaviorInit();

	for (;;) {
		lastWakeTime = osTaskGetTickCount();	// We have woken

		// Set robot to pivot robot if green button pressed
		if (buttonsGet(BUTTON_GREEN)) {
			setGRLpivot();
		// Set robot to guide robot if blue button pressed
		} else if (buttonsGet(BUTTON_BLUE)) {
			setGRLguide();
		}

		// If host, don't do anything
		if (rprintfIsHost() || externalPoseIsHost()) {
			behOutput = behInactive;
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		// If in idle state, also don't do anything
		} else if (getState() == STATE_IDLE) {
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
				if (!isInitStartNbrRound()) {
					setStartNbrRound(neighborRound);
				}

				// Print out some data
				switch (getState()) {
				case (STATE_CGUESS): {
					rprintf("%d,%d,%d,%d\n", navDataRead.centroidX,
										     navDataRead.centroidY,
										     navDataRead.childCountSum,
										     getDeltaStartNbrRound(neighborRound));
					rprintfFlush();
					break;
				}
				case (STATE_ROTATE):
				case (STATE_PIVOT): {
					rprintf("%d,%d,%d,%d,%d,%d\n", navDataRead.centroidX,
												   navDataRead.centroidY,
												   behGetTv(&behOutput),
												   behGetRv(&behOutput),
												   navDataRead.childCountSum,
												   getDeltaStartNbrRound(neighborRound));
					rprintfFlush();
					break;
				}
				default: {
					break;
				}
				}

				cprintf("pt %d,%d\n", navDataRead.centroidX / 10, navDataRead.centroidX / 10);
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
			} else if (roneID == getPivotRobot() && roneID == getGuideRobot()) {
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			} else {
				switch (getState()) {
				case (STATE_CGUESS): {
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
					break;
				}
				case (STATE_ROTATE):
				case (STATE_PIVOT):{
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
					break;
				}
				default: {
					break;
				}
				}
			}

			// Set motion based on state
			if (getState() == STATE_CGUESS) {
			} else if (getState() == STATE_ROTATE) {
				mrmOrbitCentroid(&navDataRead, &behOutput, MRM_TV_GAIN);
		 	} else if (getState() == STATE_ROTATE) {
		 		mrmOrbitPivot(&navDataRead, &behOutput, MRM_TV_GAIN);
		 	}
		}

		// Set motion output
		motorSetBeh(&behOutput);
		// Delay task until next time
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

/**
 * Set up system
 */
int main(void) {
  	systemInit();
	behaviorSystemInit(behaviorTask, 4096);
	osTaskStartScheduler();

	return 0;
}
