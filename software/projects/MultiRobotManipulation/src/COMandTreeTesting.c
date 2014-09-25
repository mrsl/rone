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

uint8 gripPos = -1;

void mrmBehaviorInit() {
	navDataInit(&navDataAvg);
	navDataInit(&navDataRead);

	// Initialize neighbor subsystem
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	// Initialize rprintf time to slower value
	rprintfSetSleepTime(RPRINTF_SLEEP_TIME);

	mrmInitCallbacks();

	// Initialize GRL and centroid, pivot, and guide robot position estimations
	createGRLscaleCoordinates(GRLcentroidCooridates);
	createGRLpivotCoordinate(&GRLpivotCoordinate);
	createGRLguideCoordinate(&GRLguideCoordinate);

	// Create distributed state information
	createStateInformation();

	globalRobotListCreate(&globalRobotList);

	// Initialize the external pose subsystem for location
	externalPoseInit();

	// Gripper stuff
//	gripperBoardInit();
//	gripperCalibratServo();

	// Status check
	systemPrintStartup();
	systemPrintMemUsage();
}

void behaviorTask(void* parameters) {
	uint32 lastWakeTime;		// The last time this task was woken

	Beh behOutput;				// Output motion behavior

	boolean nbrUpdate;			// Has the neighbor system updated?
	NbrList nbrList;			// The neighbor list
	uint32 neighborRound;	// The current neighbor round

	uint8 gripperEscape = 0;

	// Initialize variables and subsystems
	mrmBehaviorInit();

	for (;;) {
//		// Calibrate gripper
//		if(!gripperServoCalibratFinish() && !gripperEscape){
//			if (buttonsGet(BUTTON_RED)) {
//				gripperEscape = 1;
//			}
//			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE,
//				LED_BRIGHTNESS_LOW, LED_RATE_FAST);
//			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
//			lastWakeTime = osTaskGetTickCount();
//			continue;
//		}

		lastWakeTime = osTaskGetTickCount();	// We have woken

		// Set robot to pivot robot if green button pressed
		if (buttonsGet(BUTTON_GREEN)) {
			(roneID != getPivotRobot()) ? setGRLpivot(roneID) : setGRLpivot(0);

		// Set robot to guide robot if blue button pressed
		} else if (buttonsGet(BUTTON_BLUE)) {
			(roneID != getGuideRobot()) ? setGRLguide(roneID) : setGRLguide(0);

		// Increment through states
		} else if (buttonsGet(BUTTON_RED)) {
			setState((getState() + 1) % (STATE_MAX + 1));
		}

//		// Attempt to grip until you are gripped, then stay gripped
//		if (!gripperBoardGetGripped()) {
//			if (gripPos != ATTEMPTING) {
//				gripperGripUntilGripped();
//				gripPos = ATTEMPTING;
//			}
//		} else {
//			gripPos = CLKWISE;
//			if (gripperBoardGetServo() > 100) {
//				gripPos = CNTCLK;
//			}
//		}

		// If a host, don't do anything
		if (rprintfIsHost() || externalPoseIsHost()) {
			behOutput = behInactive;
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
				LED_BRIGHTNESS_LOW, LED_RATE_FAST);

		// If in idle state, also don't do anything
		} else {
			// Lock the neighbor list
			neighborsGetMutex();

			nbrListCreate(&nbrList);

			// Check for update
			nbrUpdate = neighborsNewRoundCheck(&neighborRound);

			// If neighbor data has updated, update our guesses
			if (nbrUpdate) {
				// Update pivot and guide robot IDs, as well as state
				updateDistributedInformation(&nbrList);
			}

			// If we are idle
			if (getState() == STATE_IDLE) {
				behOutput = behInactive;
				ledsSetPattern(LED_GREEN, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			// If we are active
			} else {
				// Only run the update on new neighbor data
				if (nbrUpdate) {
					// Update the GRL
					globalRobotListUpdate(&globalRobotList, &nbrList);

					// Update our position estimations
					centroidGRLUpdate(&navDataRead, &globalRobotList,
						&nbrList, GRLcentroidCooridates);

					pivotGRLUpdate(&navDataRead, &globalRobotList,
						&nbrList, &GRLpivotCoordinate);

					guideGRLUpdate(&navDataRead, &globalRobotList,
						&nbrList, &GRLguideCoordinate);

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
					case (STATE_RALIGN):
					case (STATE_ROTATE): {
						rprintf("%d,%d,%d,%d\n", navDataRead.centroidX,
												 navDataRead.centroidY,
												 navDataRead.childCountSum,
												 getDeltaStartNbrRound(neighborRound));
						rprintfFlush();
						break;
					}
					case (STATE_PALIGN):
					case (STATE_PIVOT): {
						break;
					}
					case (STATE_TALIGN):
					case (STATE_FTRANS):
					case (STATE_BTRANS): {
						break;
					}
					case (STATE_CALIGN):
					case (STATE_CYCLD): {
						rprintf("%d,%d,%d,%d,%d,%d\n", navDataRead.centroidX,
													   navDataRead.centroidY,
													   navDataRead.guideX,
													   navDataRead.guideY,
													   navDataRead.childCountSum,
													   getDeltaStartNbrRound(neighborRound));
						rprintfFlush();
						break;
					}
					default: {
						break;
					}
					}

					//cprintf("pt 0,%d,%d\n", navDataRead.centroidX / 10, navDataRead.centroidY / 10);
					//cprintf("pt 1,%d,%d\n", navDataRead.pivotX / 10, navDataRead.pivotY / 10);
					//cprintf("pt 1,%d,%d\n", navDataRead.guideX / 10, navDataRead.guideY / 10);

				}

				// Calculate rolling average of estimates
				rollingAverageNavData(&navDataRead, &navDataAvg);
				cprintf("pt 3,%d,%d\n", navDataAvg.centroidX / 10, navDataAvg.centroidY / 10);
				cprintf("pt 4,%d,%d\n", navDataAvg.guideX / 10, navDataAvg.guideY / 10);

				// Set LEDs based on state
				if (roneID == getPivotRobot() && roneID == getGuideRobot()) {
					ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
				} else if (roneID == getGuideRobot()) {
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
				} else if (roneID == getPivotRobot()) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
				} else {
					switch (getState()) {
					case (STATE_CGUESS): {
						ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
						break;
					}
					case (STATE_RALIGN): {
						ledsSetPattern(LED_BLUE, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
						break;
					}
					case (STATE_ROTATE): {
						ledsSetPattern(LED_RED, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
						break;
					}
					case (STATE_PALIGN): {
						ledsSetPattern(LED_BLUE, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
						break;
					}
					case (STATE_PIVOT): {
						ledsSetPattern(LED_RED, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
						break;
					}
					case (STATE_TALIGN): {
						ledsSetPattern(LED_BLUE, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
						break;
					}
					case (STATE_FTRANS):
					case (STATE_BTRANS): {
						ledsSetPattern(LED_RED, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
						break;
					}
					case (STATE_CALIGN): {
						ledsSetPattern(LED_BLUE, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
						break;
					}
					case (STATE_CYCLD): {
						ledsSetPattern(LED_RED, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
						break;
					}
					default: {
						ledsSetPattern(LED_ALL, LED_PATTERN_ON,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
						break;
					}
					}
				}

				// Set motion based on state
				if (roneID != getGuideRobot()) {
					switch (getState()) {
					case (STATE_CGUESS): {
						behOutput = behInactive;
						break;
					}
					case (STATE_RALIGN): {
						mrmOrbitCentroid(&navDataAvg, &behOutput, 0);
						break;
					}
					case (STATE_ROTATE): {
						mrmOrbitCentroid(&navDataAvg, &behOutput, MRM_ROTATE_TV_GAIN);
						break;
					}
					case (STATE_PALIGN): {
						mrmOrbitPivot(&navDataAvg, &behOutput, 0);
						break;
					}
					case (STATE_PIVOT): {
						mrmOrbitPivot(&navDataAvg, &behOutput, MRM_PIVOT_TV_GAIN);
						break;
					}
					case (STATE_TALIGN): {
						mrmTranslateLeaderToGuideVector(&navDataAvg,
							&behOutput, 0);
						break;
					}
					case (STATE_FTRANS): {
						mrmTranslateLeaderToGuideVector(&navDataAvg,
							&behOutput, MRM_TRANS_TV_GAIN);
						break;
					}
					case (STATE_BTRANS): {
						mrmTranslateLeaderToGuideVector(&navDataAvg,
							&behOutput, MRM_TRANS_TV_GAIN);
						break;
					}
					case (STATE_CALIGN): {
						mrmCycloidMotion(&navDataAvg,
							&behOutput, 0);
						break;
					}
					case (STATE_CYCLD): {
						mrmCycloidMotion(&navDataAvg,
							&behOutput, MRM_CYCLD_TV_GAIN);
						break;
					}
					default: {
						behOutput = behInactive;
						break;
					}
					}
				} else {
					behOutput = behInactive;
				}

			}
		}

		// Unlock the neighbor list
		neighborsPutMutex();
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
