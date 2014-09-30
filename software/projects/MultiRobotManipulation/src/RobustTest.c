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

	// Create distributed state information
	createStateInformation();

	globalRobotListCreate(&globalRobotList);

	// Initialize the external pose subsystem for location
	externalPoseInit();

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

	uint8 buttonRedPrev = 0;

	uint8 gripperEscape = 0;

	// Initialize variables and subsystems
	mrmBehaviorInit();

	for (;;) {
		behOutput = behInactive;
		lastWakeTime = osTaskGetTickCount();	// We have woken

		if (buttonsGet(BUTTON_RED)) {
			if (buttonRedPrev == 0) {
				if (getState() == STATE_IDLE) {
					setState(STATE_CGUESS);
				} else {
					setState(STATE_IDLE);
				}
			}
			buttonRedPrev = 1;
		} else {
			buttonRedPrev = 0;
		}

		// If a host, don't do anything
		if (rprintfIsHost() || externalPoseIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
				LED_BRIGHTNESS_LOW, LED_RATE_FAST);

		// If in idle state, also don't do anything
		} else {

			// Check for update
			nbrUpdate = neighborsNewRoundCheck(&neighborRound);

			// If we are idle
			if (getState() == STATE_IDLE) {
				neighborsXmitEnable(FALSE);
				ledsSetPattern(LED_GREEN, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

				// Print out some data
				if (nbrUpdate) {
					rprintf("0,0,0,0\n");
					rprintfFlush();
				}


			// If we are active
			} else {
				neighborsXmitEnable(TRUE);

				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_MED);

				// Lock the neighbor list
				neighborsGetMutex();

				nbrListCreate(&nbrList);

				// Only run the update on new neighbor data
				if (nbrUpdate) {
					// Update the GRL
					globalRobotListUpdate(&globalRobotList, &nbrList);

					// Update our position estimations
					centroidGRLUpdate(&navDataRead, &globalRobotList,
						&nbrList, GRLcentroidCooridates);

					// If this is the first neighbor round we are active, set our start
					if (!isInitStartNbrRound()) {
						setStartNbrRound(neighborRound);
					}

					// Print out some data
					rprintf("%d,%d,%d,%d\n", navDataRead.centroidX,
											 navDataRead.centroidY,
											 navDataRead.childCountSum,
											 getDeltaStartNbrRound(neighborRound));
					rprintfFlush();
				}

				// Calculate rolling average of estimates
				rollingAverageNavData(&navDataRead, &navDataAvg);

				cprintf("pt 3,%d,%d\n", navDataAvg.centroidX / 10, navDataAvg.centroidY / 10);

				// Unlock the neighbor list
				neighborsPutMutex();
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
