/*
 * test_1.c
 *
 *  Created on: Oct 11, 2014
 *      Author: Zak
 */

/* Main includes */
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

/* Our includes */
#include "./consensus/consensus.h"

#define NEIGHBOR_ROUND_PERIOD	1500
#define RPRINTF_SLEEP_TIME		30

#define MOVE_STATE_IDLE	0
#define MOVE_STATE_MOVE	1

#define MOVE_TV			20

uint8 moveState = MOVE_STATE_IDLE;

/**
 * Initialization of subsystems and such
 */
void behaviorTaskInit() {
	/* Set our radio subnet */
	radioCommandSetSubnet(1);

	/* Initialize neighbor subsystem */
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	/* Set rprintf time */
	rprintfSetSleepTime(RPRINTF_SLEEP_TIME);

	/* Enable the external pose system */
	externalPoseInit();

	/* Filter really smooth */
	neighborsSetFilterTimeConstants(50, 50);

	/* Initialize callbacks */
	consensusCallbackInit();

	/* Enable visual LED feedback from the consensus system */
	consensusEnableFeedback(1);

	/* Initialize and begin consensus using averaging */
	pipelineCentroidInit();

	/* Status check */
//	systemPrintStartup();
//	systemPrintMemUsage();
}

/**
 * Main behavior task
 */
void behaviorTask(void* parameters) {
	uint32 lastWakeTime;	// The last time this task was woken

	Beh behOutput;			// Output motion behavior

	uint32 c = 0;	// Count to know when to print neighbors

	uint8 buttonOld;		// Was the button pressed last round?

	/* Initialize variables and subsystems */
	behaviorTaskInit();

	/* Disable consensus */
	consensusDisable();

	for (;;) {
		lastWakeTime = osTaskGetTickCount();
		behOutput = behInactive;

		/* Disable neighbor comms and just be a radio host */
		if (rprintfIsHost() || externalPoseIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_FAST);
			neighborsDisable();
			/* Delay task until next time */
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
			continue;
		}

		uint8 button = buttonsGet(BUTTON_RED);
		if (button && !buttonOld) {
			if (consensusIsEnabled()) {
				consensusDisable();
			} else {
				consensusEnable();
			}
		}
		buttonOld = button;

		if (!consensusIsEnabled()) {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			if (c > 20) {
				c = 20;
			}
		} else {
			switch (moveState) {
			case (MOVE_STATE_MOVE): {
				behIRObstacleAvoid(&behOutput, MOVE_TV, FALSE);
				if (!behIsActive(&behOutput)) {
					behMoveForward(&behOutput, MOVE_TV);
				}
				break;
			}
			case (MOVE_STATE_IDLE):
			default: {
				break;
			}
			}
		}

		/* Print out neighbor information */
		if (c == 200 || (c == 20 && !consensusIsEnabled())) {
			rprintf("N,%d", roneID);
			NbrList nbrList;
			nbrListCreate(&nbrList);
			uint8 i;
			for (i = 0; i < nbrList.size; i++) {
				Nbr *nbrPtr = nbrListGetNbr(&nbrList, i);
				if (nbrPtr == NULL) {
					break;
				} else {
					rprintf(",%d", nbrGetID(nbrPtr));
				}
			}
			rprintf("\n");
			rprintfFlush();
			c = 0;
		}
		c++;

		/* Set motion output */
		motorSetBeh(&behOutput);
		/* Delay task until next time */
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

/**
 * Startup
 */
int main(void) {
  	systemInit();
	behaviorSystemInit(behaviorTask, 2048);
	osTaskStartScheduler();

	return 0;
}
