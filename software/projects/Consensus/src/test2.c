/*
 * test_1.c
 *
 *  Created on: Oct 11, 2014
 *      Author: Zak
 */

/* Main includes */
#include <stdlib.h>
#include <stdio.h>
#include "roneos.h"
#include "ronelib.h"

/* Our includes */
#include "./consensus/consensus.h"

#include "./operations/consensusPipelineMinMax.h"

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
	consensusPipelineMinMaxInit();

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
			case (MOVE_STATE_IDLE):
			default: {
				break;
			}
			}
		}

		float centroidX, centroidY;
		float posDiff;
		float posMult;

		consensusPipelineMinMaxGetCentroid(&centroidX, &centroidY);

		consensusPipelineMinMaxGetPosDiff(&posDiff);
		consensusPipelineMinMaxGetPosMult(&posMult);

		char buffer[100];

		sprintf(buffer, "CX: %.3f CY: %.3f PD: %.3f PM: %.3f\n", centroidX, centroidY, posDiff, posMult);
		cprintf(buffer);


//		consensusPipelineMinMaxSetPosDiff(centroidX);
//		consensusPipelineMinMaxSetPosMult(3);

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
