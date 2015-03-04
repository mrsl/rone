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
#include <math.h>

/* Our includes */
#include "./consensus/consensus.h"
#include "./consensus/operations/consensusPipelineMinMax.h"
#include "./tree/centroidLite.h"

#define NEIGHBOR_ROUND_PERIOD	2000
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
	neighborsSetFilterTimeConstants(40, 40);

	/* Initialize callbacks */
	consensusCallbackInit();

	/* Enable visual LED feedback from the consensus system */
	//consensusEnableFeedback(1);

	centroidLiteInit();

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

		uint8 button = buttonsGet(BUTTON_GREEN);
		if (button && !buttonOld) {
			if (consensusIsEnabled()) {
				consensusDisable();
			} else {
				consensusEnable();
			}
		}
		buttonOld = button;

		if (!externalPoseIsActive()) {
			audioNoteOffAll();
			audioNoteOn(CONSENSUS_INSTRUMENT, 100, CONSENSUS_VELOCITY, 200);
		}


		if (consensusIsEnabled()) {
			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
		} else {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_FAST);
		}

		NbrList nbrList;
		nbrListCreate(&nbrList);
		behFlock_gain(&behOutput, &nbrList, 0, 20);

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
