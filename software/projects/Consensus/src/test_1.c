/*
 * test_1.c
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

/* Main includes */
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

/* Our includes */
#include "consensus.h"

#define NEIGHBOR_ROUND_PERIOD	600
#define RPRINTF_SLEEP_TIME		30

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

	/* Enable visual LED feedback from the consensus system */
	consensusEnableFeedback(1);
	/* Initialize and begin consensus using averaging */
	pipelineAverageInit();

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

	/* Initialize variables and subsystems */
	behaviorTaskInit();

	for (;;) {
		lastWakeTime = osTaskGetTickCount();
		behOutput = behInactive;

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
	behaviorSystemInit(behaviorTask, 4096);
	osTaskStartScheduler();

	return 0;
}
