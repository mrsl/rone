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
#include "swarmOrchestra.h"

#define NEIGHBOR_ROUND_PERIOD	300
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

	synchInit();
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

		if (buttonsGet(BUTTON_RED)) {
			synchMakeClock();
		}

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
