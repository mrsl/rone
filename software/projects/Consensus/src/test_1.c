/*
 * test_1.c
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#include "consensus.h"

#define STATE_WAIT	0
#define STATE_ACK	1
#define STATE_IDLE	2

/**
 * Initialization of subsystems and such
 */
void behaviorTaskInit() {
	// Set our subnet
	radioCommandSetSubnet(1);

	// Initialize neighbor subsystem
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	// Set rprintf time
	rprintfSetSleepTime(RPRINTF_SLEEP_TIME);

	// Set audio volume
	audioVolume(230);

	// Initialize the external pose subsystem for location
	externalPoseInit();

	consensusNoOpInit();

	// Status check
	systemPrintStartup();
	systemPrintMemUsage();
}

/**
 * Main behavior task
 */
void behaviorTask(void* parameters) {
	uint32 lastWakeTime;		// The last time this task was woken

	Beh behOutput;				// Output motion behavior

//	boolean nbrUpdate;			// Has the neighbor system updated?
//	NbrList nbrList;			// The neighbor list
//	uint32 neighborRound;		// The current neighbor round

	uint8 mode = STATE_WAIT;

	// Initialize variables and subsystems
	behaviorTaskInit();

	for (;;) {

		lastWakeTime = osTaskGetTickCount();	// We have woken
		behOutput = behInactive;

		if (buttonsGet(BUTTON_RED)) {
			consensusSetAckChance(0);
		} else if (buttonsGet(BUTTON_GREEN)) {
			consensusSetAckChance(500);
		} else if (buttonsGet(BUTTON_BLUE)) {
			consensusSetAckChance(1000);
		}

//		switch (mode) {
//		case (STATE_ACK): {
//			consensusAckMode();
//			break;
//		} case (STATE_IDLE): {
//			consensusIdleMode();
//			break;
//		}
//		case (STATE_WAIT):
//		default: {
//			break;
//		}
//		}

		// Set motion output
		motorSetBeh(&behOutput);
		// Delay task until next time
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
