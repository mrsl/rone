/*
 * irCommsTest.c
 *
 *  Created on: Jun 8, 2012
 *      Author: Lindsay
 */

#include "roneos.h"
#include "ronelib.h"


void behaviorTask (void* parameters){
	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 counter = 0;

	// Start the neighbors system
	neighborsInit(330);

	while(TRUE){
		counter++;
		if (counter > 5) {
			ledsSetAll(20);
			if (counter > 10) {
				counter = 0;
			}
		} else {
			ledsSetAll(0);
		}
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}


int main(void) {
	// init the rone hardware and roneos services
	systemInit();
	systemPrintStartup();

	// init the behavior system and start the behavior thread
	//behaviorSystemInit(behaviorTask, 2048);
	osTaskCreate(behaviorTask, "behavior", 2048, NULL, BEHAVIOR_TASK_PRIORITY);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
