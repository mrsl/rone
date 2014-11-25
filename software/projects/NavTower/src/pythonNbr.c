/*
 * pythonNbr.c
 *
 *  Created on: Nov 12, 2012
 *      Author: Jeremy Hunt
 */

#include "roneos.h"
#include "ronelib.h"

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50
void behaviorTask(void* parameters);

int main(void){
	systemPreInit();
	systemInit();
	systemPrintStartup();
	neighborsInitPython(330);

	osTaskCreate(behaviorTask, "behavior", 1024, NULL, BEHAVIOR_TASK_PRIORITY);

	/* Start the scheduler. */
	osTaskStartScheduler();
	// Will only hit this point if there is not enough memory to schedule the tasks
	return 0;
}


void behaviorTask (void* parameters){
	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 counter = 0;

	NbrMsg msgMessage;

	nbrMsgCreate(&msgMessage, "message", 8, 0);


	// Set the robot to broadcast an 'h'
	nbrMsgSet(&msgMessage, (uint8)'h');

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
