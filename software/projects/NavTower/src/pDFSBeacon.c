/*
 * Beacon.c
 *
 *  Created on: Jun 27, 2012
 *      Author: Melanie
 */

#include "roneos.h"
#include "ronelib.h"
#include "pDFSBeacon.h"

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50
#define BEACON_TYPE                     DEAD_END_BEACON

void behaviorTask(void* parameters);

void createMessage(robotMessage *rm) {

	nbrMsgCreate(&rm->msgSourceIDJunction, "junctionSourceID", 7, 0);
	nbrMsgCreate(&rm->msgHopsJunction, "junctionHops", 3, MAX_HOPS);
	nbrMsgCreate(&rm->msgJunctionTimeStamp, "junctionTimeStamp", 4, 0);
	nbrMsgCreate(&rm->msgRobotType, "robotType", 3, BEACON_TYPE);
	nbrMsgCreate(&rm->deadEndID, "deadEndID", 7, 0);
	nbrMsgCreate(&rm->deadEndValid, "deadEndValid", 1, 0);

}

int main(void) {
	systemPreInit();
	systemInit();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	neighborsXmitEnable(TRUE);
	osTaskCreate(behaviorTask, "behavior", 1024, NULL, BEHAVIOR_TASK_PRIORITY);

	/* Start the scheduler. */
	osTaskStartScheduler();
	// Will only hit this point if there is not enough memory to schedule the tasks
	return 0;
}


void behaviorTask (void* parameters) {

	uint32 lastWakeTime = osTaskGetTickCount();

	robotMessage rm;
	createMessage(&rm);

	while(1) {


		ledsSetAll(20);
		if (BEACON_TYPE == JUNCTION_BEACON) {
			osTaskDelay(200);
			ledsSetAll(0);
		}


		//ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}



