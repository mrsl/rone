/**
 * @file https://svn.rice.edu/r/mrone/robotcode/MultiRobotManipulation/src/lightsensor_calibration.c
 *
 * @since: Dec 1, 2012
 * @author: Golnaz Habibi
 * @brief: reads the light sensors and prints the data to the terminal.
 */


#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50
#define NEIGHBOR_ROUND_PERIOD		300


void behaviorTask(void *parameters);
int main(void) {
	volatile uint32 val1, val2;
	systemPreInit();
	systemInit();


	val1 = osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);

	if ((val1 != pdTRUE)) {
		cprintf("could not create a task");
	}

	/* Start the scheduler. */
	osTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
   	 task. This is a very bad thing, and should not happen.*/
	return 0;
}


/** @brief: reads the light sensors and prints this data to the terminal
 *
 */
void behaviorTask(void* parameters) {
	//initialization
	uint32 lastWakeTime = osTaskGetTickCount();
	int32 light_value[4];

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	for (;;) {
		if (cprintfTerminalGetHost() == CPRINTF_TERMINAL_REMOTE) {
	
			light_value[0] = light_sensor_get_value(0)  ; // front right light sensor
			light_value[1] = light_sensor_get_value(1); // front left light sensor
			light_value[2] = light_sensor_get_value(2); // rear left light sensor
			light_value[3] = light_sensor_get_value(3) ; // rear right sensor

			cprintf("%d,%d,%d,%d\n",  light_value[0], light_value[1], light_value[2],light_value[3]);
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		}
	}
	return;
}

