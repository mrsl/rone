/*
 * SimpleDemo.c
 *
 *  Created on: Feb 13, 2013
 *      Author: Golnaz Habibi
 */


////////////////////////////////
/**
 * @file: https://svn.rice.edu/r/mrone/robotcode/MultiRobotManipulation/src/exp0a.c
 *
 * @since July 31st, 2012
 * @author Divya Bhat
 *
 * @brief: used in AAMAS paper to demonstrate multi-robot manipulation
 *
 * 1-  for light following ( no consensus) : Run exp0a.c
 * 2-  for leader following (consensus) : Run rotatelight.c
 *
 *
 *     TODO: K_PROPORTIONAL has been adjusted. Now K_INTEGRAL has to be adjusted (as you pick a high value and bring it down to stabilize
 *     		the robot, settle on one, and decrease K_PROPORTIONAL slightly if necessary). 0.001 may be the right value (the errors get very large).
 *
 *			Don't bother testing them while stationary-- it doesn't look right even with exact SuperDemo code.
 *     To get a better K_INTEGRAL_MAX, print out the integral values and see what range they tend to fall in. Pick a max based on that.
 *
 *  TODO: this code has no commenting.  Each function needs a description so we can easily fix problems.
 *
 */

//Behavior: move forward; if robot bumps into something, enter into flock behavior


#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define BEHAVIOR_TASK_PERIOD		50
#define NEIGHBOR_ROUND_PERIOD		300

#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50

#define K							5
#define SPEED						60
#define TV_MAX						100
#define RV_MAX						4000

#define K_INTEGRAL_MAX				1000 //400 works, 300 doesn't
#define DECAY_INTEGRAL              70

#define OBJECT_MASS					778
#define K_PI_N						100
#define K_PI_D					    100


#define K_I				            75
#define K_D				            25

#define FR_OFFSET				    90 // robot 64 = 85  ,  robot 61 = 90 , robot 62 =  00100
#define RR_OFFSET				    0


int32 currentTime;
Beh behMove;
Beh* behPtr = &behMove;

boolean isBumped = FALSE;
int32 sensor_offset = 0;
int32 integralError =  0;


void behaviorTask(void *parameters);
void backgroundTask(void *parameters);
int main(void) {
	volatile uint32 val1, val2;

	systemPreInit();
	systemInit();
	neighborsXmitEnable(TRUE);
	val1 = osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	val2 = osTaskCreate(backgroundTask, "background", 1024, NULL, BACKGROUND_TASK_PRIORITY);

	if ((val1 != pdTRUE)) {
		cprintf("could not create a task");
	}

	/* Start the scheduler. */
	osTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
   	 task. This is a very bad thing, and should not happen.*/
	return 0;
}


// the background task runs all the time
void backgroundTask(void* parameters) {
	for (;;) {
		// delay to let other tasks run at same priority
		osTaskDelay(100);
	}
}





void behaviorTask(void* parameters) {
	int32 TimeStart = 0;
	int32 TimeForThisSegment= 1000;
	for (;;) {
		if (cprintfTerminalGetHost() == CPRINTF_TERMINAL_REMOTE) {
			//		neighborsGetMutex();
			//		nbrListCreate(nbrList);
			currentTime = osTaskGetTickCount();


			}





				while(currentTime >= TimeStart+TimeForThisSegment ){
					currentTime = osTaskGetTickCount();

				}
				//Compute control law

				behSetTv(&behMove, 50);

				motorSetBeh(&behMove);

				//  we added 100 to the multiplication to make a tighter spiral.
				behSetRv(&behMove,  0);



			motorSetBeh(&behMove);
			//		neighborsPutMutex();

			//osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);

			//TODO: limit tv and rv.  (100 is a good limit?)
		}
	}










