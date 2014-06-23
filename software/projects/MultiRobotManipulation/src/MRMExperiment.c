/*
 * Main File to load on robots for MRM testing
 *
 * Buttons:
 *    RED - Set as TRANSPORT bot
 *    GREEN - Set as GUIDE bot
 *    BLUE - Empty
 *
 * TRANSPORT Buttons:
 * 	  RED - Set as TRANSPORT leader bot
 * 	  GREEN - Start transport
 * 	  BLUE - Stop transport
 *
 * GUIDE Buttons:
 *    RED - Show lowest weight path
 *    GREEN - Set as goal location
 *    BLUE - Empty
 *
 * Author: Lauren Schmidt
 * Date: Sept. 17, 2013
 */
#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"

/*** CONSTANTS ***/
#define INACTIVE 			0
#define GUIDE   			1
#define TRANSPORT           2

void behaviorTask(void* parameters) {
	Beh behOutput;
	uint8 botMode = INACTIVE;
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	for(;;){
		/*** READ BUTTONS ***/

		if (buttonsGet(BUTTON_RED)) {
			//Set as TRANSPORT bot
			botMode = TRANSPORT;
		} else if (buttonsGet(BUTTON_GREEN)) {
			//Set as TRANSPORT bot
			botMode = GUIDE;
		} else if (buttonsGet(BUTTON_BLUE)) {
			//empty
		}

		/*** Behave as Bot ***/
		switch(botMode){
		case(INACTIVE): {
			behOutput = behInactive;
			break;
		}
		case(TRANSPORT): {
			behTransport(&behOutput);
			break;
		}
		case(GUIDE): {
			behGuide(&behOutput);
			break;
		}
		default: {
			break;
		}
		}

		/*** Set Behavior and Finish Round ***/
		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		lastWakeTime = osTaskGetTickCount();

	}

}

/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);
	osTaskCreate(backgroundTask, "background", 1536, NULL,BACKGROUND_TASK_PRIORITY);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
