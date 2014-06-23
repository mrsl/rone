/*
 *	 globalRobotTree.c
 *  Created on: Jun 23, 2014
 *      Author: Mathew jellins
 */

#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD			300
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50
#define FLOCK_RV_GAIN					150
#define REALESE_WAIT_TIME				2000
#define LOST_OBJECT_WAIT				5000
#define CONV_ANGL						1100

#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50
#define TV_MIN							15

#define MODE_IDLE		 0
#define SEEKER_SEL		 1
#define SEEKER			 2
#define GRIPPED			 3
#define CONV_SEL		 4
#define CONVERGE		 5
#define MOVE_OBJ		 6
#define REMOTE			 8
#define RELEASE			 9

#define SEEKING			1
#define CONVERGING		2
#define GRIPPING		3
#define LEAVING			4


#define CNTCLK			1
#define CLKWISE			2
#define REST			0
#define ATTEMPTING		3


void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput;
	boolean printNow;
	uint32 neighborRound = 0;
	NbrList nbrList;

	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);

	NbrData msgNbrType;

	nbrDataCreate(&msgNbrType, "type", 3, 0);

	//uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;
	GlobalRobotList globalRobotListPtr;
	globalRobotListCreate(&globalRobotListPtr);


	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		}else{
			/*** INIT STUFF ***/
			behOutput = behInactive;
			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);
			//irCommsSetXmitPower(IRXmitPower);
			nbrListCreate(&nbrList);
			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			globalRobotListUpdate(&globalRobotListPtr, &nbrList);

			if(printNow){
				globalRobotListPrintAllTree(&globalRobotListPtr, &nbrList);
			}

			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			/*** READ BUTTONS ***/
			if (buttonsGet(BUTTON_RED)) {
			}else if (buttonsGet(BUTTON_GREEN)) {
			} else if (buttonsGet(BUTTON_BLUE)) {
			}

			/** STATES MACHINE **/
			switch (navigationMode) {
			}

			/*** FINAL STUFF ***/
			motorSetBeh(&behOutput);
			neighborsPutMutex();

			// delay until the next behavior period
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
			lastWakeTime = osTaskGetTickCount();
		}//end not host
	} // end for loop
} //end behavior function


/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}


