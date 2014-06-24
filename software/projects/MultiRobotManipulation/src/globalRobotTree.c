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

#define BUILD_TREE		 0
#define GUESS_COM		 1



typedef struct posCOM {
	NbrData X_H;
	NbrData X_L;
	NbrData Y_H;
	NbrData Y_L;
} posCOM;

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 state = BUILD_TREE;
	Beh behOutput;
	boolean printNow;
	uint32 neighborRound = 0;
	NbrList nbrList;
	int i;
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);

	NbrData msgNbrType;

	nbrDataCreate(&msgNbrType, "type", 3, 0);

	posCOM treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE];
	for(i = 0; i <GLOBAL_ROBOTLIST_MAX_SIZE; i++){
		nbrDataCreate16(&treeGuessCOM[i].X_H,&treeGuessCOM[i].X_L,"X_H", "X_L", 0);
		nbrDataCreate16(&treeGuessCOM[i].Y_H,&treeGuessCOM[i].Y_L,"Y_H", "Y_L", 0);
	}

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

			/*** READ BUTTONS ***/
			if (buttonsGet(BUTTON_RED)) {
			}else if (buttonsGet(BUTTON_GREEN)) {
			} else if (buttonsGet(BUTTON_BLUE)) {
			}

			/** STATES MACHINE **/
			switch (state) {
			case BUILD_TREE:{
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				if(globalRobotListPtr.size >= GLOBAL_ROBOTLIST_MAX_SIZE){
					state = GUESS_COM;
				}
				break;
			}
			case GUESS_COM:{
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			}

			if(state == GUESS_COM){

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


