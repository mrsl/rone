/*
 *	 globalRobotTree.c
 *  Created on: Jun 23, 2014
 *      Author: Mathew jellins
 */

#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"
#include "globalTreeCOM.h"


#define NEIGHBOR_ROUND_PERIOD			300
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50

#define BUILD_TREE		 0
#define GUESS_COM		 1

#define PI				3147
#define nbrEdgeDis		500				//hardcoded distance
#define COM_WAIT		3
#define NORM_TV			125

#define REST			0
#define CNTCLK			1
#define CLKWISE			2
#define ATTEMPTING		3

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 state = BUILD_TREE;
	Beh behOutput = behInactive;
	boolean printNow;
	uint32 neighborRound = 0;
	NbrList nbrList;
	uint8 changeCOM = 0;
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);

	PosistionCOM treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE];
	creatGlobalTreeCOMList(treeGuessCOM);

	//uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;
	GlobalRobotList globalRobotList;
	globalRobotListCreate(&globalRobotList);

	gripperBoardInit();
	uint8 gripPos = REST;

	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		}else{
			/*** INIT STUFF ***/
			//behOutput = behInactive;
			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);
			//irCommsSetXmitPower(IRXmitPower);
			nbrListCreate(&nbrList);
			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			globalRobotListUpdate(&globalRobotList, &nbrList);

			if(printNow){
				//globalRobotListPrintAllTree(&globalRobotListPtr, &nbrList);
			}

			/*** READ BUTTONS ***/
			if (buttonsGet(BUTTON_RED)) {
			}else if (buttonsGet(BUTTON_GREEN)) {
			} else if (buttonsGet(BUTTON_BLUE)) {
				state = GUESS_COM;
			}

			/** STATES MACHINE **/
			switch (state) {
			case BUILD_TREE:{
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				if(globalRobotList.size >= GLOBAL_ROBOTLIST_MAX_SIZE){
					state = GUESS_COM;
				}
				break;
			}
			case GUESS_COM:{
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			}


			if(!gripperBoardGetGripped()){
				if(gripPos != ATTEMPTING){
					gripperGripUntilGripped();
					gripPos = ATTEMPTING;
				}
			}else{
				gripPos = CLKWISE;
				if(gripperBoardGetServo() > 100){
					gripPos = CNTCLK;
				}
			}

			if(state == GUESS_COM){
				updateGlobalTreeCOM(globalRobotList, nbrList, treeGuessCOM, nbrEdgeDis);

				if(changeCOM >= COM_WAIT){
					changeCOM = 0;
					int16 COM_Y,COM_X;
					int8 selfIdx = globalRobotListGetIndex(&globalRobotList,roneID);
					if(selfIdx == -1){
						COM_Y = 0;
					}else{
						COM_Y =  nbrDataGet16(&treeGuessCOM[selfIdx].Y_H,&treeGuessCOM[selfIdx].Y_L);
						COM_X =  nbrDataGet16(&treeGuessCOM[selfIdx].X_H,&treeGuessCOM[selfIdx].X_L);
					}

					orbitGlobalTreePoint(COM_X, COM_Y, &behOutput,  NORM_TV);
				}else{
					changeCOM++;
				}
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


