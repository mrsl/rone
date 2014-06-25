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

#define PI				3147

#define nbrEdgeDis		250				//hardcoded distance

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
	int i,j;
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
				Nbr* nbrPtr;
				for (j = 0; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++) {
					uint16 xtot = 0;
					uint16 ytot = 0;
					uint8 wieght = 0;

					for (i = 0; i < nbrList.size; i++){
						nbrPtr = nbrList.nbrs[i];
						uint8 nbrTreeParentId = nbrDataGetNbr(&(globalRobotListPtr.list[j].ParentID), nbrPtr);
						if(nbrTreeParentId == roneID){
							uint16 x,y,xprime,yprime,x_dob_prime,y_dob_prime,x_tri_prime,y_tri_prime;
							uint32 nbrOrient =  + PI;
							uint32 nbrBear = nbrGetBearing(nbrPtr) + PI;

							x = nbrDataGetNbr16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_H,nbrPtr);
							y = nbrDataGetNbr16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_H,nbrPtr);

							xprime = x*cosMilliRad(nbrOrient)/1000 - y*sinMilliRad(nbrOrient)/1000;
							yprime = x*sinMilliRad(nbrOrient)/1000 + y*cosMilliRad(nbrOrient)/1000;

							x_dob_prime = xprime;
							y_dob_prime = yprime + nbrEdgeDis;

							x_tri_prime = x_dob_prime*cosMilliRad(nbrBear)/1000 - y_dob_prime*sinMilliRad(nbrBear)/1000;
							y_tri_prime = x_dob_prime*sinMilliRad(nbrBear)/1000 + y_dob_prime*cosMilliRad(nbrBear)/1000;

							xtot +=  x_tri_prime;
							ytot +=  y_tri_prime;
							wieght++;
							if(printNow){rprintf("ID %d NbrID %d TrID %d W %d X %d Y %d X' %d Y' %d X'' %d, Y'' %d X''' %d Y''' %d XT %d YT %d XA %d YA %d\n",
									roneID, nbrGetID(nbrPtr),nbrDataGetNbr(&(globalRobotListPtr.list[j].ID), nbrPtr),wieght, x, y, xprime, yprime,
									x_dob_prime,y_dob_prime,x_tri_prime,y_tri_prime,xtot,ytot,xtot/wieght,ytot/wieght);}
						}
					}
					if(wieght == 0){
						nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,0);
						nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,0);
					}else{
						nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,xtot/wieght);
						nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,ytot/wieght);
					}
					//rprintf(" XT %d, YT %d\n", 	nbrDataGet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L), nbrDataGet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L));
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


