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
#define NORM_RV			250
#define NORM_TV			0


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

			if(state == GUESS_COM){
				/*Nbr* nbrPtr;
				for (j = 0; j < globalRobotListPtr.size; j++) {
					int32 xtot = 0;
					int32 ytot = 0;
					uint8 wieght = 0;

					for (i = 0; i < nbrList.size; i++){
						nbrPtr = nbrList.nbrs[i];
						uint8 nbrTreeParentId = nbrDataGetNbr(&(globalRobotListPtr.list[j].ParentID), nbrPtr);
						if(nbrTreeParentId == roneID){
							int16 x,y,xprime,yprime;
							nbrPtr = nbrListGetNbr(&nbrList, i);
							int32 nbrOrient = nbrGetOrientation(nbrPtr);
							int32 nbrBear = nbrGetBearing(nbrPtr);

							x = nbrDataGetNbr16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,nbrPtr);
							y = nbrDataGetNbr16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,nbrPtr);

							//if(printNow){rprintf("ID %d TrID %d J%d - Nbr %d: X%d Y%d\n", roneID, nbrDataGetNbr(&(globalRobotListPtr.list[j].ID),nbrPtr), j, nbrGetID(nbrPtr), x,y);}

							xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
							yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
							x = xprime;
							y = yprime + nbrEdgeDis;

							xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
							yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
							//if(printNow){rprintf("Nbr %d: O%d B%d X''%d Y''%d XCOM%d YCOM%d\n",nbrGetID(nbrPtr),nbrOrient,nbrBear,x,y, xprime,yprime);}
							xtot +=  xprime;
							ytot +=  yprime;
							wieght++;
						}
					}
					if(wieght == 0){
						//nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,0);
						//nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,0);
					}else{
						int16 xave = xtot/wieght;
						int16 yave = ytot/wieght;
						nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,xave);
						nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,yave);
						//if(printNow){rprintf("TrID %d XA %d YA %d\n", nbrDataGet(&(globalRobotListPtr.list[j].ID)),xave,yave);}
					}
				}*/
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
						if(printNow){rprintf("X%d, Y%d\n",COM_X, COM_Y);}
					}
					if(abs(COM_Y) > 10){
						if(COM_Y > 0){
							behSetTvRv(&behOutput, NORM_TV/2, NORM_RV);
						}else{
							behSetTvRv(&behOutput, NORM_TV/2, -NORM_RV);
						}

					}else{
						behSetTvRv(&behOutput, NORM_TV, 0);
					}
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


