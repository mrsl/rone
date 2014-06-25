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

#define nbrEdgeDis		5000				//hardcoded distance

typedef struct posCOM {
	NbrData X_HH;
	NbrData X_HL;
	NbrData X_LH;
	NbrData X_LL;
	NbrData Y_HH;
	NbrData Y_HL;
	NbrData Y_LH;
	NbrData Y_LL;
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

	posCOM treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE];
	for(i = 0; i <GLOBAL_ROBOTLIST_MAX_SIZE; i++){
		nbrDataCreate32(&treeGuessCOM[i].X_HH,&treeGuessCOM[i].X_HL,&treeGuessCOM[i].X_LH,&treeGuessCOM[i].X_LL,"X_HH", "X_HL","X_LH", "X_LL", 0);
		nbrDataCreate32(&treeGuessCOM[i].Y_HH,&treeGuessCOM[i].Y_HL,&treeGuessCOM[i].Y_LH,&treeGuessCOM[i].Y_LL,"Y_HH", "Y_HL","Y_LH", "Y_LL", 0);
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
				for (j = 0; j < globalRobotListPtr.size; j++) {
					int32 xtot = 0;
					int32 ytot = 0;
					uint8 wieght = 0;

					for (i = 0; i < nbrList.size; i++){
						nbrPtr = nbrList.nbrs[i];
						uint8 nbrTreeParentId = nbrDataGetNbr(&(globalRobotListPtr.list[j].ParentID), nbrPtr);
						if(nbrTreeParentId == roneID){
							int32 x,y,xprime,yprime;
							nbrPtr = nbrListGetNbr(&nbrList, i);
							int32 nbrOrient = nbrGetOrientation(nbrPtr);
							int32 nbrBear = nbrGetBearing(nbrPtr);

							x = nbrDataGetNbr32(&treeGuessCOM[j].X_HH,&treeGuessCOM[j].X_HL,&treeGuessCOM[j].X_LH,&treeGuessCOM[j].X_LL,nbrPtr);
							y = nbrDataGetNbr32(&treeGuessCOM[j].Y_HH,&treeGuessCOM[j].Y_HL,&treeGuessCOM[j].Y_LH,&treeGuessCOM[j].Y_LL,nbrPtr);
							//if(printNow){rprintf("Nbr %d: X%d Y%d\n",nbrGetID(nbrPtr), x,y);}

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
						nbrDataSet32(&treeGuessCOM[j].X_HH,&treeGuessCOM[j].X_HL,&treeGuessCOM[j].X_LH,&treeGuessCOM[j].X_LL,0);
						nbrDataSet32(&treeGuessCOM[j].Y_HH,&treeGuessCOM[j].Y_HL,&treeGuessCOM[j].Y_LH,&treeGuessCOM[j].Y_LL,0);
					}else{
						int32 xave = xtot/wieght;
						int32 yave = ytot/wieght;
						nbrDataSet32(&treeGuessCOM[j].X_HH,&treeGuessCOM[j].X_HL,&treeGuessCOM[j].X_LH,&treeGuessCOM[j].X_LL,xave);
						nbrDataSet32(&treeGuessCOM[j].Y_HH,&treeGuessCOM[j].Y_HL,&treeGuessCOM[j].Y_LH,&treeGuessCOM[j].Y_LL,yave);
						if(printNow){rprintf("TrID %d XA1 %d YA1 %d XA2 %d YA2 %d\n", nbrDataGetNbr(&(globalRobotListPtr.list[j].ID), nbrPtr),xave,yave,
								nbrDataGet32(&treeGuessCOM[j].X_HH,&treeGuessCOM[j].X_HL,&treeGuessCOM[j].X_LH,&treeGuessCOM[j].X_LL),
								nbrDataGet32(&treeGuessCOM[j].Y_HH,&treeGuessCOM[j].Y_HL,&treeGuessCOM[j].Y_LH,&treeGuessCOM[j].Y_LL));}
					}
					//if(printNow){rprintf("TrID %d XT %d, YT %d\n",nbrDataGet(&(globalRobotListPtr.list[j].ID)),
					//		nbrDataGet32(&treeGuessCOM[j].X_HH,&treeGuessCOM[j].X_HL,&treeGuessCOM[j].X_LH,&treeGuessCOM[j].X_LL),
					//		nbrDataGet32(&treeGuessCOM[j].Y_HH,&treeGuessCOM[j].Y_HL,&treeGuessCOM[j].Y_LH,&treeGuessCOM[j].Y_LL));}
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


