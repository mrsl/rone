/*
 * test_RotObj.c
 *
 *  Created on: Jun 16, 2014
 *      Author: MaJellins
 */

#include <stdio.h>
#include <stdlib.h>

#include <math.h>

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
#define GUESSCOM		 1

#define PI				3147

#define nbrEdgeDis		250				//hardcoded distance

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput;
	boolean printNow;
	uint32 neighborRound = 0;
	NbrList nbrList;
	int i;
	Nbr* nbrPtr;

	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);

	NbrData guessWeight;
	NbrData guessX_1,guessX_2,guessX_3,guessX_4;
	NbrData guessY_1,guessY_2,guessY_3,guessY_4;

	nbrDataCreate(&(guessWeight), "guessWeight", 8, 0);
	nbrDataCreate32(&guessX_1,&guessX_2,&guessX_3,&guessX_4, "guessX_1", "guessX_2","guessX_3","guessX_4",0);			//in milliradians
	nbrDataCreate32(&guessY_1,&guessY_2,&guessY_3,&guessY_4, "guessY_1", "guessY_2","guessY_3","guessY_4",0);	 //in millimeters

	uint32 x,y, xprime, yprime, nbrOrient,nbrBear, nbrWeight, weightTot, xtot, ytot, xave, yave;
	uint8 roundWeight;

	//uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;
	//GlobalRobotList globalRobotListPtr;
	//globalRobotListCreate(&globalRobotListPtr);

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

			//globalRobotListUpdate(&globalRobotListPtr, &nbrList);

			if(printNow){
			//	globalRobotListPrintAllTree(&globalRobotListPtr, &nbrList);
			}

			/*** READ BUTTONS ***/
			if (buttonsGet(BUTTON_RED)) {
				navigationMode = GUESSCOM;
			}else if (buttonsGet(BUTTON_GREEN)) {
			} else if (buttonsGet(BUTTON_BLUE)) {
			}

			/** STATES MACHINE **/
			switch (navigationMode) {
			case GUESSCOM: {
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			case MODE_IDLE:
			default: {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			}
			if(navigationMode == GUESSCOM){
				roundWeight = 0;
				xtot = 0;
				ytot = 0;
				weightTot = 0;
				for (i = 0; i < nbrListGetSize(&nbrList); i++) {
					nbrPtr = nbrListGetNbr(&nbrList, i);
					nbrOrient = nbrGetOrientation(nbrPtr) + PI;
					nbrBear = nbrGetBearing(nbrPtr) + PI;
					x = nbrDataGetNbr32(&guessX_1,&guessX_2,&guessX_3,&guessX_4,nbrPtr);
					y = nbrDataGetNbr32(&guessY_1,&guessY_2,&guessY_3,&guessY_4,nbrPtr);
					if(printNow){
						cprintf("NBR: B%d O%d X%d Y%d ",nbrOrient,nbrOrient,x,y);
					}
					xprime = x*cosMilliRad(nbrOrient)/1000 - y*sinMilliRad(nbrOrient)/1000;
					yprime = x*sinMilliRad(nbrOrient)/1000 + y*cosMilliRad(nbrOrient)/1000;

					x = xprime;
					y = yprime + nbrEdgeDis;
					if(printNow){
						cprintf("MATH: X'%d Y'%d X''%d Y''%d ",xprime,xprime,x,y);
					}

					xprime = x*cosMilliRad(nbrBear)/1000 - y*sinMilliRad(nbrBear)/1000;
					yprime = x*sinMilliRad(nbrBear)/1000 + y*cosMilliRad(nbrBear)/1000;
					if(printNow){
						cprintf("X'''%d Y'''%d\n",xprime,xprime);
					}
					nbrWeight = nbrDataGetNbr(&guessWeight,nbrPtr);
					weightTot += nbrWeight;
					xtot += nbrWeight * xprime;
					ytot += nbrWeight * yprime;
					roundWeight++;
				}
				xave = xtot / weightTot;
				yave = ytot / weightTot;
				nbrDataSet32(&guessX_1,&guessX_2,&guessX_3,&guessX_4,xave);
				nbrDataSet32(&guessY_1,&guessY_2,&guessY_3,&guessY_4,yave);
				nbrDataSet(&guessWeight,roundWeight);
				if(printNow){
					cprintf("Guess: X %d Y%d Weight %d\n",xave,yave,weightTot);
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


