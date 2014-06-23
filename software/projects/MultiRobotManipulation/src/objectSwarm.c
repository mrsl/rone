/*
 * objectSwarm.c
 *
 *     Created on: 6/4/2014
 *         Author: Mathew Jellins
 *        Summary: Finds a robot through follow the leader and swarms it
 */
#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD			500
#define MOTION_TV_MIN  					0
#define MOTION_TV_DEFAULT				40
#define MOTION_TV_MAX  					120
#define MOTION_TV_STEP					20
#define BUMP_RELECT_DISTANCE			15
#define MAJOR_AXIS						25
#define MAJOR_AXIS_BITS					2
#define MINOR_AXIS						10
#define MINOR_AXIS_BITS					175
#define SWITCH_DELAY					3
#define SAFETY_CHECK_TIME				2

#define MODE_IDLE		 0
#define FOLLOWER		 1
#define LEADER			 2
#define OBJECT			 3
#define OBJECT_FOUND	 4
#define GRIPPED			 5



void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput;
	boolean printNow = FALSE;
	uint8 i;
	uint32 neighborRound = 0;
	NbrList nbrList;
	Nbr* nbrPtr;
	Nbr* bestParentPtr;
	uint32 destination;
	uint8 objectID;
	uint8 bumpSense;


	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(2);

	NbrData msgNbrType;


	nbrDataCreate(&msgNbrType, "type", 3, 0);

	uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;

	int16 newRV = 0;

	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		}//end if host
		else{
			/*** INIT STUFF ***/
			behOutput = behInactive;
			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);

			irCommsSetXmitPower(IRXmitPower);

			nbrListCreate(&nbrList);


			broadcastMsgUpdate(&broadcastMessage, &nbrList);


			/*** READ BUTTONS ***/
			if (buttonsGet(BUTTON_RED)) {
				navigationMode = FOLLOWER;
			}else if (buttonsGet(BUTTON_GREEN)) {
				navigationMode = OBJECT;
			} else if (buttonsGet(BUTTON_BLUE)) {
			}

			switch (navigationMode) {
			case MODE_IDLE: {
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				nbrDataSet(&msgNbrType, 0);
				break;
			}
			case FOLLOWER: {
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, 1);
				break;
			}
			case LEADER: {
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, 2);
				break;
			}
			case OBJECT: {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, 3);
				break;
			}
			case OBJECT_FOUND: {
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, 4);
				break;
			}
			case GRIPPED: {
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, 5);
				break;
			}
			default:{
			}

			}//end switch

			/** Elect Leader **/
			if (navigationMode == FOLLOWER || navigationMode == LEADER){
				uint8 currID = 0;
				uint8 lowestID = 0;
				uint8 parentType;
				bestParentPtr = NULL;
				//For all parents in list
				for (i = 0; i < nbrList.size; i++){
					//Search for lowest ID
					nbrPtr = nbrList.nbrs[i];
					currID = nbrGetID(nbrPtr);
					if(lowestID == 0){
						lowestID = currID;
						bestParentPtr = nbrPtr;
					} else if(currID < lowestID){
						lowestID = currID;
						bestParentPtr = nbrPtr;
					}
					//Search for object
					parentType = nbrDataGetNbr(&msgNbrType, nbrPtr);
					if(parentType == OBJECT || parentType == GRIPPED){
						navigationMode = OBJECT_FOUND;
						objectID = currID;
					}
				}
				//if (printNow){rprintf("c%d l%d\n",currID,lowestID);}
				if(navigationMode != OBJECT_FOUND){
					if(roneID < lowestID || lowestID == 0){
						navigationMode = LEADER;
						destination = 0;
					} else{
						navigationMode = FOLLOWER;
						destination = nbrGetBearing(bestParentPtr);
					}
				}
			}

			/****Search for OBJECT****/
			if (navigationMode == OBJECT_FOUND ){
				//For all parents in list
				objectID = 0;
				for (i = 0; i < nbrList.size; i++){
					//Search for object
					if(nbrDataGetNbr(&msgNbrType, nbrPtr) == OBJECT){
						objectID = nbrGetID(nbrPtr);
					}
				}
				if(objectID == 0){
					navigationMode = FOLLOWER;
				}else{
					destination = nbrGetBearing(bestParentPtr);
					if(destination <=0){
						destination = destination + 1571;
					}else {
						destination = destination - 1571;
					}
				}
				bumpSense = bumpSensorsWhichHit();
				if(bumpSense == 0){
					navigationMode = GRIPPED;
				}
			}

			/**Motion**/
			if(navigationMode == MODE_IDLE || navigationMode ==OBJECT){
				behOutput = behInactive;
			}else{
				//if leader is to the right
				if(abs(destination) <= 750){
					behSetTvRv(&behOutput, 50, 0);
				} else if(destination >= 0){
					newRV = (destination)/ 1.5;
				}
				else{ //if leader is to the left
					newRV = (destination) / 1.5;
				}
				behSetTvRv(&behOutput, 50, newRV);
			}
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
