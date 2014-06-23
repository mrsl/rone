/*
 * guideMotionTest.c
 *
 * Author: Lauren Schmidt
 * Date: August 28, 2013
 *
 * Implements guide motion for multi-robot manipulation of an object.
 * Robots either translate the object or rotate the object.
 * The leader for translation is selected by pressing the red button of a robot
 * and rotation mode is selected by pressing the blue button of a robot.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "roneos.h"
#include "ronelib.h"

/* CONSTANTS *****************************************************/
#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

#define NEIGHBOR_ROUND_PERIOD			500
#define MOTION_TV  						45
#define PARENT_ID_UNDECIDED				MAX_PARENT_ID
#define BUMP_RELECT_DISTANCE			35

#define TURNTIME						100

#define INACTIVE						0
#define TRANSLATE						1
#define ROTATE							2
#define LEADERSTOP						3

#define LIGHT_VALUE_HOME		        300


/* User-defined functions *****************************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);

/* global variables ***********************************************/
boolean printNow = FALSE;
BroadcastMessage broadcastMessage;


// the background task runs all the time.  Put slow stuff here, like compute intensive functions
// or large data transfers, like getting the Yun's map from the robot.
void backgroundTask(void* parameters)
{
	uint8 ones, tenths;
	for (;;)
	{
		// delay to let other tasks run at same priority
		osTaskDelay(5000);
		systemBatteryVoltageGet2(&ones, &tenths);
	}//Infinite for loop
}//backgroundTask()


// behaviors run every 50ms.  They should be designed to be short, and terminate quickly.
// they are used for robot control.  Watch this space for a simple behavior abstraction
// to appear.
//
void behaviorTask(void* parameters)
{

	/******** Variables *********/
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound = 0;
	uint32 alpha = 0;
	uint8 i = 0;
	uint16 IRXmitPower = IR_COMMS_POWER_MAX;

	boolean newSensorData;
	int32 tv, rv;

	uint16 leader = 0;
	uint16 mode = INACTIVE;
	uint16 nbrMode = INACTIVE;
	uint16 nbrLeader = 0;
	uint16 wait = 0;
	uint32 rotateNum = 0;
	uint32 translateNum = 0;
	NbrData msgLeader;
	NbrData msgBearing;
	NbrData msgMode;
	NbrList nbrList;
	Nbr* nbrPtr;
	Nbr* leaderPtr;

	nbrDataCreate(&msgLeader, "leader", 1, 0);
	nbrDataCreate(&msgBearing, "bearing", 7, 0);
	nbrDataCreate(&msgMode, "mode", 2, 0);

	Beh behOutput;

	/******** Initializations ********/
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();

	/******** Behavior **************/
	for (;;)
	{

		behOutput = behInactive;
		neighborsGetMutex();
		printNow = neighborsNewRoundCheck(&neighborRound);
		cprintf("print %d \n",printNow);
		irCommsSetXmitPower(IRXmitPower);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);

		//Check Buttons, won't start until leader selected
		if (buttonsGet(BUTTON_RED)==1)
		{
			leader = 1;
			mode = TRANSLATE;
		}
		// Checks Green button to stop behavior
		if (buttonsGet(BUTTON_GREEN)==1)
		{
			leader = 1;
			mode = ROTATE;
		}
		if (buttonsGet(BUTTON_BLUE)==1)
		{
			leader = 1;
			mode = LEADERSTOP;
		}

		translateNum = 0;
		rotateNum = 0;
		uint8 foundLeader = 0;
		uint32 interruptNum = 0;
		if (leader ==1){
			//do nothing
		} else {
			//check for mode change
			for (i = 0; i < nbrList.size; ++i){
				nbrPtr = nbrList.nbrs[i];
				nbrMode = nbrDataGetNbr(&msgMode, nbrPtr);
				nbrLeader = nbrDataGetNbr(&msgLeader, nbrPtr);
				if (nbrLeader == 1){
					foundLeader = 1;
					mode = nbrMode;
					leaderPtr = nbrPtr;
				} else {
					if (nbrMode != 0){
						if(nbrMode == 1){
							translateNum = translateNum +1;
						} else if (nbrMode == 2){
							rotateNum = rotateNum + 1;
						} else if (nbrMode == 3){
							interruptNum = interruptNum + 1;
						}
					}
				}
			}
			if (foundLeader != 1){
				if(translateNum > rotateNum && translateNum > interruptNum){
					mode = TRANSLATE;
				} else if (rotateNum > translateNum && rotateNum > interruptNum){
					mode = ROTATE;
				} else if (rotateNum == 0 && translateNum == 0 && interruptNum == 0){
					//dont change from previous mode
				} else if (interruptNum > translateNum && interruptNum > rotateNum){
					mode = LEADERSTOP;
				}	else {
					mode= TRANSLATE;
				}
			}
		}
		nbrDataSet(&msgLeader, leader);
		nbrDataSet(&msgMode, mode);
		if(printNow){
			cprintf("found? %d leader %d, mode %d \n",foundLeader,leader, mode);
		}

		switch(mode){
		case INACTIVE: {
			behOutput = behInactive;
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			break;
		}
		case LEADERSTOP: {
			behOutput = behInactive;
			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			break;
		}
		case TRANSLATE: {
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			if(leader){
				//move forward
				behSetTv(&behOutput,50);
			} else {
				alpha = behFlockAngle(&nbrList);
				if (abs(alpha) < 95){
					wait = wait + 1;
					behFlock(&behOutput, &nbrList, 50);
					//if (wait > 10) {
					//	behFlock(&behOutput, &nbrList, 50);
					//} else {
					//	behFlock(&behOutput, &nbrList, 0);
					//}
				} else {
					wait = 0;
					behFlock(&behOutput, &nbrList, 50);
				}
			}
			break;
		}
		case ROTATE: {
			ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			if(leader){
				behOutput = behInactive;
			} else {
				behFlockNormalToLeader(&behOutput,&nbrList,50);
				//behOrbit(&behOutput,&nbrList,50);
			}
			break;
		}
		}//end mode switch
		motorSetBeh(&behOutput);
		neighborsPutMutex();
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}//forever for loop
}//behaviorTask()


/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);
	osTaskCreate(backgroundTask, "background", 1536, NULL, BACKGROUND_TASK_PRIORITY);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}


