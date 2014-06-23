/*
 * CentroidEstimateTreeBasedMethod.c
 *
 *  Created on: May 10, 2014
 *      Author: Golnaz
 */


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "roneos.h"
#include "ronelib.h"


/* CONSTANTS *****************************************************/
#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

#define NEIGHBOR_ROUND_PERIOD			500

#define MULTI_TREE_MAX					10

/* User-defined functions *****************************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);

/* global variables ***********************************************/
boolean printNow = FALSE;
BroadcastMessage broadcastMessage[MULTI_TREE_MAX];


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
	uint16 IRXmitPower = IR_COMMS_POWER_MAX;

	int32 tv, rv;
	int32 i;

	NbrList nbrList;
	Nbr* nbrPtr;
	Nbr* leaderPtr;

	GlobalRobotList robotList;


	/******** Initializations ********/
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	for (i = 0; i < MULTI_TREE_MAX; ++i) {
		broadcastMsgCreate(&broadcastMessage[i], 4);
	}
	globalRobotListCreate(&robotList);

	systemPrintStartup();

	/******** Behavior **************/
	for (;;) {
		neighborsGetMutex();
		printNow = neighborsNewRoundCheck(&neighborRound);
		cprintf("print %d \n",printNow);
		irCommsSetXmitPower(IRXmitPower);

		nbrListCreate(&nbrList);
		for (i = 0; i < MULTI_TREE_MAX; ++i) {
			broadcastMsgUpdate(&broadcastMessage[i], &nbrList);
		}
		globalRobotListUpdate(&robotList, &nbrList);
		if (printNow) globalRobotListPrint(&robotList);

		int8 listIdx = globalRobotListGetIndex(&robotList, roneID);
		if (listIdx >= 0) {
			// we have the position of our ID on the list.  Use the broadcast message slot for our communications
			//TODO: broadcastM
		}
		neighborsPutMutex(); // commented
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}


int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);
	osTaskCreate(backgroundTask, "background", 1536, 0,
			BACKGROUND_TASK_PRIORITY); // commented for testing radio message

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;

}

