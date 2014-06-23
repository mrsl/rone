/*
 * MRMCoMestimate.c
 *
 *  Created on: Feb 26, 2014
 *      Author: Wei Zeng
 */
// This code is for the estimation of Center of Mass

// system includes
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// OS includes
#include "roneos.h"
#include "ronelib.h"

// project includes
#include "robotList.h"
#include "nbrDataCoM.h"


/* CONSTANTS *****************************************************/
#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

#define NEIGHBOR_ROUND_PERIOD			550

/********* Constants *********/
#define MRM_ROBOT_COUNT_MAX 			6
#define MRM_TREE_MAX_HOP	 			4
#define MRM_IR_XMIT_COMMS_XMIT_POWER	(IR_COMMS_POWER_MAX / 2)


/* User-defined functions ***************+**************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);

/* global variables ***********************************************/
boolean printNow = FALSE;

// the background task runs all the time.  Put slow stuff here, like compute intensive functions
// or large data transfers, like getting the Yun's map from the robot.


/******* helper functions ***********/

void printTreeArray(BroadcastMessage* tree_array, uint8 manipulationRobotsCounter) {
	cprintf("[");
	int i;
	for (i = 0; i < manipulationRobotsCounter; i++) {
		cprintf("pos: %d, sourceID: %d, senderID: %d, hopNum: %d;\n", i, tree_array[i].msgSourceID.value,
				tree_array[i].senderID, tree_array[i].msgHops.value);
	}
	cprintf("]\n");
}



void backgroundTask(void* parameters) {
	uint8 ones, tenths;
	for (;;) {
		// delay to let other tasks run at same priority
		osTaskDelay(5000);
		systemBatteryVoltageGet2(&ones, &tenths);
	} //Infinite for loop
} //backgroundTask()

// behaviors run every 50ms.  They should be designed to be short, and terminate quickly.
// they are used for robot control.  Watch this space for a simple behavior abstraction
// to appear.
//


void behaviorTask(void* parameters) {

	/******** Variables *********/

	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRound = 0;
	uint8 manipulationRobotsCounter = 0;

	/******** Variables Declared by JJF ********/

	//BroadcastMessage broadcastMessage;
	BroadcastMessage treeArray[MRM_ROBOT_COUNT_MAX]; // changed type
	nbrDataRobotList robotList[MRM_ROBOT_COUNT_MAX];

	/******** Initializations ********/
	radioCommandSetSubnet(2);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	//TODO yo uonly need to call this function once
	irCommsSetXmitPower(MRM_IR_XMIT_COMMS_XMIT_POWER);

	systemPrintStartup();


	// make a robot list to sort all the robots
	robotListCreate(robotList, MRM_ROBOT_COUNT_MAX);  	// generate a robot list


	/* create all the trees, and set them to not be sources */
	for (i = 0; i < MRM_ROBOT_COUNT_MAX; ++i) {
		broadcastMsgCreate(&treeArray[i], MRM_TREE_MAX_HOP);  	// generate a broadcast tree
		//broadcastMsgSetSource(&tree_array[i], FALSE);		// I'm not the course yet myself as the source of the broadcastMessage
	}


//	int i;		// used as loop counter
//	for (i = 1; i < MAX_ROBOT_NUM; i++) {
//		broadcastMsgCreate(&tree_array[i], maxHop)
//	}

	/* add one manipultaion robot for ourself */
	manipulationRobotsCounter = 1;

	/******** Behavior **************/


	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		} //end if host
		else { // infinite loop starts from here

			BroadcastMessage tempTree;
			boolean treeExist = FALSE;
			NbrList nbrList;

			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);
			//cprintf("print %d \n",printNow);

			nbrListCreate(&nbrList);
			uint8 i;
			for (i = 0; i < manipulationRobotsCounter; i++){
				broadcastMsgUpdate(&treeArray[i], &nbrList);
			}



			//nbrListPrintHops(&nbrList, &broadcastMessage, "OMG");

//			uint8 tempSourceID = broadcastMessage.msgSourceID.value;
//			uint8 tempMsgHops = broadcastMessage.msgHops.value;
//			uint8 tempSenderID = broadcastMessage.senderID;
//
//			treeUpdate(&tree_array[0], tempSourceID,
//					tempSenderID, tempMsgHops);	// update my tree, it will be at the top of the array initially


//			// create a data wrapper that contains my broadcastMsg
//			nbrDataBroadcastMsgCreate(&broadcastMsgData, "broadcastMsg");

			for (i = 0; i < nbrListGetSize(&nbrList); i++) {
				nbrPtr = nbrListGetNbr(&nbrList, i);

				if (nbrPtr != NULL) {
					int j;
					uint8 tempSourceID;
					uint8 tempMsgHops;
					uint8 tempSenderID;

					for (j = 0; j < manipulationRobotsCounter; j++) {
						if (treeArray[j].msgSourceID.value == roneID) {
							tempSourceID = broadcastMsgGetSourceIDNbr(&treeArray[j], nbrPtr);
							tempMsgHops = broadcastMsgGetHopsNbr(&treeArray[j], nbrPtr);
							tempSenderID = broadcastMsgGetSenderIDNbr(&treeArray[j], nbrPtr);
							cprintf("In my message, my sourceID is %d, MsgHops is %d, SenderID is %d, tempSourceID is %d, tempMsgHop"
									"s is %d, tempSenderID is %d.\n",
									treeArray[j].msgSourceID.value, treeArray[j].msgHops.value, treeArray[j].senderID, tempSourceID, tempMsgHops, tempSenderID);
							break;
						}
					}


					if ( tempSourceID != 0) {

//						/* Update my broadcastMsg with tempSouceId, tempMsgHops, and tempSenderID */
//						broadcastMessage.senderID = tempSenderID;
//						broadcastMessage.msgSourceID.value = tempSourceID;
//						broadcastMessage.msgHops.value = tempMsgHops;
//						if (tempSourceID == roneID)
//							broadcastMessage.isSource == TRUE;
//						else
//							broadcastMessage.isSource == FALSE;
//
//						broadcastMsgUpdate(&broadcastMessage, &nbrList);



						for (j = 0; j < manipulationRobotsCounter; j++) {
							if (treeArray[j].msgSourceID.value == tempSourceID) {
								broadcastMsgUpdate(&treeArray[j], &nbrList);
								//treeUpdate(&tree_array[j], broadcastMessage.msgSourceID.value, broadcastMessage.senderID, broadcastMessage.msgHops.value);
								treeExist = TRUE;
								break;
							}
						}
						if (!treeExist) {
							cprintf("I am making a tree %d.\n", manipulationRobotsCounter);
							cprintf("after making a treebb, my sourceID is %d, senderID is %d, hopNum is %d.\n", treeArray[manipulationRobotsCounter].msgSourceID.value, treeArray[manipulationRobotsCounter].senderID, treeArray[manipulationRobotsCounter].msgHops.value);
							cprintf("msgUpdate nbrlist's length %d\n", nbrListGetSize(&nbrList));
							cprintf("robot no.%d Num hops: %d\n", treeArray[manipulationRobotsCounter-1].msgSourceID.value, treeArray[manipulationRobotsCounter-1].msgHops.value);
							broadcastMsgUpdate(&treeArray[manipulationRobotsCounter], &nbrList);
							cprintf("after making a tree, my sourceID is %d, senderID is %d, hopNum is %d.\n", treeArray[manipulationRobotsCounter].msgSourceID.value, treeArray[manipulationRobotsCounter].senderID, treeArray[manipulationRobotsCounter].msgHops.value);
							//treeUpdate(&tree_array[robot_counter], broadcastMessage.msgSourceID.value, broadcastMessage.senderID, broadcastMessage.msgHops.value);
							// sort the tree array;
							int loop_counter = manipulationRobotsCounter;
							while(loop_counter > 0) {
								if (treeArray[loop_counter].msgSourceID.value < treeArray[loop_counter-1].msgSourceID.value) {
									tempTree = treeArray[loop_counter];
									treeArray[loop_counter] = treeArray[loop_counter-1];
									treeArray[loop_counter-1] = tempTree;
								}
								loop_counter--;
							}
							manipulationRobotsCounter++;  // we increment robot_counter because we've updated a new tree in array
						} else {
							treeExist = FALSE;
						}
					}

				}
			}
			printTreeArray(&treeArray, manipulationRobotsCounter);


			///////////////////////////////////////////////////////////
			neighborsPutMutex(); // commented
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);

		} // END of else , if the robot is not Host

	} //forever for loop
} //behaviorTask()



/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);
	osTaskCreate(backgroundTask, "background", 1536, NULL,
			BACKGROUND_TASK_PRIORITY); // commented for testing radio message

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;

}
