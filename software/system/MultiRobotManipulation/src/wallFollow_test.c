/*	wallFollow_test.c
 *
 * Author: Mathew Jellins
 * Date: 07/29/2012
 * Summary: This program heads straight until it finds a wall.
 * It then follows along that wall. Lights tell you which wall is being followed.
 * Circling lights means the robot thinks it is 90 degrees do something
 *
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

#define MOVE_STRAIGHT					0
#define AVOIDING_WALLS					1

#define TURNTIME						100


#define LIGHT_VALUE_HOME		        300


/* User-defined functions *****************************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);

/* global variables ***********************************************/
boolean printNow = FALSE;
BroadcastMessage broadcastMessage;

//NbrList Parents, Siblings, nbrListIn, pick_nei;
//uint8 right_nbr_ID, left_nbr_ID;
//NbrNbr klist;


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
		//cprintf("battery voltage: %1d.%1d\n", ones, tenths);
		//systemPrintMemUsage();
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

	boolean newSensorData;
	uint32 timeWallStart;
	uint32 timeStragihtStart;
	int32 tv, rv;

	// NEIGHBOR_MESSAGE_LENGTH creates #define not found errors
	//char nbrMessage[NEIGHBOR_MESSAGE_LENGTH];

	uint32 guideStartTime = 0;
	Beh behOutput;

	/******** Initializations ********/
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	broadcastMsgCreate(&broadcastMessage, 20); 		 //broadcastMessageCreate(&broadcastMessage)
	int startBeh = 0;
	int navigationMode = MOVE_STRAIGHT;

	systemPrintStartup();
	int16 bearing;
	int16 bearGroups[4];
	int16 newRV = 0;
	/******** Behavior **************/
	for (;;)
	{
		//Checks Red button. Doesn't start till it has been pushed
		if (buttonsGet(BUTTON_RED))
		{
			startBeh = 1;
		}
		// Checks Green button to stop behavior
		if (buttonsGet(BUTTON_GREEN))
		{
			startBeh = 0;
		}
		splitBearingGroup(bearGroups);
		cprintf("1 %d 2 %d 3 %d 4 %d\n",bearGroups[0],bearGroups[1],bearGroups[2],bearGroups[3]);
		behOutput = behInactive;
		// if red button pressed Start behavior
		if (startBeh){
			ledsSetPattern(LED_GREEN,LED_PATTERN_CIRCLE,LED_BRIGHTNESS_MED,LED_RATE_MED);
			bearing = irObstaclesGetBearing();
			uint8 bits = irObstaclesGetBits();
						//if wall is sensed
						if(bits != 0){
							bearing = bearing - 3141;

							//if wall is to the right
							if(bearing >= 0){
								newRV = (bearing - 1571)/ 1.5;
								cprintf(" rv = %d\n",newRV);
								behSetRv(&behOutput,newRV);
								//ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								//if wall is normal to robot (within 10 degrees), stop rotating
								if (abs(newRV) < 100){
									behSetTvRv(&behOutput, 0, 0);
									//ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								}
							}//end right wall
							//if wall is to the left
							else{
								newRV = (bearing + 1571) / 1.5;
								cprintf(" rv = %d\n",newRV);
								behSetTvRv(&behOutput, 0, newRV);
								//ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								//if wall is normal to robot, stop rotating
								if (abs(newRV) < 100){
									behSetTvRv(&behOutput, 0, 0);
									//ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								}
							} // end left wall
						} // end if bearing != 0
						//of wall is sensed
						else{
							behSetTvRv(&behOutput, 0, 0);
							//ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
						} //End else no wall
						cprintf("RV: %d active: %d\n",newRV,1);
						/*
			bearing = irObstaclesGetBearing();

			//if wall is sensed
			if(bearing != 0){
				bearing = bearing - 3141;
				//cprintf(" bearing = %d",bearing);

				//if wall is to the right
				if(bearing >= 0){
					newRV = (bearing - 1571)/ 1.5;
					//cprintf(" rv = %d\n",newRV);
					behSetTvRv(&behOutput, 50, newRV);
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					//if wall is apporzimatly 90 degrees
					if (abs(newRV) < 100){
						behSetTvRv(&behOutput, 50, 0);
						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					}
				}//end right wall
				//if wall is to the left
				else{
					newRV = (bearing + 1571) / 1.5;
					//cprintf(" rv = %d\n",newRV);
					behSetTvRv(&behOutput, 50, newRV);
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					//if wall is apporzimatly 90 degrees
					if (abs(newRV) < 100){
						behSetTvRv(&behOutput, 50, 0);
						ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					}
				} // end left wall
			} // end if bearing != 0
			//of wall is sensed
			else{
				behSetTvRv(&behOutput, 50, 0);
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			} //End else no wall*/
		} //End behavior start
		else{
			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		} // end else no behavior

		motorSetBeh(&behOutput);
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

