/*
 * test_RV_Gripper.c
 *
 *  Created on: Jun 10, 2014
 *      Author: MaJellins
 */


/*
/*
 * test_bumpSkirtWallAngle.c
 *
 *     Created on: 07/30/2012
 *         Author: Mathew Jellins, Zack
 *        Summary: This code calculates a tree based on the weight.
 *        Weight is determined by how close walls are, and a node can be excluded if a wall is to close.
 *        A path can be calculated to the source from any node.
 */
#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD			300
#define MOTION_TV_MIN  					0
#define MOTION_TV_DEFAULT				40
#define MOTION_TV_MAX  					120
#define MOTION_TV_STEP					20
#define BUMP_RELECT_DISTANCE			15
#define SAFETY_CHECK_TIME				2
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50
#define FLOCK_RV_GAIN					150
#define REALESE_WAIT_TIME				2000
#define STRAIGHT_TIME					5000
#define TURN_TIME						1000

#define GRIPPER_UPPER_BOUND		180
#define GRIPPER_LOWER_BOUND		0
#define GRIPPER_INCREMENT		10
#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50
#define TV_MIN							15

const uint8 gHi = GRIPPER_UPPER_BOUND;
const uint8 gLow = GRIPPER_LOWER_BOUND;
const uint8 gInc = GRIPPER_INCREMENT;


#define MODE_IDLE		 0
#define ROT_LEFT		 1
#define ROT_RIGHT		 2




#define CNTCLK			1
#define CLKWISE			2
#define REST			0
#define ATTEMPTING		3

void navigationLEDsSet(uint8 redCount, uint8 greenCount, uint8 blueCount, uint8 brightness) {
	uint8 redBIN;
	uint8 greenBIN;
	uint8 blueBIN;
	switch (redCount){
		case 0: {
			redBIN = 0;
			break;
		}
		case 1: {
			redBIN = 1;
			break;
		}
		case 2: {
			redBIN = 3;
			break;
		}
		case 3: {
			redBIN = 7;
			break;
		}
		case 4: {
			redBIN = 15;
			break;
		}
		case 5: {
			redBIN = 31;
			break;
		}
	}
	switch (greenCount){
		case 0: {
			greenBIN = 0;
			break;
		}
		case 1: {
			greenBIN = 1;
			break;
		}
		case 2: {
			greenBIN = 3;
			break;
		}
		case 3: {
			greenBIN = 7;
			break;
		}
		case 4: {
			greenBIN = 15;
			break;
		}
		case 5: {
			greenBIN = 31;
			break;
		}
	}
	switch (blueCount){
		case 0: {
			blueBIN = 0;
			break;
		}
		case 1: {
			blueBIN = 1;
			break;
		}
		case 2: {
			blueBIN = 3;
			break;
		}
		case 3: {
			blueBIN = 7;
			break;
		}
		case 4: {
			blueBIN = 15;
			break;
		}
		case 5: {
			blueBIN = 31;
			break;
		}
	}
	ledsSetBinary(redBIN,greenBIN,blueBIN);
}

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput;
	uint8 i;
	boolean printNow, WAITING;
	uint32 neighborRound = 0;
	int RV;


	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(2);


	//uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;

	boolean bounceGreen = 0;
	boolean bounceBlue = 0;
	boolean bounceRed  = 0;
	uint8 redCount = 0;
	uint8 blueCount = 0;

	//GRIPPER INIT
	gripperBoardInit();
	uint8 gripPos = REST;
	gripperGripRelax();
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


			/*** READ BUTTONS ***/
			if (buttonsGet(BUTTON_RED)) {
				navigationMode = ROT_LEFT;
				if(bounceRed){
					redCount++;
					if(redCount > 5){
						redCount = 0;
					}
				}
				bounceRed = 0;
			}
			if (buttonsGet(BUTTON_GREEN)) {
				gripPos = 0;
				navigationMode = MODE_IDLE;
			}
			if (buttonsGet(BUTTON_BLUE)) {
				navigationMode = ROT_RIGHT;
				if(bounceBlue){
					blueCount++;
					cprintf("1\n");
					if(blueCount > 5){
						blueCount = 0;
					}
				}
				bounceBlue = 0;
			}

			if (!buttonsGet(BUTTON_RED)) {
				bounceRed = 1;
			}
			if (!buttonsGet(BUTTON_GREEN)) {
				bounceGreen = 1;
			}
			if (!buttonsGet(BUTTON_BLUE)) {
				bounceBlue = 1;
			}

			/** STATES MACHINE **/
			switch (navigationMode) {
			case MODE_IDLE: {
				//Start Menu
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				behOutput = behInactive;
				if(gripPos != ATTEMPTING){
					gripperGripUntilGripped();
					gripPos = ATTEMPTING;
				}
				cprintf("Idle");
				break;
			}
			case ROT_LEFT: {
				//Choose lowest robot amongst themselves to be controlled by remote
				navigationLEDsSet(redCount,0,blueCount,  0);
				RV = redCount*300;
				behSetTvRv(&behOutput, 0, RV);
				cprintf("Left RV %d ",RV);

				break;
			}
			case ROT_RIGHT: {
				//Robot to be controled by remote
				navigationLEDsSet(redCount,0,blueCount,  0);
				RV = blueCount*300;
				behSetTvRv(&behOutput, 0, -RV);
				cprintf("Right RV %d ",RV);
				break;
			}
			}

			cprintf("BUMP %d A X %d A Y %d A Z %d G X %d G Y %d G Z %d\n", bumpSensorsGetBearing(),
					accelerometerGetValue(ACCELEROMETER_X), accelerometerGetValue(ACCELEROMETER_Y), accelerometerGetValue(ACCELEROMETER_Z),
					gyroGetValue(GYRO_X_AXIS), gyroGetValue(GYRO_Y_AXIS), gyroGetValue(GYRO_Z_AXIS));
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


