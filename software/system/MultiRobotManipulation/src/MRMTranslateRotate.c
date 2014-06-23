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
#define TV								40

#define INACTIVE						0
#define TRANSLATE						1
#define ROTATE							2
#define LEADERSTOP						3

#define K							5
#define SPEED						60
#define TV_MAX						100
#define RV_MAX						4000

#define FLOCK_TV					10

#define FLOCK_INTEGRAL_MAX		3000
#define FLOCK_INTEGRAL_DECAY	970
#define FLOCK_INTEGRAL_DECAY2	4

#define K_INTEGRAL_MAX				1000 //400 works, 300 doesn't
#define DECAY_INTEGRAL              70

#define OBJECT_MASS					778
#define K_PI_N						100
#define K_PI_D					    100



#define K_I				            10
#define K_P			            	5000
#define K_D				            75


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
	uint8 i = 0;
	uint16 IRXmitPower = IR_COMMS_POWER_MAX;

	boolean newSensorData;
	int32 tv, rv;

	uint16 leader = 0;
	uint16 mode = INACTIVE;
	uint16 nbrMode = INACTIVE;
	uint16 nbrLeader = 0;
	uint16 wait = 0;
	NbrData msgLeader;
	NbrData msgBearing;
	NbrData msgMode;
	NbrList nbrList;
	Nbr* nbrPtr;
	Nbr* leaderPtr;
	int32 translatetime = 0;
	int32 rotateTime = 0;


	int32 alpha;
	int32 flockAngleIntegral = 0;
	uint32 accelCounter = 0;



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
			translatetime=0;
			rotateTime = 0;
		}
		// Checks Green button to stop behavior
		if (buttonsGet(BUTTON_GREEN)==1)
		{
			leader = 1;
			mode = ROTATE;
			translatetime = 0;
			rotateTime = 0;
		}
		if (buttonsGet(BUTTON_BLUE)==1)
		{
			leader = 1;
			mode = LEADERSTOP;
			translatetime = 0;
			rotateTime = 0;
		}

		uint32 translateNum = 0;
		uint32 rotateNum = 0;
		uint8 foundLeader = 0;
		uint32 interruptNum = 0;
		leaderPtr = NULL;
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
					if(mode == ROTATE){
						translatetime = 0;
					}
					if(mode == TRANSLATE){
						rotateTime = 0;
					}
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
					rotateTime = 0;
				} else if (rotateNum > translateNum && rotateNum > interruptNum){
					mode = ROTATE;
					translatetime = 0;
				} else if (rotateNum == 0 && translateNum == 0 && interruptNum == 0){
					//dont change from previous mode
				} else if (interruptNum > translateNum && interruptNum > rotateNum){
					mode = LEADERSTOP;
					translatetime = 0;
					rotateTime = 0;
				}	else {
					mode= TRANSLATE;
					rotateTime = 0;
				}
			}
		}
		nbrDataSet(&msgLeader, leader);
		nbrDataSet(&msgMode, mode);

		switch(mode){
		case INACTIVE: {
			behOutput = behInactive;
			cprintf("inactive \n");
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			break;
		}
		case LEADERSTOP: {
			behOutput = behInactive;
			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			break;
		}
		case TRANSLATE: {
			translatetime = translatetime +1;

			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			if(leader){
				//Pulsing Green LEDS indicate waiting to translate
				//Circling Green LEDS indicate translation has begun
				if (translatetime<=100){
					behSetTvRv(&behOutput,0,0);
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				}
				else {
					behSetTvRv(&behOutput,TV,0);
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				}
			} else {
				int32 tvgain, error;
				// either push or flock, depending on the experiment
				//				alpha = behFlockAngle(&nbrList);
				//				rvBearingController(&behMove, alpha, 90);
				//				error = behFlockAngleMax(&nbrList)/ 100;
				//				if(abs(error) > 3) {
				//					//behMove.tv = 0;
				//					if(error > 0) {
				//						motorSetPWM(MOTOR_LEFT, -PHOTOTAXIS_PWM);
				//						motorSetPWM(MOTOR_RIGHT, PHOTOTAXIS_PWM);
				//					} else {
				//						motorSetPWM(MOTOR_LEFT, PHOTOTAXIS_PWM);
				//						motorSetPWM(MOTOR_RIGHT, -PHOTOTAXIS_PWM);
				//					}
				//					cprintf("rotate    error % 4d  tv% 4d size% 4d\n",	error, behMove.tv, nbrList.size );
				//				} else {
				////					behMove.tv = FLOCK_TV;
				//					motorSetPWM(MOTOR_LEFT, PHOTOTAXIS_PWM);
				//					motorSetPWM(MOTOR_RIGHT, PHOTOTAXIS_PWM);
				//					cprintf("translate error % 4d  tv% 4d size% 4d\n",	error, behMove.tv, nbrList.size );
				//				}

				alpha = behFlockAngle(&nbrList);

				error = alpha / 100;
				flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY / 1000;
				flockAngleIntegral += (error * error) * (error > 0 ? 1 : -1);
				flockAngleIntegral = bound(flockAngleIntegral, -FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
				tvgain = 100 - (abs(flockAngleIntegral) * 100)/FLOCK_INTEGRAL_MAX;
				//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
				//behMove.rv = flockAngleIntegral;

				int32 rv_I = flockAngleIntegral * K_I/100 ; //WEIGHTED TEST WORKS WITH 2
				int32 rv_P = error * K_P/100; //WEIGHTED TEST WORKS WITH 2

				cprintf("error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",	alpha, flockAngleIntegral, rv_I, rv_P );

				behOutput.rv = (rv_I + rv_P) ;

				//rv = ((100 - (networkSize*100)/4) * rv) / 100;



				behOutput.active = TRUE;

				//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));


				if (translatetime<=100){
					behOutput.tv = 0;
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				}
				else {
					//behOutput.tv = TV;
					behOutput.tv =  2 * FLOCK_TV * tvgain / 100;
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				}
				motorSetBeh(&behOutput);

			}

			break;
		}
		case ROTATE: {
			rotateTime = rotateTime +1;
			if(leader){
				if (rotateTime<=100){
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SNAIL);
					behOutput = behInactive;
				} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					behOutput = behInactive;
				}
			} else {
				//behFlockNormalToLeader(&behOutput,&nbrList,50);
				uint16 speed = 0;
				if (rotateTime <=100){
					tv = 0;
					speed = LED_RATE_SNAIL;
				} else {
					tv = TV;
					speed = LED_RATE_MED;
				}
				if(leaderPtr != NULL){
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, speed);
					behOrbitRange(&behOutput,leaderPtr,tv,50);
				} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, speed);
				}
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


