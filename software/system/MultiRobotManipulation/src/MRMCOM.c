/*
 * MRMObstacleAvoidance.c
 *
 *  Created on: Aug 28, 2013
 *      Author: mrsl
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
#define MOTION_TV  						45
#define PARENT_ID_UNDECIDED				MAX_PARENT_ID
#define BUMP_RELECT_DISTANCE			35

#define TURNTIME						100
#define TV								30

#define MODE_INACTIVE						0
#define MODE_TRANSLATE						1
#define MODE_ROTATE							2
#define MODE_LEADERSTOP						3
#define MODE_COM							4

#define K						     	5
#define SPEED					    	60
#define TV_MAX					    	100
#define RV_MAX					    	4000

#define TV_GAIN				        	4

#define FLOCK_TV					   10

#define FLOCK_INTEGRAL_MAX		       4000
#define FLOCK_INTEGRAL_DECAY	       970
#define FLOCK_INTEGRAL_DECAY2	     4

#define K_INTEGRAL_MAX				1000 //400 works, 300 doesn't
#define DECAY_INTEGRAL              70

#define OBJECT_MASS					778
#define K_PI_N						100
#define K_PI_D					    100

#define K_I				            20
#define K_P			            	50
#define K_D				            75
#define DELAY_TRANSLATE				200
#define DELAY_ROTATE				400

#define LEADER_POS_SCALER			10
#define LEADER_POS_BOUND			(127 * LEADER_POS_SCALER )

/* User-defined functions *****************************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);

/* global variables ***********************************************/
boolean printNow = FALSE;
BroadcastMessage broadcastMessage;
BroadcastMsgData broadcastData_Mode;
BroadcastMsgData broadcastData_DesiredHeading;

// the background task runs all the time.  Put slow stuff here, like compute intensive functions
// or large data transfers, like getting the Yun's map from the robot.
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
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound = 0;
	uint8 i = 0;
	uint16 IRXmitPower = IR_COMMS_POWER_MAX;

	boolean newSensorData;
	int32 tv, rv;

	uint16 leader = 0;
	uint16 mode = MODE_INACTIVE;
	uint16 nbrMode = MODE_INACTIVE;
	uint16 nbrLeader = 0;
	uint16 wait = 0;

	NbrData msgLeaderPosXR;
	NbrData msgLeaderPosYR;

	NbrData msgLeaderPosXS;
	NbrData msgLeaderPosYS;

	Nbr* nbrPtr;
	Nbr* leaderPtr;
	int32 translatetime = 0;
	int32 rotateTime = 0;
	NbrList nbrList;

	int32 nbrBearing = 0;
	int32 nbrDist = 0;
	int32 alpha;
	int32 flockAngleIntegral = 0;
	int32 flockAngleIntegral2 = 0;

	uint32 accelCounter = 0;
	int32 leaderPosY = 0;
	int32 leaderPosX = 0;
	Beh behOutput;

	/******** Initializations ********/
	radioCommandSetSubnet(1);
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	nbrDataCreate(&msgLeaderPosXR, "positionx", 8, 0);
	nbrDataCreate(&msgLeaderPosYR, "positiony", 8, 0);

	nbrDataCreate(&msgLeaderPosXS, "positionx", 8, 0);
	nbrDataCreate(&msgLeaderPosYS, "positiony", 8, 0);

	broadcastMsgCreate(&broadcastMessage, 20);
	broadcastMsgDataCreate(&broadcastData_Mode, &broadcastMessage, "mode",
			MODE_INACTIVE);
	broadcastMsgDataCreate(&broadcastData_DesiredHeading, &broadcastMessage,
			"heading", 0);
	uint32 senderID_seen = 0;

	systemPrintStartup();
	uint8 val = 0;
	int32 t = 0;
	int16 leaderBearing = 0;
	uint32 leaderDist = 0;
	int32 l1 = 0;
	int32 l2 = 0;
	/******** Behavior **************/
	for (;;) {
		behOutput = behInactive;
		neighborsGetMutex();
		printNow = neighborsNewRoundCheck(&neighborRound);
		//cprintf("print %d \n",printNow);
		//TODO yo uonly need to call this function once
		irCommsSetXmitPower(IRXmitPower);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);
		if (broadcastMessage.active == TRUE) {
			if (broadcastMsgIsSource(&broadcastMessage)) {
				nbrDataSet(&msgLeaderPosXS, 0);
				nbrDataSet(&msgLeaderPosYS, 0);
				//nbrDataSet(&msgLeaderPosXR, 0);
				//nbrDataSet(&msgLeaderPosYR, 0);
				if (printNow) {
					cprintf("x: %d y: %d dist: %d bear: %d\n", 0, 0, 0, 0);
				}

			} else

			{
				int32 senderID = broadcastMsgGetSenderID(&broadcastMessage);

				//behFlockNormalToLeader(&behOutput, &nbrList, nbrPtr,tv);
				for (t = 0; t < nbrList.size; ++t) {
					nbrPtr = nbrList.nbrs[t];
					if (nbrPtr->ID == senderID) {
						senderID_seen = 1;
						nbrBearing = nbrGetBearing(nbrPtr);
						nbrDist = nbrGetRange(nbrPtr);

						if (printNow) {
							cprintf("dist: %d bear: %d\n", nbrDist, nbrBearing);
							//cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);

						}
						l1 = ((int8) nbrDataGet(nbrPtr))	* LEADER_POS_SCALER;
						l2 = ((int8) nbrDataGet(&msgLeaderPosYR))
								* LEADER_POS_SCALER;

						leaderPosX = l1
								+ (int32)(
										nbrDist * cosMilliRad(nbrBearing)
												/ MILLIRAD_TRIG_SCALER);
						leaderPosY = l2
								+ (int32)(
										nbrDist * sinMilliRad(nbrBearing)
												/ MILLIRAD_TRIG_SCALER);
						leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
						leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);

						nbrDataSet(&msgLeaderPosXS,
								(int8)(leaderPosX / LEADER_POS_SCALER));
						nbrDataSet(&msgLeaderPosYS,
								(int8)(leaderPosY / LEADER_POS_SCALER));
						int32 dist = leaderPosX * leaderPosX
								+ leaderPosY * leaderPosY;
						leaderDist = sqrtInt((uint32) dist);
						leaderBearing = atan2MilliRad(leaderPosY, leaderPosX);

						/*
						 * Problems:
						 * 	1- leader bearing not normalized
						 * 	2- distance never changes
						 */
						leaderBearing = normalizeAngleMilliRad2(leaderBearing);

					}
				}

				/*if (printNow) {
				 cprintf("x: %d y: %d dist: %d bear: %d\n", leaderPosX,
				 leaderPosY, leaderDist, leaderBearing);
				 //cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);
				 cprintf("posx: %d posy: %d \n", l1, l2);

				 }*/

			}
		}
		//Check Buttons, won't start until leader selected
		if(buttonsGet(BUTTON_RED)&&buttonsGet(BUTTON_GREEN)){
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_COM);
			translatetime = 0;
			rotateTime = 0;
			val = MODE_COM;
		}else if (buttonsGet(BUTTON_RED)) {
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_TRANSLATE);
			val = MODE_TRANSLATE;
			translatetime = 0;
			rotateTime = 0;
		} else if (buttonsGet(BUTTON_GREEN)) {
			// Checks Green button to stop behavior
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_ROTATE);
			translatetime = 0;
			rotateTime = 0;
			val = MODE_ROTATE;

		} else if (buttonsGet(BUTTON_BLUE)) {
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_LEADERSTOP);
			translatetime = 0;
			rotateTime = 0;
			val = MODE_LEADERSTOP;

		}

		if (printNow)
			cprintf("hops, mode = %d,%d\n",
					broadcastMsgGetHops(&broadcastMessage),
					broadcastMsgDataGet(&broadcastData_Mode));

		//		uint32 translateNum = 0;
		//		uint32 rotateNum = 0;
		//		uint8 foundLeader = 0;
		//		uint32 interruptNum = 0;
		//		leaderPtr = NULL;
		//		if (broadcastMsgIsSource(&broadcastMessage)){
		//			//do nothing
		//		} else {
		//			//check for mode change
		//			for (i = 0; i < nbrList.size; ++i){
		//				nbrPtr = nbrList.nbrs[i];
		//				nbrMode = nbrDataGetNbr(&msgMode, nbrPtr);
		//				nbrLeader = nbrDataGetNbr(&msgLeader, nbrPtr);
		//				if (nbrLeader == 1){
		//					foundLeader = 1;
		//					mode = nbrMode;
		//					leaderPtr = nbrPtr;
		//					if(mode == MODE_ROTATE){
		//						translatetime = 0;
		//					}
		//					if(mode == MODE_TRANSLATE){
		//						rotateTime = 0;
		//					}
		//				} else {
		//					if (nbrMode != 0){
		//						if(nbrMode == 1){
		//							translateNum = translateNum +1;
		//						} else if (nbrMode == 2){
		//							rotateNum = rotateNum + 1;
		//						} else if (nbrMode == 3){
		//							interruptNum = interruptNum + 1;
		//						}
		//					}
		//				}
		//			}
		//			if (foundLeader != 1){
		//				if(translateNum > rotateNum && translateNum > interruptNum){
		//					mode = MODE_TRANSLATE;
		//					rotateTime = 0;
		//				} else if (rotateNum > translateNum && rotateNum > interruptNum){
		//					mode = MODE_ROTATE;
		//					translatetime = 0;
		//				} else if (rotateNum == 0 && translateNum == 0 && interruptNum == 0){
		//					//dont change from previous mode
		//				} else if (interruptNum > translateNum && interruptNum > rotateNum){
		//					mode = MODE_LEADERSTOP;
		//					translatetime = 0;
		//					rotateTime = 0;
		//				}	else {
		//					mode= MODE_TRANSLATE;
		//					rotateTime = 0;
		//				}
		//			}
		//		}
		if (printNow) {
			cprintf("sender: %d\n", broadcastMsgGetSenderID(&broadcastMessage));
		} else {
			val = broadcastMsgDataGet(&broadcastData_Mode);
			//cprintf("val: %d\n", val);
			broadcastMsgDataSet(&broadcastData_Mode, val);
		}
		switch (val) {
		case MODE_INACTIVE: {
			behOutput = behInactive;
			//cprintf("inactive \n");
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED,
					LED_RATE_MED);
			break;
		}
		case MODE_LEADERSTOP: {
			behOutput = behInactive;
			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED,
					LED_RATE_MED);
			break;
		}
		case MODE_TRANSLATE: {
			translatetime = translatetime + 1;

			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED,
					LED_RATE_MED);
			if (broadcastMsgIsSource(&broadcastMessage)) {
				//Pulsing Green LEDS indicate waiting to translate
				//Circling Green LEDS indicate translation has begun
				if (translatetime <= DELAY_TRANSLATE) { // set the time delay before start translating
					behSetTvRv(&behOutput, 0, 0);
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
				} else {

					// TODO : add a timer for the duration of translation : if (translatetime <= TRANSLATE_ PERIOD)
					behSetTvRv(&behOutput, TV, 0);
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
					// TODO : else {					behSetTvRv(&behOutput, TV, 0);
					//  					// mode = MODE_INACTIVE    }




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

				alpha = behFlockAngle(&nbrList); // consensus, be aligned with the leader
				// start PI controller
				error = alpha / 100;
				flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY	/ 1000;
				flockAngleIntegral += (error * error) * (error > 0 ? 1 : -1);
				flockAngleIntegral = bound(flockAngleIntegral,
						-FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
				tvgain = 100 - (abs(flockAngleIntegral) * 100) / FLOCK_INTEGRAL_MAX;
				//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
				//behMove.rv = flockAngleIntegral;

				int32 rv_I = flockAngleIntegral * K_I / 100; //WEIGHTED TEST WORKS WITH 2
				int32 rv_P = error * K_P / 100; //WEIGHTED TEST WORKS WITH 2

				if (printNow) {
					cprintf(
							"error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",
							alpha, flockAngleIntegral, rv_I, rv_P);
				}

				behOutput.rv = (rv_I + rv_P);

				//rv = ((100 - (networkSize*100)/4) * rv) / 100;

				behOutput.active = TRUE;

				//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

				if (translatetime <= DELAY_TRANSLATE) { // set the time delay before start translating
					behOutput.tv = 0;
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
				} else {

					// TODO : add a timer for the duration of translation : if (translatetime <= TRANSLATE_ PERIOD)

					//behOutput.tv = TV;
					behOutput.tv = TV_GAIN * FLOCK_TV * tvgain / 100;
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);


					// TODO : else {					behSetTvRv(&behOutput, 0, 0);
					// mode = MODE_INACTIVE    }



				}

			}
			//motorSetBeh(&behOutput);

			break;
		}
		case MODE_ROTATE: {
			translatetime = 0;
			rotateTime = rotateTime + 1;
			if (rotateTime <= DELAY_ROTATE) {
				behOutput.tv = 0;
				//behSetTvRv(&behOutput, 0, 0);
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW,
						LED_RATE_MED);
			} else {

				if (broadcastMsgIsSource(&broadcastMessage)) {
					//if (rotateTime<=100){
					//	ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SNAIL);
					//behOutput = behInactive;
					//} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, LED_RATE_MED);
					behOutput = behInactive;

					//}

				} else {

					//uint16 speed = 0;
					//	if (rotateTime <=100){
					//	tv = 0;
					//		speed = LED_RATE_SNAIL;
					//	} else {
					//	tv = TV;
					uint16 speed = LED_RATE_MED;
					//	}
					//	if(broadcastMsgIsSource(&broadcastMessage))
					//	{
					//	ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, speed);
					//	behOutput = behInactive;
					//	nbrDataSet(&msgLeaderPosX , 0);
					//	nbrDataSet(&msgLeaderPosY , 0);

					//	} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, speed);
					int32 tv_gain, alpha;
					tv = 0;
					rv = 0;
					tv_gain = 0;
					behOutput.active = TRUE;
					//tv= TV_GAIN* nbrPtr->range/10;

					//alpha = behAngleNormalToLeader(leaderBearing);
					int32 error = alpha / 100;
					flockAngleIntegral2 = flockAngleIntegral2
							* FLOCK_INTEGRAL_DECAY / 1000;
					flockAngleIntegral2 += 1 * (error * error)
							* (error > 0 ? 1 : -1);
					flockAngleIntegral2 = bound(flockAngleIntegral2,
							-FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
					tv_gain = 100
							- (abs(flockAngleIntegral2) * 100)
									/ FLOCK_INTEGRAL_MAX;

					//leaderBearing = atan2MilliRad(leaderPosY , leaderPosX);
					//leaderDist = sqrtInt((uint32)dist) ;

					if (leaderBearing > 0) // turn to be normal to the leader
						alpha = normalizeAngleMilliRad2(
								leaderBearing - (int32) MILLIRAD_DEG_90);
					else

						alpha = normalizeAngleMilliRad2(
								leaderBearing + (int32) MILLIRAD_DEG_90);

					//uint32 leaderDist =dist/80 ;
					//alpha = MILLIRAD_DEG_30;
					//int32 FLOCK_RV_GAIN = 50;
					//rv = alpha * FLOCK_RV_GAIN / 100;

					int32 rv_I = 0 * flockAngleIntegral2 * K_I / 100; //WEIGHTED TEST WORKS WITH 2
					int32 rv_P = error * K_P; //WEIGHTED TEST WORKS WITH 2

					//	if (printNow) {
					//		cprintf(
					//				"error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",
					//				alpha, flockAngleIntegral, rv_I, rv_P);
					//	}

					behOutput.rv = (rv_I + rv_P);

					//tv_gain = 1;
					tv = tv_gain * leaderDist / 4300; // set tv based on the distance to the leader

					//behOutput.rv = rv;
					behOutput.tv = tv;

					//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

					//if (translatetime<=100){
					//	behOutput.tv = 0;
					//	ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					//}
					//	else {
					//behOutput.tv = TV;0

					//	}
					//	motorSetBeh(&behOutput);

					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);

					/*	if (senderID_seen == 1)

					 {
					 behOutput.rv = rv;
					 behOutput.tv = tv;

					 } else {
					 behOutput.rv = 0;
					 behOutput.tv = 0;
					 }


					 */

				}
				/*
				 for (i = 0; i < nbrList.size; ++i){
				 nbrPtr = nbrList.nbrs[i];
				 nbrLeader = nbrDataGetNbr(&msgLeader, nbrPtr);
				 //if (nbrLeader == 1)
				 if(broadcastMsgIsSource(&broadcastMessage))
				 {
				 tv= TV_GAIN* nbrPtr->range/10;
				 behFlockNormalToLeader(&behOutput, &nbrList, nbrPtr,tv);
				 }
				 }

				 if(leaderPtr != NULL){
				 ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, speed);

				 motorSetBeh(&behOutput);

				 //behOrbitRange(&behOutput,leaderPtr,tv,2);
				 } else {
				 ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, speed);
				 }
				 */

				//}
			}

			break;
		}//end rotate
		case MODE_COM: {
			ledsSetPattern(LED_BLUE,LED_PATTERN_CLAW,LED_BRIGHTNESS_MED,LED_RATE_MED);
			break;
		}
		} //end mode switch
		  //motorSetBeh(&behOutput);
		motorSetBeh(&behOutput);

		neighborsPutMutex();
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	} //forever for loop
} //behaviorTask()

/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);
	osTaskCreate(backgroundTask, "background", 1536, NULL,
			BACKGROUND_TASK_PRIORITY);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
