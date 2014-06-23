/*
 * MRMObstacleAvoidance.c
 *
 *  Created on: Aug 28, 2013
 *      Author: Golnaz Habibi
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

#define NEIGHBOR_ROUND_PERIOD			550
#define MOTION_TV  						45
#define PARENT_ID_UNDECIDED				MAX_PARENT_ID
#define BUMP_RELECT_DISTANCE			35

#define TURNTIME						100
#define TV								40
#define DESIRED_TV						40

#define MODE_INACTIVE						0
#define MODE_TRANSLATE						1
#define MODE_ROTATE							2
#define MODE_LEADERSTOP						3

#define K						     	5
#define SPEED					    	60
#define TV_MAX					    	100
#define RV_MAX					    	4000

#define TV_GAIN				        	40

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
#define DELAY_TRANSLATE				300
#define DELAY_ROTATE				500
#define TRANSLATE_PERIOD			700   //  700 - 200 = 500 ,about 37.5 sec
#define TT_MAX			     		2000
#define DESIRED_RV_N			    1
#define DESIRED_RV_D			   	10

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
BroadcastMsgData broadcastData_TT;
BroadcastMsgData broadcastData_Rotation;
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
	int32 tv = 0, rv = 0;

	uint16 leader = 0;
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
	int32 alpha = 0;
	int32 flockAngleIntegral = 0;
	int32 flockAngleIntegral2 = 0;

	uint32 accelCounter = 0;
	int32 leaderPosY = 0;
	int32 leaderPosX = 0;
	int32 ModeCoM = 0;
	int32 tvgain = 0;
	int32 error = 0;

	Beh behOutput;
	int32 picknumber = 5;

	/******** Initializations ********/
	radioCommandSetSubnet(2);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	nbrDataCreate(&msgLeaderPosXR, "positionxr", 8, 0);
	nbrDataCreate(&msgLeaderPosYR, "positionyr", 8, 0);

	nbrDataCreate(&msgLeaderPosXS, "positionxs", 8, 0);
	nbrDataCreate(&msgLeaderPosYS, "positionys", 8, 0);

	broadcastMsgCreate(&broadcastMessage, 20);
	//broadcastMsgDataCreate(&broadcastData_TT, &broadcastMessage, "picknumber",picknumber);
	broadcastMsgDataCreate(&broadcastData_Mode, &broadcastMessage, "mode",
			MODE_INACTIVE);
	broadcastMsgDataCreate(&broadcastData_Rotation, &broadcastMessage,
			"rotation", 0);
	broadcastMsgDataCreate(&broadcastData_DesiredHeading, &broadcastMessage,
			"heading", 0);
	uint32 senderID_seen = 0;

	systemPrintStartup();
	uint8 val = 0;
	int32 valrot = 0;

	//uint8 tt_val = TT_MAX;

	int32 t = 0;
	int16 leaderBearing = 0;
	uint32 leaderDist = 0;
	int32 l1 = 0;
	int32 l2 = 0;
	int32 translateperiod[] = { 250, 350, 450, 550, 650, 2000 };
	int32 TransTime = 650;
	int32 RotateTime = 5000; // 141 sec for one complete circle rotation ~ 1880
	int32 newRV = 0;
	int32 rotation = 0;

	//int32 TransTime = translateperiod[picknumber];
	/******** Behavior **************/
	for (;;) {

		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW,
					LED_RATE_MED);
			continue;
		} //end if host
		else {

			behOutput = behInactive;

			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);
			//cprintf("print %d \n",printNow);
			//TODO yo uonly need to call this function once
			irCommsSetXmitPower(IRXmitPower);
			nbrListCreate(&nbrList);
			 int32 VectorForce = bumpSensorsGetBearing();
			if (printNow)
				rprintf(" %d, %d, %d \n", rv, tv, alpha);

			broadcastMsgUpdate(&broadcastMessage, &nbrList);
			if (broadcastMessage.active == TRUE) {

				if (ModeCoM == 1) {

					//behFlockNormalToLeader(&behOutput, &nbrList, nbrPtr,tv);
					for (t = 0; t < nbrList.size; ++t) {
						nbrPtr = nbrList.nbrs[t];
						nbrBearing = nbrGetBearing(nbrPtr);
						nbrDist = nbrGetRange(nbrPtr);

						if (printNow) {
							//cprintf("dist: %d bear: %d\n", nbrDist, nbrBearing);    // commented for printing radio message
							//cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);

						}

						{
							l1 = ((int8) nbrDataGet(&msgLeaderPosXR))
									* LEADER_POS_SCALER;
							l2 = ((int8) nbrDataGet(&msgLeaderPosYR))
									* LEADER_POS_SCALER;

							leaderPosX = leaderPosX + l1
									+ (int32)(
											nbrDist * cosMilliRad(nbrBearing)
													/ MILLIRAD_TRIG_SCALER);
							leaderPosY = leaderPosY + l2
									+ (int32)(
											nbrDist * sinMilliRad(nbrBearing)
													/ MILLIRAD_TRIG_SCALER);
							leaderPosX = leaderPosX / 2.0;
							leaderPosY = leaderPosY / 2.0;

							leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
							leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);

							nbrDataSet(&msgLeaderPosXS,
									(int8)(leaderPosX / LEADER_POS_SCALER));
							nbrDataSet(&msgLeaderPosYS,
									(int8)(leaderPosY / LEADER_POS_SCALER));
							int32 dist = leaderPosX * leaderPosX
									+ leaderPosY * leaderPosY;
							leaderDist = sqrtInt((uint32) dist);
							leaderBearing = atan2MilliRad(leaderPosY,
									leaderPosX);

							/*
							 * Problems:
							 * 	1- leader bearing not normalized
							 * 	2- distance never changes
							 */
							leaderBearing = normalizeAngleMilliRad2(
									leaderBearing);

						}

					}
				} else { // select pivot
					if (broadcastMsgIsSource(&broadcastMessage)
							&& ModeCoM != 1) {
						nbrDataSet(&msgLeaderPosXS, 0);
						nbrDataSet(&msgLeaderPosYS, 0);
						//nbrDataSet(&msgLeaderPosXR, 0);
						//nbrDataSet(&msgLeaderPosYR, 0);
						if (printNow) {
							//cprintf("x: %d y: %d dist: %d bear: %d\n", 0, 0, 0, 0);  // commented for printing radio message
						}

					} else

					{
						int32 senderID = broadcastMsgGetSenderID(
								&broadcastMessage);

						//behFlockNormalToLeader(&behOutput, &nbrList, nbrPtr,tv);
						for (t = 0; t < nbrList.size; ++t) {
							nbrPtr = nbrList.nbrs[t];
							if (nbrPtr->ID == senderID) {
								senderID_seen = 1;
								nbrBearing = nbrGetBearing(nbrPtr);
								nbrDist = nbrGetRange(nbrPtr);

								if (printNow) {
									//cprintf("dist: %d bear: %d\n", nbrDist,nbrBearing);  // commented for printing radio message
									//cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);

								}

								{
									l1 = ((int8) nbrDataGet(&msgLeaderPosXR))
											* LEADER_POS_SCALER;
									l2 = ((int8) nbrDataGet(&msgLeaderPosYR))
											* LEADER_POS_SCALER;

									leaderPosX =
											l1
													+ (int32)(
															nbrDist
																	* cosMilliRad(
																			nbrBearing)
																	/ MILLIRAD_TRIG_SCALER);
									leaderPosY =
											l2
													+ (int32)(
															nbrDist
																	* sinMilliRad(
																			nbrBearing)
																	/ MILLIRAD_TRIG_SCALER);
									leaderPosX = boundAbs(leaderPosX,
											LEADER_POS_BOUND);
									leaderPosY = boundAbs(leaderPosY,
											LEADER_POS_BOUND);

									nbrDataSet(
											&msgLeaderPosXS,
											(int8)(
													leaderPosX
															/ LEADER_POS_SCALER));
									nbrDataSet(
											&msgLeaderPosYS,
											(int8)(
													leaderPosY
															/ LEADER_POS_SCALER));
									int32 dist = leaderPosX * leaderPosX
											+ leaderPosY * leaderPosY;
									leaderDist = sqrtInt((uint32) dist);
									leaderBearing = atan2MilliRad(leaderPosY,
											leaderPosX);

									/*
									 * Problems:
									 * 	1- leader bearing not normalized
									 * 	2- distance never changes
									 */
									leaderBearing = normalizeAngleMilliRad2(
											leaderBearing);

								}
							}

							/*if (printNow) {
							 *
							 cprintf("x: %d y: %d dist: %d bear: %d\n", leaderPosX,
							 leaderPosY, leaderDist, leaderBearing);
							 //cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);
							 cprintf("posx: %d posy: %d \n", l1, l2);

							 }*/

						}
					}

				} //end if for selcting pivit or com

			}
			//Check Buttons, won't start until leader selected
			if (buttonsGet(BUTTON_RED)) {

				val = MODE_TRANSLATE;
				// picknumber = rand()%5;
				// TransTime = translateperiod[picknumber];
				translatetime = 0;
				rotateTime = 0;

				broadcastMsgSetSource(&broadcastMessage, TRUE);
				broadcastMsgDataSet(&broadcastData_Mode, MODE_TRANSLATE);
				//broadcastMsgDataSet(&broadcastData_TT, picknumber);
				broadcastMsgDataSet(&broadcastData_Rotation, rotation);
				valrot = rotation;

			} else if (buttonsGet(BUTTON_GREEN)) {
				// Checks Green button to stop behavior
				broadcastMsgSetSource(&broadcastMessage, TRUE);
				broadcastMsgDataSet(&broadcastData_Mode, MODE_ROTATE);
				rotation = rand() % 2 + 1;
				broadcastMsgDataSet(&broadcastData_Rotation, rotation);

				translatetime = 0;
				rotateTime = 0;
				val = MODE_ROTATE;
				valrot = rotation;

			} else if (buttonsGet(BUTTON_BLUE)) {
				broadcastMsgSetSource(&broadcastMessage, TRUE);
				broadcastMsgDataSet(&broadcastData_Mode, MODE_LEADERSTOP);
				translatetime = 0;
				rotateTime = 0;
				val = MODE_LEADERSTOP;
				valrot = 0;
			}

			//if (printNow)

			//rprintf(" LeaderPoseX %d, LeaderPoseX %d \n",leaderPosX, leaderPosY);

			//cprintf("hops, mode = %d,%d\n",                     // commented for printing radio message
			//	broadcastMsgGetHops(&broadcastMessage),
			//	broadcastMsgDataGet(&broadcastData_Mode));

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
				//cprintf("sender: %d\n",
				//broadcastMsgGetSenderID(&broadcastMessage));   // commented for printing radio message

			} else {
				if (broadcastMsgIsSource(&broadcastMessage) == FALSE) {
					val = broadcastMsgDataGet(&broadcastData_Mode);
					valrot = broadcastMsgDataGet(&broadcastData_Rotation);

					//picknumber = broadcastMsgDataGet(&broadcastData_TT);
				} else {
					valrot = rotation;
				}
				//cprintf("val: %d\n", val);
				broadcastMsgDataSet(&broadcastData_Mode, val);
				broadcastMsgDataSet(&broadcastData_Rotation, valrot);

				//broadcastMsgDataSet(&broadcastData_TT, picknumber);
				//TransTime = translateperiod[picknumber];
			}
			switch (val) {
			case MODE_INACTIVE: {
				//cprintf("inactive \n");
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED,
						LED_RATE_MED);
				break;
			}
			case MODE_LEADERSTOP: {
				behOutput = behInactive;
				rotateTime = 0;
				translatetime = 0;

				tv = 0;
											rv = 0;
											behSetTvRv(&behOutput, tv, rv);
								//cprintf("inactive \n");
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED,
										LED_RATE_MED);



								break;




			}
			case MODE_TRANSLATE: {
				rotateTime = 0;
				translatetime = translatetime + 1;

				if (broadcastMsgIsSource(&broadcastMessage)) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, LED_RATE_MED);

					//Pulsing Green LEDS indicate waiting to translate
					//Circling Green LEDS indicate translation has begun
					if (translatetime <= DELAY_TRANSLATE) { // set the time delay before start translating
						tv = 0;
						rv = 0;
						behSetTvRv(&behOutput, tv, rv);
					} else {

						// TODO : add a timer for the duration of translation : if (translatetime <= TRANSLATE_ PERIOD)

						if (translatetime <= DELAY_TRANSLATE + TransTime) {
							/*if (bumpSensorsGetBits() != 0) {
							 int32 bearingToForce = -bumpSensorsGetBearing();
							 behSetTvRv(&behOutput, TV *3/4,-bearingToForce * 50/100);

							 } else {
							 behSetTvRv(&behOutput, TV, 0);
							 }
							 */
							tv = DESIRED_TV;
							rv = 0;
							behSetTvRv(&behOutput, tv, rv);

						} else {
							tv = 0;
							rv = 0;
							behSetTvRv(&behOutput, tv, rv);

							behSetTvRv(&behOutput, tv, rv);
							val = MODE_LEADERSTOP;
							broadcastMsgDataSet(&broadcastData_Mode, val);

						}

					}
				}

				else {
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_MED);
					behOutput.active = TRUE;

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
					error = alpha;
					flockAngleIntegral = flockAngleIntegral
							* FLOCK_INTEGRAL_DECAY / 1000;
					flockAngleIntegral += (error / 100) * (error > 0 ? 1 : -1);
					flockAngleIntegral = bound(flockAngleIntegral,
							-FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
					tvgain = 100
							- (abs(flockAngleIntegral) * 100
									/ FLOCK_INTEGRAL_MAX);
					//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
					//behMove.rv = flockAngleIntegral;

					int32 rv_I = 0 * flockAngleIntegral * K_I / 100; //WEIGHTED TEST WORKS WITH 2
					int32 rv_P = 1 * error * K_P / 100; //WEIGHTED TEST WORKS WITH 2

					//if (printNow) {
					//cprintf(
					//"error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",
					//alpha, flockAngleIntegral, rv_I, rv_P);   // commented for printing radio message
					//}

					newRV = (rv_I + rv_P);

					//rv = ((100 - (networkSize*100)/4) * rv) / 100;

					//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

					if (translatetime <= DELAY_TRANSLATE) { // set the time delay before start translating
						tv = 0;

						behOutput.tv = tv;
						if (abs(newRV) < 120) {
							rv = 0;
							behOutput.rv = rv;
						} else {
							rv = newRV;
							behOutput.rv = rv;
						}

					} else {

						if (abs(newRV) < 120) {
							rv = 0;
							behOutput.rv = rv;
						} else {
							rv = newRV;
							behOutput.rv = rv;
						}

						// TODO : add a timer for the duration of translation : if (translatetime <= TRANSLATE_ PERIOD)
						if (translatetime <= TransTime + DELAY_TRANSLATE) {
							//behOutput.tv = TV;

							tv = 1 * DESIRED_TV * tvgain / 100;
							behOutput.tv = tv;

						}

						else {
							rv = 0;
							tv = 0;

							behSetTvRv(&behOutput, tv, rv);

						}

					}

				}
				motorSetBeh(&behOutput);

				break;
			}
			case MODE_ROTATE: {
				translatetime = 0;
				rotateTime = rotateTime + 1;

				if (broadcastMsgIsSource(&broadcastMessage) && ModeCoM != 1) {
					//	ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SNAIL);
					//behOutput = behInactive;
					//} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, LED_RATE_MED);
					behOutput = behInactive;
					broadcastMsgDataSet(&broadcastData_Rotation, rotation);
					if (rotateTime <= DELAY_ROTATE + RotateTime) {
						/*if (bumpSensorsGetBits() != 0) {
						 int32 bearingToForce = -bumpSensorsGetBearing();
						 behSetTvRv(&behOutput, TV/4,-50* bearingToForce/100);

						 } else {
						 behSetTvRv(&behOutput, 0, 0);

						 }*/ // the pivot rotates in its position
						rv = 0;
						tv = 0;
						behSetTvRv(&behOutput, tv, rv);

					} else

					{
						broadcastMsgDataSet(&broadcastData_Mode, MODE_LEADERSTOP);

						rv = 0;
						tv = 0;
						behSetTvRv(&behOutput, tv, rv);
					}

					//behSetTvRv(&behOutput, 0, 100);

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
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, speed);
					int32 alpha;
					tv = 0;
					rv = 0;
					tvgain = 0;
					behOutput.active = TRUE;
					//tv= TV_GAIN* nbrPtr->range/10;

					//alpha = behAngleNormalToLeader(leaderBearing);

					//leaderBearing = atan2MilliRad(leaderPosY , leaderPosX);
					//leaderDist = sqrtInt((uint32)dist) ;

					//if (leaderBearing > 0) // turn to be normal to the leader
					//	alpha = normalizeAngleMilliRad2(leaderBearing - (int32) MILLIRAD_DEG_90);
					//else

					//	alpha = normalizeAngleMilliRad2(leaderBearing + (int32) MILLIRAD_DEG_90);
					if (valrot == 1)
						alpha = normalizeAngleMilliRad2(
								-1 * (int32) MILLIRAD_DEG_90 + leaderBearing);
					else if (valrot == 2)
						alpha = normalizeAngleMilliRad2(
								(int32) MILLIRAD_DEG_90 + leaderBearing);
					else
						alpha = 0;
					//uint32 leaderDist =dist/80 ;
					//alpha = MILLIRAD_DEG_30;
					//int32 FLOCK_RV_GAIN = 50;
					//rv = alpha * FLOCK_RV_GAIN / 100;
					int32 error = alpha / 100;
					flockAngleIntegral2 = flockAngleIntegral2
							* FLOCK_INTEGRAL_DECAY / 1000;
					flockAngleIntegral2 += 1 * (error) / 100
							* (error > 0 ? 1 : -1);
					flockAngleIntegral2 = bound(flockAngleIntegral2,
							-FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
					tvgain = 100
							- (abs(flockAngleIntegral2) * 100
									/ FLOCK_INTEGRAL_MAX);

					int32 rv_I = 0 * flockAngleIntegral2 * K_I / 100; //WEIGHTED TEST WORKS WITH 2
					int32 rv_P = 45 * error; //WEIGHTED TEST WORKS WITH 2

					//	if (printNow) {
					//		cprintf(
					//				"error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",
					//				alpha, flockAngleIntegral, rv_I, rv_P);
					//	}

					newRV = (rv_I + rv_P);
					if (abs(newRV) < 120) {
						rv = 0;

						behOutput.rv = rv;
					} else {
						rv = newRV;
						behOutput.rv = rv;
					}
					//tv_gain = 1;

					if (rotateTime <= DELAY_ROTATE) {
						tv = 0;
						behOutput.tv = tv;
						//behSetTvRv(&behOutput, 0, 0);
						//ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW,LED_RATE_MED);
					} else {
						//behOutput.rv = rv;
						if (rotateTime <= DELAY_ROTATE + RotateTime) {
							tv = 100 * (DESIRED_RV_N * leaderDist)
									/ (100 * DESIRED_RV_D); // set tv based on the distance to the leader
							behOutput.tv = tv;
							// cprintf("mode % 4d  rotateTime  % 4d  tv %4d  dist %4d \n",	val, rotateTime,tv, leaderDist);  // commented for printing radio message

						} else {
							tv = 0;
							rv = 0;
							behOutput.tv = tv;

							behOutput.rv = rv;

							if (broadcastMsgIsSource(&broadcastMessage)) {

								broadcastMsgDataSet(&broadcastData_Mode,
										MODE_LEADERSTOP);
								tv = 0;
															rv = 0;
															behOutput.tv = tv;

															behOutput.rv = rv;

							}

						}

						//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

						//if (translatetime<=100){
						//	behOutput.tv = 0;
						//	ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
						//}
						//	else {
						//behOutput.tv = TV;0

						//	}
						//	motorSetBeh(&behOutput);

						//ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
						//	LED_BRIGHTNESS_LOW, LED_RATE_MED);

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
			}
			} //end mode switch
			  //motorSetBeh(&behOutput);
			motorSetBeh(&behOutput);

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
