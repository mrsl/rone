/*
 * MRMCycloidMotion.c
 *
 *  Created on: Oct 31, 2013
 *      Author: Valerie Baretsky
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
//#define MODE_TRANSLATE						1
//#define MODE_ROTATE							2
#define MODE_LEADERSTOP						3
#define MODE_CYCLOID						4

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
//#define DELAY_TRANSLATE			300
//#define DELAY_ROTATE				500
#define DELAY_CYCLOID				100
//#define TRANSLATE_PERIOD			700   //  700 - 200 = 500 ,about 37.5 sec
#define TT_MAX			     		2000
#define DESIRED_RV_N			    15
#define DESIRED_RV_D			   	100

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
	int32 cycloidTime = 0;
	int32 angleT = 0;
	int32 theta = 0;
	int32 curX = 0;
	int32 curY = 0;
	int32 phaseShift = 0;
	NbrList nbrList;

	int32 nbrBearing = 0;
	int32 nbrDist = 0;
	int32 alpha;
	int32 flockAngleIntegral = 0;
	int32 flockAngleIntegral2 = 0;

	uint32 accelCounter = 0;
	int32 leaderPosY = 0;
	int32 leaderPosX = 0;
	int32 ModeCoM = 1; //0: pivot, 1: CoM
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
	int32 cycloidPeriod = 5000;
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
				rprintf(" %d, %d, %d, %d, %d, %d  \n", leaderPosX, leaderPosY,
						tv, rv, VectorForce, ModeCoM);

			broadcastMsgUpdate(&broadcastMessage, &nbrList);
			if (broadcastMessage.active == TRUE) {

				if (ModeCoM == 1) { //if we are in CoM mode, compute location of CoM

					//behFlockNormalToLeader(&behOutput, &nbrList, nbrPtr,tv);
					for (t = 0; t < nbrList.size; ++t) {
						nbrPtr = nbrList.nbrs[t];
						nbrBearing = nbrGetBearing(nbrPtr);
						nbrDist = nbrGetRange(nbrPtr);

						if (printNow) {
							//cprintf("dist: %d bear: %d\n", nbrDist, nbrBearing);    // commented for printing radio message
							//cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);
						}
						l1 = ((int8) nbrDataGet(&msgLeaderPosXR)) * LEADER_POS_SCALER;
						l2 = ((int8) nbrDataGet(&msgLeaderPosYR)) * LEADER_POS_SCALER;

						leaderPosX = leaderPosX + l1 + (int32)(nbrDist * cosMilliRad(nbrBearing) / MILLIRAD_TRIG_SCALER);
						leaderPosY = leaderPosY + l2 + (int32)(nbrDist * sinMilliRad(nbrBearing) / MILLIRAD_TRIG_SCALER);
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
				} else { // select pivot
					if (broadcastMsgIsSource(&broadcastMessage) && ModeCoM != 1) { //if we are the pivot
						nbrDataSet(&msgLeaderPosXS, 0);
						nbrDataSet(&msgLeaderPosYS, 0);
						//nbrDataSet(&msgLeaderPosXR, 0);
						//nbrDataSet(&msgLeaderPosYR, 0);
						if (printNow) {
							//cprintf("x: %d y: %d dist: %d bear: %d\n", 0, 0, 0, 0);  // commented for printing radio message
						}

					} else { //compute location of pivot
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

				} //end if for selecting pivot or com

			}
			//Check Buttons, won't start until leader selected
			if (buttonsGet(BUTTON_RED)) {
				//Press Red Button to go into LeaderStop mode
				broadcastMsgSetSource(&broadcastMessage, TRUE); //pressing any button makes that robot the source
				broadcastMsgDataSet(&broadcastData_Mode, MODE_LEADERSTOP);
				//translatetime = 0;
				//rotateTime = 0;
				val = MODE_LEADERSTOP;
				//valrot = 0;
				cycloidTime = 0;

			} else if (buttonsGet(BUTTON_GREEN)) {
				//Green Button not currently used

			} else if (buttonsGet(BUTTON_BLUE)) {
				//Press Blue Button to begin cycloid motion (translate and rotate)
				broadcastMsgSetSource(&broadcastMessage, TRUE); //pressing any button makes this robot the source
				broadcastMsgDataSet(&broadcastData_Mode, MODE_CYCLOID);
				//translatetime = 0;
				//rotateTime = 0;
				//valrot = 0;
				val = MODE_CYCLOID;
			}

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
				cycloidTime = 0;
				tv = 0;
				rv = 0;
				behOutput.tv = 0;
				behOutput.rv = 0;
				//cprintf("inactive \n");
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED,
						LED_RATE_MED);
				break;
			}
			case MODE_CYCLOID: {
				behOutput.active = TRUE;
				cycloidTime = cycloidTime + 1;
				angleT = angleT + 1; //time-dependent variable in cycloid eqns (the angle the circle has rotated through)
				ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED,
						LED_RATE_MED);

				//insert if statement for leader, if using leader pivot - just use CoM for now
				if(broadcastMsgIsSource(&broadcastMsg)){ //arbitrarily assign "leader" robot to have phase shift of 0
					phaseShift = 0;
				}

				//calculate phase shift
				phaseShift = 0; //replace with real calculations

				//calculate current x and y positions on cycloid, and desired angle
				curX = leaderDist * (angleT - phaseShift - sinMilliRad(angleT - phaseShift)); //cycloid eqn: x = r(t-sint)
				curY = leaderDist * (1 - cosMilliRad(angleT - phaseShift)); //cycloid eqn: y = r(1 - cost)
				theta = atan2MilliRad(y, x); //desired angle to axis of translation = arctan(y/x)

				//rotate to desired angle
				//copied from MRMObstacleAvoidance case MODE_ROTATE (needs to be altered for cycloids)
				int32 error = alpha / 100; //current angle to axis of translation - theta (/100?)
				//int32 rv_I = 0 * flockAngleIntegral2 * K_I / 100; //what is flockAngleIntegral2? Ignore for now, was 0 anyway
				int32 rv_P = 45 * error;
				//newRV = (rv_I + rv_P);
				newRV = rv_P;
				if (abs(newRV) < 120) {
					rv = 0;
					behOutput.rv = rv;
				} else {
					rv = newRV;
					behOutput.rv = rv;
				}

				if(cycloidTime < DELAY_CYCLOID){
				 //delay starting translation while robots orient themselves
				 tv = 0;
				 behOutput.tv = tv;
				 }
				else if (cycloidTime < DELAY_CYCLOID + cycloidPeriod) {
					//perform cycloid motion during the cycloid period

					//set tv
					tv = leaderDist * sqrtInt(2 - 2*cosMilliRad(angleT)); //tv = r*sqrt(2-2cost)
					//magnitude of vector sum of dx/dt and dy/dt
					behOutput.tv = tv;

					//test that buttons work, just make the robot rotate when in cycloid mode
					//rv = 500;
					//behOutput.rv = rv;
				} else {
					//stop moving when cycloid period has ended
					tv = 0;
					rv = 0;
					behOutput.tv = tv;
					behOutput.rv = rv;
					if (broadcastMsgIsSource(&broadcastMessage)) { //if you are the source, go into MODE_LEADERSTOP
						broadcastMsgDataSet(&broadcastData_Mode,
								MODE_LEADERSTOP);
					}
				}
				break;
			} //end case MODE_CYCLOID
			} //end mode switch
			motorSetBeh(&behOutput);

			neighborsPutMutex();
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
