/*
 * MRMMotionControl.c
 *
 *  Created on: Jan 25, 2014
 *      Author: Golnaz Habibi
 */

// This code is written to motion control robots that transport the object, in this controller robots are either translating or rotating based on the bottom that is pressed on the leader robot,.
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nbrDataCoM.h"

#include "roneos.h"
#include "ronelib.h"

/* CONSTANTS *****************************************************/
#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

#define NEIGHBOR_ROUND_PERIOD			2000
#define MOTION_TV				45
#define PARENT_ID_UNDECIDED			MAX_PARENT_ID
#define BUMP_RELECT_DISTANCE			35

#define TURNTIME				100
#define TV					40
#define DESIRED_TV				40

#define MODE_INACTIVE				0
#define MODE_TRANSLATE				1
#define MODE_ROTATE				2
#define MODE_LEADERSTOP				3
#define MODE_ESTIMATE				0
#define COM_STATE				1
#define PIVOT_STATE				0
#define ROT_MODE				1
#define K					5
#define SPEED6					0
#define TV_MAX					100
#define RV_MAX					4000
#define RV_GAIN
#define TV_GAIN					40

#define FLOCK_TV				10

#define FLOCK_INTEGRAL_MAX			4000
#define FLOCK_INTEGRAL_DECAY			970
#define FLOCK_INTEGRAL_DECAY2			4

#define K_INTEGRAL_MAX1000 //400 works, 300 doesn't
#define DECAY_INTEGRAL70

#define OBJECT_MASS				778
#define K_PI_N					100
#define K_PI_D	    				100
#define K_I				        20
#define K_D				        75
#define K_P_ROTATION            27
#define ERROR_MAX               100
#define ERROR_MAX_TRANSLATION    90

#define K_P_TRANSLATE			80   ///
#define DELAY_TRANSLATE				 600 //25
#define DELAY_ROTATE				900 // 30
#define TRANSLATE_PERIOD			 700    //40//  700 - 200 = 500 ,about 37.5 sec
#define ROTATE_PERIOD			    700   //  700 - 200 = 500 ,about 37.5 sec


#define TT_MAX			     		2000
#define DESIRED_TV_N		  		1
#define DESIRED_TV_D			   	15
#define RANGE_IIR      				60
#define STARTUP_STATE				0
#define IDLE_STATE				1
#define REQUEST_STATE				2
#define REQUEST_PROBABILITY			3 //3
#define START_ROUND				0
#define IDLE_ROUND_MAX  			20
#define ACK_ROUND_MAX				40  // 2000
#define REQUEST_ROUND_MAX			70 //70   3500
#define WAIT_ROUND_MAX				70 always REQUEST_ROUND_MAX + 1
#define NONCE_MAX				100
#define LEADER_POS_SCALER			10
#define LEADER_POS_BOUND			(127 * LEADER_POS_SCALER )
#define ESTIMATE_MIN 				100
#define ROT_MAX					0
#define ROTATE_MAX				5
/* User-defined functions *****************************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);
int32 updatePosX(Nbr* nbrPtr, int8 CoMStoredPosX, int8 CoMStoredPosY);
int32 updatePosY(Nbr* nbrPtr, int8 CoMStoredPosX, int8 CoMStoredPosY);

/* global variables ***********************************************/
boolean printNow = FALSE;
BroadcastMessage broadcastMessage;
BroadcastMsgData broadcastData_Mode;
BroadcastMsgData broadcastData_DesiredHeading;
BroadcastMsgData broadcastData_TT;
BroadcastMsgData broadcastData_Rotation;
// the background task runs all the time.  Put slow stuff here, like compute intensive functions
// or large data transfers, like getting the Yun's map from the robot.

int32 updatePosX(Nbr* nbrPtr, int8 CoMStoredPosX, int8 CoMStoredPosY) {

	int32 nbrBearing = nbrGetBearing(nbrPtr);
	int32 nbrOrientation = nbrGetOrientation(nbrPtr);
	int16 smallestAngle = normalizeAngleMilliRad2((int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
	//int32 nbrDist = nbrGetRange(nbrPtr);
	int16 nbrDist = getnbrRange(nbrPtr->ID, roneID);

	//int16 nbrDist = (nbrPtr->range) / 2;
	//nbrDist = 120;
	//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );

	int8 X = CoMStoredPosX * LEADER_POS_SCALER;
	int8 Y = CoMStoredPosY * LEADER_POS_SCALER;
	int32 sinAngle = sinMilliRad(smallestAngle);
	int32 cosAngle = cosMilliRad(smallestAngle);
	int32 cosBearing = cosMilliRad(nbrBearing);

	// int32 leaderPosXNbr1 = (nbrDist * cosBearing) / (MILLIRAD_TRIG_SCALER);
	// int32 leaderPosXNbr2 = (X * cosAngle / (MILLIRAD_TRIG_SCALER));
	// int32 leaderPosXNbr3 = -(Y * sinAngle / (MILLIRAD_TRIG_SCALER));
	//  return leaderPosXNbr1 + leaderPosXNbr2 + leaderPosXNbr3;
	return ((nbrDist * cosBearing) + (X * cosAngle) - (Y * sinAngle)) / MILLIRAD_TRIG_SCALER;

}



int32 updatePosY(Nbr* nbrPtr, int8 CoMStoredPosX, int8 CoMStoredPosY) {

	int32 nbrBearing = nbrGetBearing(nbrPtr);
	int32 nbrOrientation = nbrGetOrientation(nbrPtr);
	int16 smallestAngle = normalizeAngleMilliRad2((int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
	int16 nbrDist = getnbrRange(nbrPtr->ID, roneID);
	//int16 nbrDist = (nbrPtr->range) / 2;
	//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );
	//nbrDist = 120;
	//rprintf(" %d \n", nbrDist);
	int8 X = CoMStoredPosX * LEADER_POS_SCALER;
	int8 Y = CoMStoredPosY * LEADER_POS_SCALER;
	int32 sinAngle = sinMilliRad(smallestAngle);
	int32 cosAngle = cosMilliRad(smallestAngle);
	int32 sinBearing = sinMilliRad(nbrBearing);
	//int32 leaderPosYNbr1 = (nbrDist * sinBearing) / (MILLIRAD_TRIG_SCALER);
	//  int32 leaderPosYNbr2 = (X * sinAngle / (MILLIRAD_TRIG_SCALER));
	// int32 leaderPosYNbr3 = (X * sinAngle / (MILLIRAD_TRIG_SCALER));
	// return leaderPosYNbr1 + leaderPosYNbr2 + leaderPosYNbr3;
	return ((nbrDist * sinBearing) + (X * sinAngle) + (Y * cosAngle)) / MILLIRAD_TRIG_SCALER;

}

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
	int32 rotationState = COM_STATE; //PIVOT_STATE: pivot, COM_STATE: CoM

	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound = 0;
	uint8 i = 0;
	uint16 IRXmitPower = IR_COMMS_POWER_MAX;

	int32 rotationMode = 0;
	int32 stateval = 0;
	int32 ignoreRequestTimer = 0;
	int32 reqRound = 0;
	int32 waitRound = 0;
	int32 startRound = START_ROUND;
	Nbr* requestnbr;
	int32 actionMode = 0;
	int32 req_prob = 0;
	int32 alpha = 0;
	NbrDataCoM msgDataCoM;

	Nbr* nbrPtr;
	NbrList nbrList;

	int32 nbrBearing = 0;
	int32 nbrDist = 0;
	int32 CoMPosX = 0;
	int32 CoMPosY = 0;
	int32 PrevCoMPosX = 0;
	int32 PrevCoMPosY = 0;

	int32 tvgain = 0;
	int32 error = 0;

	int32 CoMNbrRequestTempStorageX = 0;
	int32 CoMNbrRequestTempStorageY = 0;

	/******** Initializations ********/
	radioCommandSetSubnet(4);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	nbrDataCoMCreate(&msgDataCoM, "SharedData");

	systemPrintStartup();
	rprintfSetSleepTime(100);
//uint8 tt_val = TT_MAX;

	int32 t = 0;
	int16 leaderBearing = 0;
	uint32 leaderDist = 0;
	int32 X = 0;
	int32 Y = 0;
	int32 RotateTime = 4000; // 141 sec for one complete circle rotation ~ 1880
	int32 newRV = 0;
	int32 rotation = 0;
	uint8 PosData[3];
	uint8 nonce = 0;
	uint8 ackID = 0;
	uint8 randomNbr = 0;
	uint8 requestNoncePrev = 0;
	uint8 requestNbrIDPrev = ROBOT_ID_NULL;
	int32 idleRound = 0;
	int32 ackRecieved = 0;
	uint8 noncenbr = 0;
	uint8 fail = 0;
	int32 nbrsize = 0;
	boolean newSensorData;
	int32 tv = 0, rv = 0;
	uint16 leader = 0;
	uint16 nbrMode = MODE_INACTIVE;
	uint16 nbrLeader = 0;
	uint16 wait = 0;

	NbrData msgLeaderPosX;
	NbrData msgLeaderPosY;

	Nbr* leaderPtr;
	int32 translatetime = 0;
	int32 rotateTime = 0;

	int32 flockAngleIntegral = 0;
	int32 flockAngleIntegral2 = 0;

	uint32 accelCounter = 0;
	int32 leaderPosY = 0;
	int32 leaderPosX = 0;

	Beh behOutput;
	int32 picknumber = 5;

	nbrDataCreate(&msgLeaderPosX, "positionX", 8, 0);
	nbrDataCreate(&msgLeaderPosY, "positionY", 8, 0);

	broadcastMsgCreate(&broadcastMessage, 20);
//broadcastMsgDataCreate(&broadcastData_TT, &broadcastMessage, "picknumber",picknumber);
	broadcastMsgDataCreate(&broadcastData_Mode, &broadcastMessage, "mode", MODE_INACTIVE);
	broadcastMsgDataCreate(&broadcastData_Rotation, &broadcastMessage, "rotation", 0);
	broadcastMsgDataCreate(&broadcastData_DesiredHeading, &broadcastMessage, "heading", 0);
	uint32 senderID_seen = 0;
	int32 newData = 0;
	uint8 val = 0;
	int32 valrot = 0;

//uint8 tt_val = TT_MAX;

	uint32 dist = 0;
	int32 rotTime = 0;

	int32 l1 = 0;
	int32 l2 = 0;
	int32 translateperiod[] = { 250, 350, 450, 550, 650, 2000 };
	int32 TransTime = 650;

	uint8 storedPosX = 0;
	uint8 storedPosY = 0;

//int32 rotationState = 1; // set 0 for Pivot,  1 for COM
//int32 TransTime = translateperiod[picknumber];
	/******** Behavior **************/
	for (;;) {

		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
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

			if (printNow)
				rprintf(" %d, %d, %d, %d, %d \n", rv, tv, alpha, leaderPosX, leaderPosY);

			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			int32 VectorForce = bumpSensorsGetBearing();

			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			// select pivot

			/*if (printNow) {
			 *
			 cprintf("x: %d y: %d dist: %d bear: %d\n", leaderPosX,
			 leaderPosY, leaderDist, leaderBearing);
			 //cprintf("posx: %d posy: %d \n",&msgLeaderPosX.value,&msgLeaderPosY.value);
			 cprintf("posx: %d posy: %d \n", l1, l2);

			 }*/

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

				rotation = 1;
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
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
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
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);

				break;

			}
			case MODE_TRANSLATE: {
				rotateTime = 0;
				//if (printNow)
				//{
					translatetime = translatetime + 1;
				//}

				if (broadcastMsgIsSource(&broadcastMessage)) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);

					//Pulsing Green LEDS indicate waiting to translate
					//Circling Green LEDS indicate translation has begun
					if (translatetime <= DELAY_TRANSLATE)
					{ // set the time delay before start translating
						tv = 0;
						rv = 0;
						behSetTvRv(&behOutput, tv, rv);
					} else {

						// TODO : add a timer for the duration of translation : if (translatetime <= TRANSLATE_ PERIOD)

						if (translatetime <= DELAY_TRANSLATE + TRANSLATE_PERIOD) {
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

				else // the robot that is not a source
				{
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					behOutput.active = TRUE;

					alpha = behFlockAngle(&nbrList); // consensus, be aligned with the leader
					// start PI controller
					error = alpha;
					flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY / 1000;
					flockAngleIntegral += (error / 100) * (error > 0 ? 1 : -1);
					flockAngleIntegral = bound(flockAngleIntegral, -FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
					tvgain = 100 - (abs(flockAngleIntegral) * 100 / FLOCK_INTEGRAL_MAX);
					//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
					//behMove.rv = flockAngleIntegral;

					int32 rv_I = 0 * flockAngleIntegral * K_I / 100; //WEIGHTED TEST WORKS WITH 2
					int32 rv_P = error * K_P_TRANSLATE / 100; //WEIGHTED TEST WORKS WITH 2

					//if (printNow) {
					//cprintf(
					//"error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",
					//alpha, flockAngleIntegral, rv_I, rv_P);   // commented for printing radio message
					//}

					newRV = (rv_I + rv_P);

					//rv = ((100 - (networkSize*100)/4) * rv) / 100;

					//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

					if (translatetime <= DELAY_TRANSLATE)
					{ // set the time delay before start translating
						tv = 0;

						behOutput.tv = tv;
						if (abs(newRV) <= ERROR_MAX_TRANSLATION)
						{
							rv = 0;
							behOutput.rv = rv;
						} else {
							rv = newRV;
							behOutput.rv = rv;
						}

					} else {

						if (abs(newRV) <= ERROR_MAX_TRANSLATION)
						{
							rv = 0;
							behOutput.rv = rv;
						} else {
							rv = newRV;
							behOutput.rv = rv;
						}

						// TODO : add a timer for the duration of translation : if (translatetime <= TRANSLATE_ PERIOD)
						if (translatetime <= TransTime + DELAY_TRANSLATE)
						{
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

			case MODE_ROTATE:

			{

				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);

				nbrDataSet(&msgLeaderPosX, (uint8)(leaderPosX / LEADER_POS_SCALER));
				nbrDataSet(&msgLeaderPosY, (uint8)(leaderPosY / LEADER_POS_SCALER));

				if (actionMode == MODE_ESTIMATE)
				{
					if (rotationState == COM_STATE)
					{

						{
							nbrDataCoMSetPos(&msgDataCoM, (uint8)(leaderPosX / LEADER_POS_SCALER), (uint8)(leaderPosY / LEADER_POS_SCALER));

							if (stateval == STARTUP_STATE)
							{
								nbrsize = nbrListGetSize(&nbrList);

								ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								if (startRound > 0) {
									//if (printNow)
								//	{

										startRound--;

							//	}
								} else {
									stateval = IDLE_STATE;

								}

							}

							if (stateval == IDLE_STATE)
							{
								//cprintf("inactive \n");
								ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								nbrDataCoMSetReqID(&msgDataCoM, ROBOT_ID_NULL);

								waitRound = 0;
								reqRound = 0;
							//	if (printNow)
							//	{


									idleRound++;


							//	}
								nbrsize = nbrListGetSize(&nbrList);
								if (nbrsize > 0) {

									if (ignoreRequestTimer == 0) {
										// timer is zero.  We can check for new acknoweledgements

										for (t = 0; t < nbrsize; ++t) {
											nbrPtr = nbrListGetNbr(&nbrList, t);
											if ((nbrDataCoMGetReqID(&msgDataCoM, nbrPtr) == roneID)
													&& !((nbrDataCoMGetNonce(&msgDataCoM, nbrPtr) == requestNoncePrev)
															&& (nbrGetID(nbrPtr) == requestNbrIDPrev))) {
												// copmute new CoM and Ack
												requestNbrIDPrev = nbrGetID(nbrPtr);
												requestNoncePrev = nbrDataCoMGetNonce(&msgDataCoM, nbrPtr);
												CoMNbrRequestTempStorageX = nbrDataCoMGetx(&msgDataCoM, nbrPtr);
												CoMNbrRequestTempStorageY = nbrDataCoMGety(&msgDataCoM, nbrPtr);
												/*	PrevCoMPosX = leaderPosX;
												 PrevCoMPosY = leaderPosY;

												 leaderPosX = (updatePosX(nbrPtr,
												 CoMNbrRequestTempStorageX,
												 CoMNbrRequestTempStorageY) + leaderPosX)
												 / 2;
												 leaderPosY = (updatePosY(nbrPtr,
												 CoMNbrRequestTempStorageX,
												 CoMNbrRequestTempStorageY) + leaderPosY)
												 / 2;
												 leaderPosX = boundAbs(leaderPosX,
												 LEADER_POS_BOUND);
												 leaderPosY = boundAbs(leaderPosY,
												 LEADER_POS_BOUND);*/
												//	if (abs(leaderPosX - CoMPosX)
												//		<= ESTIMATE_MIN
												//		&& abs(leaderPosY - CoMPosY)
												//			<= ESTIMATE_MIN) {
												//actionMode = ROT_MODE;
												//rotTime = ROT_MAX;
												//newData = 1;
												//	}
												//  storedPosX = nbrDataGetNbr(&msgLeaderPosX, nbrPtr);
												//    storedPosY =  nbrDataGetNbr(&msgLeaderPosY, nbrPtr);
												// leaderPosX = updatePosX(nbrPtr, storedPosX, storedPosY);
												//leaderPosY = updatePosY(nbrPtr, storedPosX, storedPosY);
												int32 nbrBearing = nbrGetBearing(nbrPtr);
												int32 nbrOrientation = nbrGetOrientation(nbrPtr);
												int16 smallestAngle = normalizeAngleMilliRad2(
														(int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
												//int32 nbrDist = nbrGetRange(nbrPtr);
												int16 nbrDist = getnbrRange(nbrPtr->ID, roneID);

											//	int16 nbrDist = (nbrPtr->range);

												//int16 nbrDist = getnbrRange(nbrPtr->ID, roneID);

												//nbrDist = 120;
												//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );

												int8 X = CoMNbrRequestTempStorageX * LEADER_POS_SCALER;
												int8 Y = CoMNbrRequestTempStorageY * LEADER_POS_SCALER;
												int32 sinAngle = sinMilliRad(smallestAngle);
												int32 cosAngle = cosMilliRad(smallestAngle);
												int32 cosBearing = cosMilliRad(nbrBearing);
												int32 sinBearing = sinMilliRad(nbrBearing);

												// int32 leaderPosXNbr1 = (nbrDist * cosBearing) / (MILLIRAD_TRIG_SCALER);
												// int32 leaderPosXNbr2 = (X * cosAngle / (MILLIRAD_TRIG_SCALER));
												// int32 leaderPosXNbr3 = -(Y * sinAngle / (MILLIRAD_TRIG_SCALER));
												//  return leaderPosXNbr1 + leaderPosXNbr2 + leaderPosXNbr3;
												leaderPosX = ((nbrDist * cosBearing) + (X * cosAngle) - (Y * sinAngle))
														/ MILLIRAD_TRIG_SCALER;

												leaderPosY = ((nbrDist * sinBearing) + (X * sinAngle) + (Y * cosAngle))
														/ MILLIRAD_TRIG_SCALER;

												leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
												leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);

												//int32 nbrBearing = nbrGetBearing(nbrPtr);
												//int32 nbrOrientation = nbrGetOrientation(nbrPtr);
												//int16 smallestAngle = normalizeAngleMilliRad2((int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
												//int16 leaderDist = (nbrPtr->range)/2;
												//   if(printNow)
												//	rprintf("  %d, %d, %d, %d, %d, %d \n", leaderPosX, leaderPosY,leaderDist, nbrBearing, nbrOrientation, smallestAngle);

												//  actionMode = ROT_MODE;
												//  rotTime = ROT_MAX;

												leaderBearing = normalizeAngleMilliRad2(atan2MilliRad(leaderPosY, leaderPosX));
												leaderDist = sqrtInt((uint32)(leaderPosX * leaderPosX + leaderPosY * leaderPosY));

												nbrDataCoMSetAckID(&msgDataCoM, requestNbrIDPrev);
												nbrDataCoMSetReqID(&msgDataCoM, ROBOT_ID_NULL);

												nbrDataCoMSetNonce(&msgDataCoM, requestNoncePrev);
												ignoreRequestTimer = ACK_ROUND_MAX;
												break;
											}
										}

										// if we are not acking a message, look for someone to average with
										if (ignoreRequestTimer == 0) {
											if ((stateval == IDLE_STATE) && (idleRound > IDLE_ROUND_MAX)) {
												if ((rand() % 11 - 1) >= REQUEST_PROBABILITY)
												{
													randomNbr = rand() % (nbrListGetSize(&nbrList));
													nbrPtr = nbrListGetNbr(&nbrList, randomNbr);
													nbrDataCoMSetReqID(&msgDataCoM, nbrGetID(nbrPtr));
													nbrDataCoMSetAckID(&msgDataCoM, ROBOT_ID_NULL);

													// store current CoM values of neighbor
													CoMNbrRequestTempStorageX = nbrDataCoMGetx(&msgDataCoM, nbrPtr);
													CoMNbrRequestTempStorageY = nbrDataCoMGety(&msgDataCoM, nbrPtr);
													//if (printNow)
												//	{

														nonce++;

											//	}
													nbrDataCoMSetNonce(&msgDataCoM, nonce);
													reqRound = REQUEST_ROUND_MAX;
													stateval = REQUEST_STATE;
												}
											}
										}
									} else {

									//	if (printNow)
										//{
											ignoreRequestTimer--;
										//}

									}

								}

							}
							if (stateval == REQUEST_STATE)
							{
								nbrsize = nbrListGetSize(&nbrList);

								ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
								if (reqRound > 0) {


									//if (printNow)
									//{

										reqRound--;



								//	}

									for (t = 0; t < nbrList.size; ++t) {
										nbrPtr = nbrListGetNbr(&nbrList, t);
										if (nbrDataCoMGetAckID(&msgDataCoM, nbrPtr) == roneID
												&& nbrDataCoMGetNonce(&msgDataCoM, nbrPtr) == nonce) {
											/*PrevCoMPosX = leaderPosX;
											 PrevCoMPosY = leaderPosY;

											 leaderPosX = (updatePosX(nbrPtr,
											 CoMNbrRequestTempStorageX,
											 CoMNbrRequestTempStorageY) + leaderPosX) / 2;
											 leaderPosY = (updatePosY(nbrPtr,
											 CoMNbrRequestTempStorageX,
											 CoMNbrRequestTempStorageY) + leaderPosY) / 2;
											 leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
											 leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);
											 //newData = 1;
											 //if (abs(leaderPosX - CoMPosX)
											 //		<= ESTIMATE_MIN
											 //		&& abs(leaderPosY - CoMPosY)
											 //			<= ESTIMATE_MIN) {
											 actionMode = ROT_MODE;
											 rotTime = ROT_MAX;

											 leaderBearing = normalizeAngleMilliRad2(atan2MilliRad(leaderPosY, leaderPosX));
											 leaderDist =sqrtInt((uint32)( leaderPosX * leaderPosX + leaderPosY * leaderPosY));
											 //leaderDist = sqrtInt((uint32) dist);
											 //	}
											 *
											 *
											 *
											 *
											 *
											 */

											storedPosX = CoMNbrRequestTempStorageX;
											storedPosY = CoMNbrRequestTempStorageY;
											// leaderPosX = updatePosX(nbrPtr, storedPosX, storedPosY);
											//leaderPosY = updatePosY(nbrPtr, storedPosX, storedPosY);

											int32 nbrBearing = nbrGetBearing(nbrPtr);
											int32 nbrOrientation = nbrGetOrientation(nbrPtr);
											int16 smallestAngle = normalizeAngleMilliRad2(
													(int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
											//int32 nbrDist = nbrGetRange(nbrPtr);
//
										//	int16 nbrDist = (nbrPtr->range);

											int16 nbrDist = getnbrRange(nbrPtr->ID, roneID);

											//nbrDist = 120;
											//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );

											int8 X = CoMNbrRequestTempStorageX * LEADER_POS_SCALER;
											int8 Y = CoMNbrRequestTempStorageY * LEADER_POS_SCALER;
											int32 sinAngle = sinMilliRad(smallestAngle);
											int32 cosAngle = cosMilliRad(smallestAngle);
											int32 cosBearing = cosMilliRad(nbrBearing);
											int32 sinBearing = sinMilliRad(nbrBearing);

											// int32 leaderPosXNbr1 = (nbrDist * cosBearing) / (MILLIRAD_TRIG_SCALER);
											// int32 leaderPosXNbr2 = (X * cosAngle / (MILLIRAD_TRIG_SCALER));
											// int32 leaderPosXNbr3 = -(Y * sinAngle / (MILLIRAD_TRIG_SCALER));
											//  return leaderPosXNbr1 + leaderPosXNbr2 + leaderPosXNbr3;
											leaderPosX = ((nbrDist * cosBearing) + (X * cosAngle) - (Y * sinAngle)) / MILLIRAD_TRIG_SCALER;

											leaderPosY = ((nbrDist * sinBearing) + (X * sinAngle) + (Y * cosAngle)) / MILLIRAD_TRIG_SCALER;

											leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
											leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);

											//int32 nbrBearing = nbrGetBearing(nbrPtr);
											//int32 nbrOrientation = nbrGetOrientation(nbrPtr);
											//int16 smallestAngle = normalizeAngleMilliRad2((int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
											//int16 leaderDist = (nbrPtr->range)/2;
											//   if(printNow)
											//	rprintf("  %d, %d, %d, %d, %d, %d \n", leaderPosX, leaderPosY,leaderDist, nbrBearing, nbrOrientation, smallestAngle);

											//  actionMode = ROT_MODE;
											//  rotTime = ROT_MAX;

											leaderBearing = normalizeAngleMilliRad2(atan2MilliRad(leaderPosY, leaderPosX));
											leaderDist = sqrtInt((uint32)(leaderPosX * leaderPosX + leaderPosY * leaderPosY));

											stateval = IDLE_STATE;
											nbrDataCoMSetReqID(&msgDataCoM, ROBOT_ID_NULL);

											break;

										}
									}

								} else {

									//if (printNow)
									//{

										fail++;

								//	}
									stateval = IDLE_STATE;
								}

							}

						}
					} else

					{
						if (rotationState == PIVOT_STATE)
						{

							if (broadcastMsgIsSource(&broadcastMessage)) {
								leaderPosX = 0;
								leaderPosY = 0;

								nbrDataSet(&msgLeaderPosX, (uint8) leaderPosX);
								nbrDataSet(&msgLeaderPosY, (uint8) leaderPosY);
							}

							else // if it is not the source

							{

								behOutput.active = TRUE;

								int32 senderID = broadcastMsgGetSenderID(&broadcastMessage);
								//behFlockNormalToLeader(&behOutput, &nbrList, nbrPtr,tv);
								for (t = 0; t < nbrList.size; ++t) {
									nbrPtr = nbrList.nbrs[t];
									if (nbrPtr->ID == senderID) {
										storedPosX = nbrDataGetNbr(&msgLeaderPosX, nbrPtr);
										storedPosY = nbrDataGetNbr(&msgLeaderPosY, nbrPtr);
										// leaderPosX = updatePosX(nbrPtr, storedPosX, storedPosY);
										//leaderPosY = updatePosY(nbrPtr, storedPosX, storedPosY);

										int32 nbrBearing = nbrGetBearing(nbrPtr);
										int32 nbrOrientation = nbrGetOrientation(nbrPtr);
										int16 smallestAngle = normalizeAngleMilliRad2((int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
										//int32 nbrDist = nbrGetRange(nbrPtr);

										//int16 nbrDist = (nbrPtr->range);
										//nbrDist = 120;
										//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );
										int16 nbrDist = getnbrRange(nbrPtr->ID, roneID);

										int8 X = storedPosX * LEADER_POS_SCALER;
										int8 Y = storedPosY * LEADER_POS_SCALER;
										int32 sinAngle = sinMilliRad(smallestAngle);
										int32 cosAngle = cosMilliRad(smallestAngle);
										int32 cosBearing = cosMilliRad(nbrBearing);
										int32 sinBearing = sinMilliRad(nbrBearing);

										// int32 leaderPosXNbr1 = (nbrDist * cosBearing) / (MILLIRAD_TRIG_SCALER);
										// int32 leaderPosXNbr2 = (X * cosAngle / (MILLIRAD_TRIG_SCALER));
										// int32 leaderPosXNbr3 = -(Y * sinAngle / (MILLIRAD_TRIG_SCALER));
										//  return leaderPosXNbr1 + leaderPosXNbr2 + leaderPosXNbr3;
										leaderPosX = ((nbrDist * cosBearing) + (X * cosAngle) - (Y * sinAngle)) / MILLIRAD_TRIG_SCALER;

										leaderPosY = ((nbrDist * sinBearing) + (X * sinAngle) + (Y * cosAngle)) / MILLIRAD_TRIG_SCALER;

										int32 rangeBit = nbrGetRangeBits(nbrPtr);
										//rprintf(" %d \n", rangeBit);

										leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
										leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);

										//int32 nbrBearing = nbrGetBearing(nbrPtr);
										//int32 nbrOrientation = nbrGetOrientation(nbrPtr);
										//int16 smallestAngle = normalizeAngleMilliRad2((int16)(MILLIRAD_PI + nbrOrientation - nbrBearing));
										//int16 leaderDist = (nbrPtr->range)/2;
										//   if(printNow)
										//	rprintf("  %d, %d, %d, %d, %d, %d \n", leaderPosX, leaderPosY,leaderDist, nbrBearing, nbrOrientation, smallestAngle);

										//  actionMode = ROT_MODE;
										//  rotTime = ROT_MAX;

										leaderBearing = normalizeAngleMilliRad2(atan2MilliRad(leaderPosY, leaderPosX));
										leaderDist = sqrtInt((uint32)(leaderPosX * leaderPosX + leaderPosY * leaderPosY));

										//leaderDist = 300;

										break;
									}

								}
							}
						}

					}
//else
//{

//if (rotTime == 0)
					// {
					//  actionMode = MODE_ESTIMATE;

					//  }
//rotTime--;
					actionMode = MODE_ESTIMATE;
					;

					//  if( (rotationState == COM_STATE && rotateCountDown == ROTATE_MAX)|| (rotationState != COM_STATE))

					//  {
					translatetime = 0;
				//	if (printNow)
					//{
						rotateTime = rotateTime + 1;
					//}
//newData = 0;
					if (broadcastMsgIsSource(&broadcastMessage) && rotationState != COM_STATE)
					{
						//	ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SNAIL);
						//behOutput = behInactive;
						//} else {
						ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
						behOutput = behInactive;
						broadcastMsgDataSet(&broadcastData_Rotation, rotation);
						if (rotateTime >= 0) {
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

						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);

						tv = 0;
						rv = 0;
						tvgain = 0;
						behOutput.active = TRUE;

						if (valrot == 1)
							alpha = normalizeAngleMilliRad2((int16)(-1 * MILLIRAD_DEG_90 + leaderBearing));
						else if (valrot == 2)
							alpha = normalizeAngleMilliRad2((int16)(MILLIRAD_DEG_90 + leaderBearing));
						else
							alpha = 0;

						//rprintf("%d", alpha);

						//uint32 leaderDist =dist/80 ;
						//alpha = MILLIRAD_DEG_30;
						//int32 FLOCK_RV_GAIN = 50;
						//rv = alpha * FLOCK_RV_GAIN / 100;
						int32 error = alpha / 100;
						flockAngleIntegral2 = flockAngleIntegral2 * FLOCK_INTEGRAL_DECAY / 1000;
						flockAngleIntegral2 += 1 * (error) / 100 * (error > 0 ? 1 : -1);
						flockAngleIntegral2 = bound(flockAngleIntegral2, -FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
						tvgain = 100 - (abs(flockAngleIntegral2) * 100 / FLOCK_INTEGRAL_MAX);

						int32 rv_I = 0 * flockAngleIntegral2 * K_I / 100; //WEIGHTED TEST WORKS WITH 2
						int32 rv_P = K_P_ROTATION * error; //WEIGHTED TEST WORKS WITH 2

						newRV = (rv_I + rv_P);
						if (abs(newRV) <= ERROR_MAX) {
							rv = 0;

							behOutput.rv = rv;
						} else {
							rv = newRV;
							behOutput.rv = rv;
						}
						//tv_gain = 1;

						if (rotateTime < DELAY_ROTATE)
						{
							tv = 0;
							behOutput.tv = tv;

						} else {
							//behOutput.rv = rv;
							if (rotateTime >= DELAY_ROTATE ) {
								tv = 100 * (DESIRED_TV_N * leaderDist) / (100 * DESIRED_TV_D); // set tv based on the distance to the leader
								behOutput.tv = tv;
								// cprintf("mode % 4d  rotateTime  % 4d  tv %4d  dist %4d \n",	val, rotateTime,tv, leaderDist);  // commented for printing radio message

							} else {
								tv = 0;
								rv = 0;
								behOutput.tv = tv;

								behOutput.rv = rv;

								if (broadcastMsgIsSource(&broadcastMessage)) {

									broadcastMsgDataSet(&broadcastData_Mode, MODE_LEADERSTOP);
									tv = 0;
									rv = 0;
									behOutput.tv = tv;

									behOutput.rv = rv;

								}

							}

						}

					}
// }
				}
				//behOutput.tv = 0;
//			behOutput.rv = 0;
				break;
			}
			} //end mode switch
			  //motorSetBeh(&behOutput);

			behOutput.tv = 0;
			behOutput.rv = 0;

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
	osTaskCreate(backgroundTask, "background", 1536, NULL, BACKGROUND_TASK_PRIORITY); // commented for testing radio message

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
