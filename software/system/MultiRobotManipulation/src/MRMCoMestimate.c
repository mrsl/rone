/*
 * MRMCoMestimate.c
 *
 *  Created on: Dec 20, 2013
 *      Author: Golnaz Habibi
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nbrDataCoM.h"

#include "roneos.h"
#include "ronelib.h"


typedef enum { false, true } bool;

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
#define MAX_ACK_WAIT                    5
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
#define DESIRED_RV_N			    15
#define DESIRED_RV_D			   	100
#define COM_STATE					1
#define LEARDER_STATE				2

#define STARTUP_STATE				0
#define IDLE_STATE					1
#define REQUEST_STATE				2
#define REQUEST_PROBABILITY			3
#define ACK_PROBABILITY				3

#define START_ROUND					10
#define IDLE_ROUND_MAX  			20
#define ACK_ROUND_MAX				40
#define REQUEST_ROUND_MAX			70 //70
#define WAIT_ROUND_MAX		     	71 //always REQUEST_ROUND_MAX + 1
#define NONCE_MAX		     	   100
#define RANGE_IIR                   60
#define LEADER_POS_SCALER			1.0
#define LEADER_POS_BOUND			(127 * LEADER_POS_SCALER )

/* User-defined functions ***************+**************************/
void backgroundTask(void* parameters);
void behaviorTask(void* parameters);
float updatePosX(Nbr* nbrPtr,	float CoMStoredPosX,  float CoMStoredPosY, float COMPosX);
float updatePosY(Nbr* nbrPtr,	float CoMStoredPosX,  float CoMStoredPosY, float COMPosY);

/* global variables ***********************************************/
boolean printNow = FALSE;

// the background task runs all the time.  Put slow stuff here, like compute intensive functions
// or large data transfers, like getting the Yun's map from the robot.

float updatePosX(Nbr* nbrPtr,  float CoMStoredPosX, float CoMStoredPosY,float COMPosX) {

	int32 nbrBearing = nbrGetBearing(nbrPtr);
	int32 nbrOrientation = nbrGetOrientation(nbrPtr);
	int32 smallestAngle = MILLIRAD_PI + nbrOrientation - nbrBearing;
	//int32 nbrDist = nbrGetRange(nbrPtr);
	int16 range = irCommsComputeNbrRange(nbrPtr->rangeBits);
	nbrPtr->range = filterIIR(nbrPtr->range, range, RANGE_IIR);
	int32 nbrDist = nbrPtr->range/2;
	//rprintf(" %d \n",nbrDist);

	//nbrDist = 120;
	//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );

	float X =  CoMStoredPosX * LEADER_POS_SCALER;
	float Y =  CoMStoredPosY * LEADER_POS_SCALER;
	int32 sinAngle =  sinMilliRad(smallestAngle);
	int32 cosAngle =  cosMilliRad(smallestAngle);
	int32 cosBearing =  cosMilliRad(nbrBearing);

	int32 leaderPosXNbr = (nbrDist * cosBearing)/(MILLIRAD_TRIG_SCALER) + (X * cosAngle/(MILLIRAD_TRIG_SCALER) )  - (Y* sinAngle/(MILLIRAD_TRIG_SCALER)) ;
	float PosX = (COMPosX + (float)leaderPosXNbr) / 2.0;
	return PosX;

}

float updatePosY(Nbr* nbrPtr,  float CoMStoredPosX, float CoMStoredPosY,float COMPosY) {

	int32 nbrBearing = nbrGetBearing(nbrPtr);
	int32 nbrOrientation = nbrGetOrientation(nbrPtr);
	int32 smallestAngle = MILLIRAD_PI + nbrOrientation - nbrBearing;
	//int32 nbrDist = nbrGetRange(nbrPtr);
	int16 range = irCommsComputeNbrRange(nbrPtr->rangeBits);
	nbrPtr->range = filterIIR(nbrPtr->range, range, RANGE_IIR);
	int32 nbrDist = nbrPtr->range/2;
	//rprintf("  %d, %d \n", nbrBearing, nbrOrientation  );
	//nbrDist = 120;
	//rprintf(" %d \n",nbrDist);
	float X =  CoMStoredPosX * LEADER_POS_SCALER;
	float Y =  CoMStoredPosY * LEADER_POS_SCALER;
	int32 sinAngle =  sinMilliRad(smallestAngle);
	int32 cosAngle =  cosMilliRad(smallestAngle);
	int32 sinBearing =  sinMilliRad(nbrBearing);
	int32 leaderPosYNbr = (nbrDist * sinBearing)/(MILLIRAD_TRIG_SCALER) + (X * sinAngle/(MILLIRAD_TRIG_SCALER) )  - (Y* cosAngle/(MILLIRAD_TRIG_SCALER)) ;
	float PosY = (COMPosY + (float)leaderPosYNbr) / 2.0;
	return PosY;

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
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound = 0;
	uint8 i = 0;
	uint16 IRXmitPower = IR_COMMS_POWER_MAX;

	int32 stateval = 0;
	int32 ignoreRequestTimer = 0;
	int32 reqRound = 0;
	int32 waitRound = 0;
	int32 startRound = START_ROUND;
	Nbr* requestnbr;

	int32 req_prob = 0;


	NbrDataCoM msgDataCoM;

	Nbr* nbrPtr;
	NbrList nbrList;

	int32 nbrBearing = 0;
	int32 nbrDist = 0;
	float CoMPosX = 20.0;
	float CoMPosY = 30.0;


	int32 state = COM_STATE; //0: pivot, 1: CoM
	int32 tvgain = 0;
	int32 error = 0;

	float CoMNbrRequestTempStorageX = 0.0;
	float CoMNbrRequestTempStorageY = 0.0;

	/******** Initializations ********/
	radioCommandSetSubnet(2);

	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	nbrDataCoMCreate(&msgDataCoM, "SharedData");

	systemPrintStartup();

	//uint8 tt_val = TT_MAX;

	int32 t = 0;
	int16 leaderBearing = 0;
	uint32 leaderDist = 0;
	float X = 0;
	float Y = 0;
	int32 RotateTime = 5000; // 141 sec for one complete circle rotation ~ 1880
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
	char tempstrFloat[16];

int32 roundAlg = 0;
	//int32 TransTime = translateperiod[picknumber];
	/******** Behavior **************/




	for (;;) {

		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		} //end if host
		else {
			roundAlg++;
			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);
			//cprintf("print %d \n",printNow);
			//TODO yo only need to call this function once
			irCommsSetXmitPower(IRXmitPower);
			nbrListCreate(&nbrList);

			if (printNow)
			{	sprintf(tempstrFloat, "%.2f, %.2f",  CoMPosX,  CoMPosY);
				rprintf(" %s \n",tempstrFloat);
				//printf("pos: %.2f, %.2f \n",  CoMPosX,  CoMPosY);
			}
			/////////////////// CoM Estimation /////////////////////////////////////////
			// Go through all neighbors, check if any of them selects the robot for sharing CoM Data
			nbrDataCoMSetPos(&msgDataCoM,
					/*(float)*/(CoMPosX / LEADER_POS_SCALER),
					/*(float)*/(CoMPosY  / LEADER_POS_SCALER));
			//nbrDataCoMSetPos(&msgDataCoM, leaderPosX, leaderPosY);


			switch (stateval) {

			case STARTUP_STATE: {
				nbrsize = nbrListGetSize(&nbrList) ;

				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				if (startRound > 0) {
					startRound--;
					}
				else
					{
					stateval = IDLE_STATE;

					}

				break;

			}

			case IDLE_STATE: {
				//cprintf("inactive \n");
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataCoMSetReqID(&msgDataCoM, ROBOT_ID_NULL);

				waitRound = 0;
				reqRound = 0;
				
				idleRound++;
				nbrsize = nbrListGetSize(&nbrList) ;
				if (nbrsize > 0) {

				if (ignoreRequestTimer == 0) {
					// timer is zero.  We can check for new acknoweledgements


						for (t = 0; t < nbrsize; ++t) {
							randomNbr = rand() % (nbrsize);
							nbrPtr = nbrListGetNbr(&nbrList, randomNbr);
							if ((nbrDataCoMGetReqID(&msgDataCoM, nbrPtr) == roneID ) &&
								!((nbrDataCoMGetNonce(&msgDataCoM, nbrPtr) == requestNoncePrev) &&
								  (nbrGetID(nbrPtr) == requestNbrIDPrev)))  {
								// copmute new CoM and Ack
								requestNbrIDPrev = nbrGetID(nbrPtr);
								requestNoncePrev = nbrDataCoMGetNonce(&msgDataCoM, nbrPtr);
								CoMNbrRequestTempStorageX = nbrDataCoMGetx(&msgDataCoM, nbrPtr);
								CoMNbrRequestTempStorageY = nbrDataCoMGety(&msgDataCoM, nbrPtr);
								//CoMPosX = updatePosX(nbrPtr, (float)CoMNbrRequestTempStorageX, (float)CoMNbrRequestTempStorageY, CoMPosX);
								//CoMPosY = updatePosY(nbrPtr, (float)CoMNbrRequestTempStorageX, (float)CoMNbrRequestTempStorageY, CoMPosY);
								//sprintf(tempstrFloat, "%.2f, %.2f , %.2f", CoMNbrRequestTempStorageX , CoMNbrRequestTempStorageY , 555.0 );
								//rprintf(" %s \n",tempstrFloat);

								CoMPosX= ((CoMNbrRequestTempStorageX * LEADER_POS_SCALER) + CoMPosX)/2.0;
								CoMPosY= ((CoMNbrRequestTempStorageY * LEADER_POS_SCALER) + CoMPosY)/2.0;

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
							if ((rand() % 11 - 1) >= REQUEST_PROBABILITY) {
								randomNbr = rand() % (nbrListGetSize(&nbrList));
								nbrPtr = nbrListGetNbr(&nbrList, randomNbr);
								nbrDataCoMSetReqID(&msgDataCoM, nbrGetID(nbrPtr));
								nbrDataCoMSetAckID(&msgDataCoM, ROBOT_ID_NULL);

								// store current CoM values of neighbor
								CoMNbrRequestTempStorageX = nbrDataCoMGetx(&msgDataCoM, nbrPtr);
								CoMNbrRequestTempStorageY = nbrDataCoMGety(&msgDataCoM, nbrPtr);
								nonce++;
								nbrDataCoMSetNonce(&msgDataCoM, nonce);
								reqRound = REQUEST_ROUND_MAX;
								stateval = REQUEST_STATE;
							}
						}
					}
				} else {

					ignoreRequestTimer--;

				}

			}
				break;
			}
			case REQUEST_STATE: {
				nbrsize = nbrListGetSize(&nbrList) ;

				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				if (reqRound > 0) {
					reqRound--;


					for (t = 0; t < nbrList.size; ++t) {
						nbrPtr = nbrListGetNbr(&nbrList, t);
						if (nbrDataCoMGetAckID(&msgDataCoM, nbrPtr) == roneID	&& nbrDataCoMGetNonce(&msgDataCoM, nbrPtr)	== nonce) {
							//CoMPosX = updatePosX(nbrPtr, (float)CoMNbrRequestTempStorageX, (float)CoMNbrRequestTempStorageY, CoMPosX);
							//CoMPosY = updatePosY(nbrPtr, (float)CoMNbrRequestTempStorageX, (float)CoMNbrRequestTempStorageY, CoMPosY);


							CoMNbrRequestTempStorageX = nbrDataCoMGetx(&msgDataCoM, nbrPtr);
							CoMNbrRequestTempStorageY = nbrDataCoMGety(&msgDataCoM, nbrPtr);

							//CoMPosX = updatePosX(nbrPtr, (float)CoMNbrRequestTempStorageX, (float)CoMNbrRequestTempStorageY, CoMPosX);
							//CoMPosY = updatePosY(nbrPtr, (float)CoMNbrRequestTempStorageX, (float)CoMNbrRequestTempStorageY, CoMPosY);

							///////// Debug
							CoMPosX = CoMNbrRequestTempStorageX * LEADER_POS_SCALER;
							CoMPosY = CoMNbrRequestTempStorageY * LEADER_POS_SCALER;
							//sprintf(tempstrFloat, "%.2f, %.2f , %.2f", CoMNbrRequestTempStorageX , CoMNbrRequestTempStorageY , 432.0 );
							//rprintf(" %s \n",tempstrFloat);
							//CoMPosX= ((CoMNbrRequestTempStorageX * LEADER_POS_SCALER) + CoMPosX)/2.0;
							//CoMPosY= ((CoMNbrRequestTempStorageY * LEADER_POS_SCALER) + CoMPosY)/2.0;
							//sprintf(tempstrFloat, "%.2f, %.2f , %.2f", CoMPosX , CoMPosY , 123.0 );
							//rprintf(" %s \n",tempstrFloat);



							stateval = IDLE_STATE;
							nbrDataCoMSetReqID(&msgDataCoM, ROBOT_ID_NULL);

							break;

						}
					}
				}
				else
				{
						fail++;
						stateval = IDLE_STATE;
				}

				break;

			}


			} //end mode switch
			  //motorSetBeh(&behOutput);

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
