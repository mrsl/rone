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
#define TV								40

#define MODE_INACTIVE						0
#define MODE_TRANSLATE						1
#define MODE_ROTATE							2
#define MODE_LEADERSTOP						3
#define MODE_LEADER							4

#define UNSAFE			 0
#define MINORSAFE		 1
#define SAFE			 2
#define TRANSPORT		 3

#define K						     	5
#define SPEED					    	60
#define TV_MAX					    	100
#define RV_MAX					    	4000

#define TV_GAIN				        	1

#define FLOCK_TV					   10

#define FLOCK_INTEGRAL_MAX		       3000
#define FLOCK_INTEGRAL_DECAY	       970
#define FLOCK_INTEGRAL_DECAY2	4

#define K_INTEGRAL_MAX				1000 //400 works, 300 doesn't
#define DECAY_INTEGRAL              70

#define OBJECT_MASS					778
#define K_PI_N						100
#define K_PI_D					    100

#define K_I				            10
#define K_P			            	5000
#define K_D				            75


#define LEADER_POS_SCALER			10
#define LEADER_POS_BOUND			(127 * LEADER_POS_SCALER)

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

	NbrData msgLeaderPosX;
	NbrData msgLeaderPosY;
	NbrData msgNbrType;
	NbrDataFloat msgWeight;

	Nbr* nbrPtr;
	Nbr* leaderPtr = NULL;
	Nbr* safestGuidePtr = NULL;
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

	nbrDataCreate(&msgLeaderPosX, "positionx", 8, 0);
	nbrDataCreate(&msgLeaderPosY, "positiony", 8, 0);
	nbrDataCreate(&msgNbrType, "type", 2, TRANSPORT);
	nbrDataCreateFloat(&msgWeight, "weight");

	broadcastMsgCreate(&broadcastMessage, 20);
	broadcastMsgDataCreate(&broadcastData_Mode, &broadcastMessage, "mode",MODE_INACTIVE);
	broadcastMsgDataCreate(&broadcastData_DesiredHeading, &broadcastMessage,"heading", 0);
	uint32 senderID_seen = 0;

	irCommsSetXmitPower(IRXmitPower);
	systemPrintStartup();
	uint8 state = 0;
	int32 t = 0;
	int16 leaderBearing = 0;
	uint32 leaderDist = 0;
	/******** Behavior **************/
	for (;;) {
		behOutput = behInactive;
		neighborsGetMutex();
		printNow = neighborsNewRoundCheck(&neighborRound);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);

		int32 senderID = broadcastMsgGetSenderID(&broadcastMessage);
		/**** Calculate Position to All Neighbors ****/
		for (t = 0; t < nbrList.size; ++t) {
			nbrPtr = nbrList.nbrs[t];
			if (nbrPtr->ID == senderID) {
				senderID_seen = 1;
				nbrBearing = nbrGetBearing(nbrPtr);
				nbrDist = nbrGetRange(nbrPtr);
				int32 l1 = ((int8)nbrDataGet(&msgLeaderPosX)) * LEADER_POS_SCALER;
				int32 l2 = ((int8)nbrDataGet(&msgLeaderPosY)) * LEADER_POS_SCALER;

				leaderPosX = l1 + (int32)(nbrDist * cosMilliRad(nbrBearing) / MILLIRAD_TRIG_SCALER);
				leaderPosY = l2 + (int32)(nbrDist * sinMilliRad(nbrBearing) / MILLIRAD_TRIG_SCALER);
				leaderPosX = boundAbs(leaderPosX, LEADER_POS_BOUND);
				leaderPosY = boundAbs(leaderPosY, LEADER_POS_BOUND);

				nbrDataSet(&msgLeaderPosX, (int8)(leaderPosX / LEADER_POS_SCALER));
				nbrDataSet(&msgLeaderPosY, (int8)(leaderPosY / LEADER_POS_SCALER));

				int32 dist = leaderPosX * leaderPosX + leaderPosY * leaderPosY;

				leaderBearing = atan2MilliRad(leaderPosY, leaderPosX);
				leaderDist = sqrtInt((uint32) dist);
			}
		}

		/**** Check Button Input ****/
		//Check Buttons, won't start until leader selected
		if (buttonsGet(BUTTON_RED)) {
			//Sets leader
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_INACTIVE);
			state = MODE_LEADER;
			translatetime = 0;
			rotateTime = 0;
		} else if (buttonsGet(BUTTON_GREEN)) {
			//Starts navigation mode
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_ROTATE);
			translatetime = 0;
			rotateTime = 0;
			state = MODE_ROTATE;

		} else if (buttonsGet(BUTTON_BLUE)) {
			//Stops all modes
			broadcastMsgSetSource(&broadcastMessage, TRUE);
			broadcastMsgDataSet(&broadcastData_Mode, MODE_LEADERSTOP);
			translatetime = 0;
			rotateTime = 0;
			state = MODE_LEADERSTOP;
		}

		/*** Debugging Prints ***/
		if (printNow){
			cprintf("hops, mode = %d,%d\n",broadcastMsgGetHops(&broadcastMessage),broadcastMsgDataGet(&broadcastData_Mode));
		}

		if (printNow) {
			cprintf("sender: %d\n", broadcastMsgGetSenderID(&broadcastMessage));
		} else {
			state = broadcastMsgDataGet(&broadcastData_Mode);
			cprintf("val: %d\n", state);
			broadcastMsgDataSet(&broadcastData_Mode, state);
		}

		/*** Determine Action ***/
		switch (state) {
		case MODE_LEADER: {
			//Leader flashes all LEDS
			ledsSetPattern(LED_ALL,LED_PATTERN_BLINK,LED_BRIGHTNESS_MED,LED_RATE_MED);

			/*** Select Safest Guide ***/
			Nbr* newSafestPtr = NULL;
			NbrDataFloat msgWeight;
			uint8 nbrType = UNSAFE;
			for(t = 0; t < nbrList.size; ++t){
				nbrPtr = nbrList.nbrs[t];
				nbrType = nbrDataGetNbr(&msgNbrType,nbrPtr);
				if(nbrType == TRANSPORT){
					//Ignore transport neighbors for choosing parent robot
					continue;
				} else {
					//Find minimum cost parent
					if(newSafestPtr != NULL){
						//If we have a guide selected, compare and determine if new should be selected
						if(nbrDataGetNbrFloat(&msgWeight,nbrPtr) < nbrDataGetNbrFloat(&msgWeight,newSafestPtr)){
							newSafestPtr = nbrPtr;
						}
					}
				}
			}
			if(newSafestPtr != NULL){
				//if we have a new safest guide with low weight, compare to current and switch or not
				if(newSafestPtr->ID != safestGuidePtr->ID){
					//switch guides
					safestGuidePtr = newSafestPtr;
				}
			}

			/*** Tell Others What to Do ***/

			break;
		}
		case MODE_INACTIVE: {
			behOutput = behInactive;
			//cprintf("inactive \n");
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED,LED_RATE_MED);
			break;
		}
		case MODE_LEADERSTOP: {
			behOutput = behInactive;
			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED,LED_RATE_MED);
			break;
		}
		case MODE_TRANSLATE: {
			translatetime = translatetime + 1;

			if (broadcastMsgIsSource(&broadcastMessage)) {
				//Pulsing Green LEDS indicate waiting to translate
				//Circling Green LEDS indicate translation has begun
				if (translatetime <= 100) {
					behSetTvRv(&behOutput, 0, 0);
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,LED_BRIGHTNESS_LOW, LED_RATE_MED);
				} else {
					behSetTvRv(&behOutput, TV, 0);
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,LED_BRIGHTNESS_LOW, LED_RATE_MED);
				}
			} else {
				int32 tvgain, error;

				alpha = behFlockAngle(&nbrList);

				error = alpha / 100;
				flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY / 1000;
				flockAngleIntegral += (error * error) * (error > 0 ? 1 : -1);
				flockAngleIntegral = bound(flockAngleIntegral,-FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
				tvgain = 100 - (abs(flockAngleIntegral) * 100) / FLOCK_INTEGRAL_MAX;

				int32 rv_I = flockAngleIntegral * K_I / 100; //WEIGHTED TEST WORKS WITH 2
				int32 rv_P = error * K_P / 100; //WEIGHTED TEST WORKS WITH 2

				if (printNow) {
					cprintf(
							"error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",
							alpha, flockAngleIntegral, rv_I, rv_P);
				}
				behOutput.rv = (rv_I + rv_P);
				behOutput.active = TRUE;

				if (translatetime <= 100) {
					//If waiting to start translating, pulse green LEDS
					behOutput.tv = 0;
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,LED_BRIGHTNESS_LOW, LED_RATE_MED);
				} else {
					//If translating, circling green LEDS
					behOutput.tv = 2 * FLOCK_TV * tvgain / 100;
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,LED_BRIGHTNESS_LOW, LED_RATE_MED);
				}
			}

			break;
		}
		case MODE_ROTATE: {
			rotateTime = rotateTime + 1;
			if (broadcastMsgIsSource(&broadcastMessage)) {
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED,LED_RATE_MED);
				behOutput = behInactive;
				nbrDataSet(&msgLeaderPosX, 0);
				nbrDataSet(&msgLeaderPosY, 0);

			} else {
				uint16 speed = LED_RATE_MED;
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED,LED_RATE_MED);
				int32 tv_gain, alpha;
				tv = 0;
				rv = 0;
				tv_gain = 0;
				behOutput.active = TRUE;
				if (leaderBearing > 0)
					alpha = normalizeAngleMilliRad2(leaderBearing - (int32) MILLIRAD_DEG_90);
				else
					alpha = normalizeAngleMilliRad2(leaderBearing + (int32) MILLIRAD_DEG_90);
				int32 FLOCK_RV_GAIN = 50;
				rv = alpha * FLOCK_RV_GAIN / 100;
				tv_gain = 1;
				tv = tv_gain * leaderDist / 20;
				if (senderID_seen == 1){
					behOutput.rv = rv;
					behOutput.tv = tv;
				} else {
					behOutput.rv = 0;
					behOutput.tv = 0;
				}
			}
			break;
		}
		} //end mode switch

		//Set behavior and finish this task period
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
