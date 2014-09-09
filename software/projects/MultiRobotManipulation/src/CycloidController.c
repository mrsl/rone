/*
 * CycloidController.c
 *
 *  Created on: Aug 25, 2014
 *      Author: Golnaz
 */


#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"
#include "globalTreeCOM.h"


#define NEIGHBOR_ROUND_PERIOD			600
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50
#define NAV_TOWER						20
#define PI								3147
#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50
#define TV_MIN							15
#define ACCEL_SCALE						5
#define NAV_TOWER_DISTANCE				1000

#define FLOCK_RV_GAIN_MOVEOBJ			70
#define	RADIUS							250
#define nbrEdgeDis						250				//hardcoded distance

#define	TOWER_WAIT_TIME					200
#define	cyldoidSpeed					150

#define CYCLOID_ROTATION_MOD			0				//Modifies translation speed vs rotational speed in cyliod motion
														//From 0 to 50, 0 for more translation, 50 for more rotation

#define OMEGA_ROT			3
#define OMEGA_TRANS			1

#define BUILD_TREE		 0
#define GUESS_COM		 1
#define REMOTE			 2

#define REST			0
#define CNTCLK			1
#define CLKWISE			2
#define ATTEMPTING		3

RadioMessage radioMessageTX;
RadioMessage radioMessageRX;
RadioCmd radioCmdRemoteControl;
char* name = "RCwifi";

void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 state = BUILD_TREE;
	Beh behOutput = behInactive;
	boolean printNow;
	uint32 neighborRound = 0;
	NbrList nbrList;
	uint8 moveState = 0;
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);
	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);
	int16 COM_Y,COM_X,PIVOT_X,PIVOT_Y, cylciodModifier;
	uint8 cycloidTime = 1;

	PositionCOM treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+2]; // why +2?
	GlobalTreeCOMListCreate(treeGuessCOM);

	NbrData LeaderHeading_H;
	NbrData LeaderHeading_L;
	nbrDataCreate16(&LeaderHeading_H,&LeaderHeading_L,"LeaderHeading_H", "LeaderHeading_L", 0);


	//uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;
	GlobalRobotList globalRobotList;
	globalRobotListCreate(&globalRobotList);

	gripperBoardInit();
	uint8 gripPos = REST;

	///Radio Init
	uint32 radioMessageTimePrev = 0;
	int TVcmd, RVcmd = 0;
	int comRed = 0;
	int comBlue = 0;
	int comGreen = 0;
	int comMoveState = 0;
	boolean bounceBlue = 0;
	boolean bounceRed = 0;
	boolean bounceGreen = 0;
	int32 accX = 0;
	int32 accY = 0;
	radioCommandAddQueue(&radioCmdRemoteControl,name, 1);
	navigationData navData;

	//Cylciod Stuff

	//Nav Tower
	Nbr* nbrNavTowerPtr;
	uint32 navTowerTime;
	int16 vecCOMtoTowerY, vecCOMtoTowerX, vecCOMtoTowerBearing;
	scaleCoordinate GRLcentroidCooridates[GLOBAL_ROBOTLIST_MAX_SIZE + 2];
	createGRLscaleCoordinates(GRLcentroidCooridates);
	Pose rotateWaypointPose,translateWaypointPose,goalWaypointPose, CurrPose;


	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		}else{
			/*** INIT STUFF ***/
			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);
			//irCommsSetXmitPower(IRXmitPower);
			nbrListCreate(&nbrList);
			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			globalRobotListUpdate(&globalRobotList, &nbrList);

			if(printNow){
				//globalRobotListPrintAllTree(&globalRobotListPtr, &nbrList);
			}

			/*** READ BUTTONS ***/
			if (!buttonsGet(BUTTON_BLUE)) {
				bounceBlue = 1;
			}
			if (!buttonsGet(BUTTON_RED)) {
				bounceRed = 1;
			}
			if (!buttonsGet(BUTTON_GREEN)) {
				bounceGreen = 1;
			}


			if(state!= REMOTE){
				if (buttonsGet(BUTTON_RED)) {
					state = REMOTE;
				}else if (buttonsGet(BUTTON_GREEN) && bounceGreen) {
					comMoveState++;
					if(comMoveState >= 4){
						comMoveState = 0;
					}
					moveState = comMoveState;
					bounceGreen = 0;
				}else if (buttonsGet(BUTTON_BLUE)) {
					state = GUESS_COM;
				}
			}

			/** STATES MACHINE **/
			switch (state) {
			case BUILD_TREE:{
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				if((globalRobotList.size >= GLOBAL_ROBOTLIST_MAX_SIZE) || (buttonsGet(BUTTON_BLUE))){
					state = GUESS_COM;
				}

				break;
			}
			case GUESS_COM:{
				switch (moveState){
				case 0:{
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					break;
				}
				case 1:{
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					break;
				}
				case 2:{
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					break;
				}
				case 3:{
					ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					break;
				}
				}
				break;
			}
			case REMOTE:{
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			}


			if(!gripperBoardGetGripped()){
				if(gripPos != ATTEMPTING){
					gripperGripUntilGripped();
					gripPos = ATTEMPTING;
				}
			}else{
				gripPos = CLKWISE;
				if(gripperBoardGetServo() > 100){
					gripPos = CNTCLK;
				}
			}


			if( radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX,0) ) {
				//Grabing stream and paresing
				char* RXmsg = radioCommandGetDataPtr(&radioMessageRX);
				radioMessageTimePrev = osTaskGetTickCount();
				sscanf(RXmsg,"%d,%d,%d,%d,%d,%d", &TVcmd, &RVcmd, &comBlue, &comRed,&comGreen, &comMoveState); //parse the speed and turning rate
				moveState = comMoveState;
				if(comBlue && state == BUILD_TREE){
					state = GUESS_COM;
				}

			} else {
				// no message this time.  see if you can just run the last command, or if it has timed out
				if (osTaskGetTickCount() > (radioMessageTimePrev + RADIO_MESSAGE_PERSISTANCE)) {
					// the previous message is too old.  clear the behRadio
					TVcmd = 0;
					RVcmd = 0;
					comBlue = 0;
					comRed = 0;
					comGreen = 0;
				}
			}

			if(state == GUESS_COM){
				nbrNavTowerPtr = nbrListGetNbrWithID(&nbrList, NAV_TOWER);
				int16 translateBearing = 0;
				if(nbrNavTowerPtr) {
					int16 x,y;
					x = 0;
					y = NAV_TOWER_DISTANCE;  // G: if robot sees the leader,
					translateBearing = nbrNavTowerPtr->bearing;

					//G: rotation matrix , converts to its own coordinate, is it correct?


					// to convert from (x2, y2) to (x1,y1) : M * [x2-x1 ; y2 -y1],
					//M  is the rotational matrix : M = [cos (theta)  -sin (theta); sin(theta cos(theta)], theta is the angle from (x1,y1) to (x2,y2)
					// theta = bearing + pi - orientation,  bearing and orientation is for the robot in (x1,y1)

					// First: find the vector from centroid to nav tower robot : each robot knows the centroid, sees the nav tower(?)
					// second: find the normal vector of robot - centroid




					vecCOMtoTowerX = x*cosMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER;
					vecCOMtoTowerY = x*sinMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER;

					//rprintf("COMtoTOWER X %d Y %d B %d CM %d\n",COMtoTower   X,COMtoTowerY, atan2MilliRad((int32)COMtoTowerX,(int32)COMtoTowerY), cylciodModifier);
					nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_L,vecCOMtoTowerX);
					nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_L,vecCOMtoTowerY);
					//Todo: Assumes that nave tower is highest ID that will be seen, need to be changed if v15 are used
					nbrList.size--;
					navTowerTime = osTaskGetTickCount();
				}else{
					vecCOMtoTowerX = 0;
					vecCOMtoTowerY = 0;
					Nbr* nbrPtr;
					int i;
					for (i = 0; i < nbrList.size; i++){
						nbrPtr = nbrList.nbrs[i];
						vecCOMtoTowerY = nbrDataGetNbr16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,nbrPtr);
						vecCOMtoTowerX = nbrDataGetNbr16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,nbrPtr);
						if(vecCOMtoTowerX || vecCOMtoTowerY){
							int16 x,y,xprime,yprime;
							int32 nbrOrient = nbrGetOrientation(nbrPtr);
							int32 nbrBear = nbrGetBearing(nbrPtr);

							x = vecCOMtoTowerX;
							y = vecCOMtoTowerY;

							xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
							yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;

							x = xprime;
							y = yprime + nbrEdgeDis;

							vecCOMtoTowerX = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
							vecCOMtoTowerY = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;

							nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_L,vecCOMtoTowerX);
							nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_L,vecCOMtoTowerY);
					  		navTowerTime = osTaskGetTickCount();
							break;
						}
					}

					nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_L,0);
					nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_L,0);
				}


				globalRobotListUpdate(&globalRobotList, &nbrList);
				centroidGRLUpdate(&navData, globalRobotList, &nbrList, GRLcentroidCooridates);


				//rprintf("%d %d\n",COM_X,COM_Y);
				if(moveState == 0){				//Translate
					behFlock_gain(&behOutput, &nbrList, TVcmd, FLOCK_RV_GAIN_MOVEOBJ);
					behOutput.rv  = behOutput.rv + (RVcmd*10);
				}else if(moveState == 1){		//Rotate around COM
					behOutput = behInactive;
					GlobalTreePointOrbit(COM_X, COM_Y, &behOutput,  RVcmd);
				}else if(moveState == 2){		//Pivot
					PIVOT_X =  nbrDataGet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L);
					PIVOT_Y =  nbrDataGet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_L);
					GlobalTreePointOrbit(PIVOT_X, PIVOT_Y, &behOutput,  RVcmd);
				}else if(moveState == 3){		// Cycloid Motion

					if(!translateBearing){  // robot does not see the guide robot

						translateBearing = (atan2MilliRad((int32)vecCOMtoTowerX,(int32)vecCOMtoTowerY) - PI)/2;
					}

					int16 rotatBearing = atan2MilliRad((int32)COM_Y,(int32)COM_X) - PI;
					int16 goalBearing;
					if(!rotatBearing || !OMEGA_ROT){
						goalBearing = translateBearing;
					}else if(!translateBearing || !OMEGA_TRANS){
						goalBearing = rotatBearing;
					}else{
						goalBearing = (OMEGA_ROT* rotatBearing + OMEGA_TRANS * translateBearing) / (OMEGA_ROT + OMEGA_TRANS);
					}


					//int32 distance = vectorMag((int32)COM_Y,(int32)COM_X);
					//int32 TV = cyldoidSpeed * distance / 100;
					int32 TV = 60 ; //  150 - 150* abs(goalBearing) / (PI/2);
					if(TV < 0){
						TV = 0;
					}
					if(abs(goalBearing) > 100){
						if(goalBearing < 0){
							behSetTvRv(&behOutput, TV/2, goalBearing/ 1);
						} else{
							behSetTvRv(&behOutput, TV/2, goalBearing/ 1);
						}
					}else{
						behSetTvRv(&behOutput, TV, 0);
					}
					rprintf("%d %d %d\n", translateBearing,rotatBearing, goalBearing );

				}

			}


			/*** FINAL STUFF ***/
			motorSetBeh(&behOutput);
			neighborsPutMutex();
			rprintfFlush();
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
	behaviorSystemInit(behaviorTask, 6144);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}


