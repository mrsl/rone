/*
 *	 globalRobotTree.c
 *  Created on: Jun 23, 2014
 *      Author: Mathew jellins
 */

#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"
#include "globalTreeCOM.h"


#define NEIGHBOR_ROUND_PERIOD			600
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50
#define NAV_TOWER						103
#define PI								3147
#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50
#define TV_MIN							15
#define ACCEL_SCALE						5
#define NAV_TOWER_DISTANCE				1000

#define FLOCK_RV_GAIN_MOVEOBJ			70
#define MAX_CYCLIOD_SPEED				200
#define	RADIUS							250
#define nbrEdgeDis						250				//hardcoded distance
#define COM_WAIT						0
#define NORM_TV							75
#define cycloidPeriod					5000
#define	TOWER_WAIT_TIME					200
#define	cyldoidSpeed					200
#define CYCLOID_ROTATION_MOD			0				//Modifies translation speed vs rotational speed in cyliod motion
														//From 0 to 50, 0 for more translation, 50 for more rotation


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
	rprintfSetSleepTime(NEIGHBOR_ROUND_PERIOD);

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

	PosistionCOM treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+2];
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

	//Cylciod Stuff

	//Nav Tower
	Nbr* nbrNavTowerPtr;
	uint32 navTowerTime;
	int16 vecCOMtoTowerY, vecCOMtoTowerX, vecCOMtoTowerBearing;

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
			if(state!= REMOTE){
				if (buttonsGet(BUTTON_RED)) {
					state = REMOTE;
				}else if (buttonsGet(BUTTON_GREEN)) {
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
				if(nbrNavTowerPtr) {
					int16 x,y;
					x = 0;
					y = NAV_TOWER_DISTANCE;
					vecCOMtoTowerX = x*cosMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER;
					vecCOMtoTowerY = x*sinMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrNavTowerPtr->bearing)/MILLIRAD_TRIG_SCALER;

					//rprintf("COMtoTOWER X %d Y %d B %d CM %d\n",COMtoTowerX,COMtoTowerY, atan2MilliRad((int32)COMtoTowerX,(int32)COMtoTowerY), cylciodModifier);
					nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].X_L,vecCOMtoTowerX);
					nbrDataSet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1].Y_L,vecCOMtoTowerY);
					//Todo: Assumes that 127 is highest ID that will be seen, need to be changed if v15 are used
					nbrList.size--;
					navTowerTime = osTaskGetTickCount();
				}else{
					vecCOMtoTowerX = 0;
					vecCOMtoTowerY = 0;
					Nbr* nbrPtr;
					int i;
					for (i = 0; i < nbrList.size; i++){
						nbrPtr = nbrList.nbrs[i];
						vecCOMtoTowerX = nbrDataGetNbr16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,nbrPtr);
						vecCOMtoTowerY = nbrDataGetNbr16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,nbrPtr);
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


				GlobalTreeCOMUpdate(globalRobotList, nbrList, treeGuessCOM, nbrEdgeDis, &LeaderHeading_H,&LeaderHeading_L);
				int8 selfIdx = globalRobotListGetIndex(&globalRobotList,roneID);
				if(selfIdx == -1){
					COM_Y = 0;
					COM_X = 0;
				}else{
					COM_Y =  nbrDataGet16(&treeGuessCOM[selfIdx].Y_H,&treeGuessCOM[selfIdx].Y_L);
					COM_X =  nbrDataGet16(&treeGuessCOM[selfIdx].X_H,&treeGuessCOM[selfIdx].X_L);
				}
				//rprintf("%d %d\n",COM_X,COM_Y);
				if(moveState == 0){				//Transport
					behFlock_gain(&behOutput, &nbrList, TVcmd, FLOCK_RV_GAIN_MOVEOBJ);
					behOutput.rv  = behOutput.rv + (RVcmd*10);
				}else if(moveState == 1){		//Rotate
					behOutput = behInactive;
					GlobalTreePointOrbit(COM_X, COM_Y, &behOutput,  RVcmd, 0);
				}else if(moveState == 2){		//Pivot
					PIVOT_X =  nbrDataGet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L);
					PIVOT_Y =  nbrDataGet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_L);
					GlobalTreePointOrbit(PIVOT_X, PIVOT_Y, &behOutput,  RVcmd, 0);
				}else if(moveState == 3){		//Cycloid Motion
					vecCOMtoTowerX = vecCOMtoTowerX - COM_X;
					vecCOMtoTowerY = vecCOMtoTowerY - COM_Y;
					vecCOMtoTowerBearing = normalizeAngleMilliRad2(atan2MilliRad((int32)vecCOMtoTowerX,(int32)vecCOMtoTowerY));
					cylciodModifier = abs(vecCOMtoTowerBearing) * (cyldoidSpeed - CYCLOID_ROTATION_MOD) / PI;

					if((abs(COM_X) >= abs(vecCOMtoTowerX/2)) && (abs(COM_Y) >= abs(vecCOMtoTowerY/2))){
						COM_X += vecCOMtoTowerX/2;
						COM_Y += vecCOMtoTowerY/2;
					}

					if(osTaskGetTickCount() >= (navTowerTime + TOWER_WAIT_TIME)){
						GlobalTreePointOrbit(COM_X, COM_Y, &behOutput,  30, vecCOMtoTowerBearing);
						//rprintf("Cyliod Mod %d C %d %d v %d %d NNT\n",cylciodModifier, COM_X,COM_Y,vecCOMtoTowerX/2, vecCOMtoTowerY/2);
					}else{
						GlobalTreePointOrbit(COM_X, COM_Y, &behOutput,  cyldoidSpeed - cylciodModifier, vecCOMtoTowerBearing);
						//rprintf("Cyliod Mod %d C %d %d v %d %d\n",cylciodModifier, COM_X,COM_Y,vecCOMtoTowerX/2, vecCOMtoTowerY/2);
					}
					behOutput.rv = behOutput.rv * 2;
				}
			}
			if(state == REMOTE){
				behOutput = behInactive;

				//convrols tv and rv using accelarmotor sensors
				accX = (((int32)accelerometerGetValue(ACCELEROMETER_X) * ACCEL_IIR_GAIN) + (accX * (100 - ACCEL_IIR_GAIN))) / 100;
				accY = (((int32)accelerometerGetValue(ACCELEROMETER_Y) * ACCEL_IIR_GAIN) + (accY * (100 - ACCEL_IIR_GAIN))) / 100;
				accX = deadzone(accX, ACCEL_DEAD_ZONE);
				accY = deadzone(accY, ACCEL_DEAD_ZONE);

				TVcmd = -accX/ACCEL_SCALE;
				RVcmd = -accY/ACCEL_SCALE;
				if (abs(TVcmd) < TV_MIN) {			//Minimum TV
					if(TVcmd > 0) {
						TVcmd = TV_MIN;
					} else {
						TVcmd = -TV_MIN;
					}
				}

				comBlue = 0;			//Tells robot to grip, moves SEEKER bot to GRIPPING
				comRed = 0;				//Tells robot to release object, moves all robots from MOVE_OBJ to RELEASE mode
				comGreen = 0;				//Sends comand to rotate based on TVcmd as rotational speed/direction
				if (buttonsGet(BUTTON_BLUE) && bounceBlue) {
					comBlue = 1;
					bounceBlue = 0;
				}
				if (buttonsGet(BUTTON_RED) && bounceRed) {
					comRed = 1;
					bounceRed = 0;
				}
				if (buttonsGet(BUTTON_GREEN) && bounceGreen) {
					comGreen = 1;
					bounceGreen = 0;
				}

				if (!buttonsGet(BUTTON_BLUE)) {
					bounceBlue = 1;
				}
				if (!buttonsGet(BUTTON_RED)) {
					bounceRed = 1;
				}
				if (!buttonsGet(BUTTON_GREEN)) {
					bounceGreen = 1;
				}

				if(comRed){
					comMoveState++;
					if(comMoveState >= 4){
						comMoveState = 0;
					}
				}
				//Sends commands over radio
				sprintf(radioMessageTX.command.data,"%d,%d,%d,%d,%d,%d",TVcmd, RVcmd, comBlue, comRed, comGreen, comMoveState);
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
			}

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
	behaviorSystemInit(behaviorTask, 6144);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
