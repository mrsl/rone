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


#define NEIGHBOR_ROUND_PERIOD			300
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50
#define FLOCK_RV_GAIN_MOVEOBJ			150

#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50
#define TV_MIN							15
#define ACCEL_SCALE						5

#define BUILD_TREE		 0
#define GUESS_COM		 1
#define REMOTE			 2

#define PI				3147
#define nbrEdgeDis		500				//hardcoded distance
#define COM_WAIT		0
#define NORM_TV			75

#define REST			0
#define CNTCLK			1
#define CLKWISE			2
#define ATTEMPTING		3

RadioMessage radioMessageTX;
RadioMessage radioMessageRX;
RadioCmd radioCmdRemoteControl;
char* name = "RCwifi";

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 state = BUILD_TREE;
	Beh behOutput = behInactive;
	boolean printNow,seePivotPoint;
	uint32 neighborRound = 0;
	NbrList nbrList;
	uint8 changeCOM = 0;
	uint8 moveState,i;
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);
	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);

	PosistionCOM treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE+1];
	creatGlobalTreeCOMList(treeGuessCOM);

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
				if(globalRobotList.size >= GLOBAL_ROBOTLIST_MAX_SIZE){
					state = GUESS_COM;
				}
				break;
			}
			case GUESS_COM:{
				switch (moveState){
				case 0:{
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					break;
				}
				case 1:{
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					break;
				}
				case 2:{
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
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

				updateGlobalTreeCOM(globalRobotList, nbrList, treeGuessCOM, nbrEdgeDis);
				if(changeCOM >= COM_WAIT){
					changeCOM = 0;
					if(moveState == 0){				//Transport
						behFlock_gain(&behOutput, &nbrList, TVcmd, FLOCK_RV_GAIN_MOVEOBJ);
						behOutput.rv  = behOutput.rv + (RVcmd*10);
					}else if(moveState == 1){		//Rotate
						int16 COM_Y,COM_X;
						int8 selfIdx = globalRobotListGetIndex(&globalRobotList,roneID);
						if(selfIdx == -1){
							COM_Y = 0;
							COM_X = 0;
						}else{
							COM_Y =  nbrDataGet16(&treeGuessCOM[selfIdx].Y_H,&treeGuessCOM[selfIdx].Y_L);
							COM_X =  nbrDataGet16(&treeGuessCOM[selfIdx].X_H,&treeGuessCOM[selfIdx].X_L);
						}
						orbitGlobalTreePoint(COM_X, COM_Y, &behOutput,  RVcmd);
					}else if(moveState == 2){		//Pivot
						int16 COM_Y,COM_X;
						COM_Y =  nbrDataGet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L);
						COM_X =  nbrDataGet16(&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&treeGuessCOM[GLOBAL_ROBOTLIST_MAX_SIZE].X_L);
						orbitGlobalTreePoint(COM_X, COM_Y, &behOutput,  RVcmd);
						//rprintf("ID %d COMX %d COMY %d TV %d RV %d RVcmd %d\n",roneID,COM_X,COM_Y ,behOutput.tv, behOutput.rv,RVcmd );
					}
					rprintf("ID %d moveState %d TV %d RV %d TVcmd %d RVcmd %d\n",roneID, moveState,behOutput.tv, behOutput.rv, TVcmd,RVcmd );

				}else{
					changeCOM++;
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

				if(comGreen){
					comMoveState++;
					if(comMoveState >= 3){
						comMoveState = 0;
					}
				}
				//Sends commands over radio
				sprintf(radioMessageTX.command.data,"%d,%d,%d,%d,%d,%d",TVcmd, RVcmd, comBlue, comRed, comGreen, comMoveState);
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
				//cprintf("TVcmd %d RVcmd %d\n", TVcmd, RVcmd);
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
	behaviorSystemInit(behaviorTask, 4096);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}


