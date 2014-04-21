/*
 * HostRobot.c
 *
 *  Created on: Mar 21, 2012
 *      Author: Sunny Kim
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"


#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50
#define GAME_TASK_PERIOD				10
#define DEMO_TV							80
#define DEMO_TV_FLOCK					50
#define DEMO_TV_CLUSTER					50
#define DEMO_TV_CLUSTER_BUMP			50
#define DEMO_DISPERSE_SIZE				120  //160 not good//  360 about 10cm

// what runmode is determines what demomode will be - there will be a switch statement
// that takes in runmode as input and set datamode accordingly
#define RUNMODE_IDLE					0
#define RUNMODE_ACTIVE					1
#define RUNMODE_WAIT					2

#define WAIT_TIME						50000
#define LOOP_TIME						1000000

#define TEAM_RED						0
#define TEAM_GREEN						1
#define TEAM_BLUE						2

#define BUTTON_NONE						0
#define BUTTON_ONE						1
#define BUTTON_TWO						2
#define BUTTON_THREE					4

#define DEMOMODE_IDLE					50 // this is a default large value for DEMOMODE_IDLE to avoid unnecessary mod

#define DEMOMODE_COUNT					4 // expect Demomode to be consecutive. (Example: use it for mod)

#define DEMOMODE_FOLLOW					0
#define DEMOMODE_FLOCK					1
#define DEMOMODE_CLUSTER				2
#define DEMOMODE_DISPERSE				3

//#define DEMOMODE_MIDPOINT				5
//#define DEMOMODE_REMOTE					7

#define COMMAND_RADIO_XMIT_PERIOD 		500
#define DEMO_IDLE_LENGTH 				20000

void backgroundTask(void* parameters);
void behaviorTask(void* parameters);
//void commander(void);

RadioCmd radioCmdRemoteControl;


int main(void) {
    systemInit();
    neighborsInit(600);
    neighborsXmitEnable(FALSE);

    //FIXME - background task cuts off sound chip
	osTaskCreate(backgroundTask, "background", 4096, NULL, BACKGROUND_TASK_PRIORITY);
	osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);

	/* Start the scheduler. */
	osTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. This is a very bad thing, and should not happen.*/
	return 0;
}

// the background task runs all the time.  Put slow stuff here, like compute intensive functions
// or large data transfers, like getting the Yun's map from the robot.
void backgroundTask(void* parameters) {
	for (;;) {
		// delay to let other tasks run at same priority
		osTaskDelay(100);
    }
}

#define LED_FL_IDX		3
#define LED_LF_IDX		4
#define LED_FR_IDX		2
#define LED_RF_IDX		1
#define LED_R_IDX		0

void ledObstacleBits(uint8 bits, uint8 color, uint8 brightness) {
	uint8 val;
	uint8 startIdx = ledStartIdx(color);

	val = ((bits & 0x01) ? brightness : 0);
	ledsSetSingle(startIdx + LED_FL_IDX, val);

	val = ((bits & 0x02) ? brightness : 0);
	ledsSetSingle(startIdx + LED_LF_IDX, val);

	val = ((bits & 0x40) ? brightness : 0);
	ledsSetSingle(startIdx + LED_RF_IDX, val);

	val = ((bits & 0x80) ? brightness : 0);
	ledsSetSingle(startIdx + LED_FR_IDX, val);

	val = ((bits & 0x3C) ? brightness : 0);
	ledsSetSingle(startIdx + LED_R_IDX, val);
}

RadioMessage radioMessage;
uint8 demoMode[3] = {DEMOMODE_IDLE};
uint8 runMode[3] = {RUNMODE_IDLE};
uint8 buttonVal = 0;
//uint8 gotCorrectString = 0;

void serialCmdUIFunc(char* arguments) {
	char parseStr[2];
	uint8 i, j, bitNum, joystickVal;
	uint8 gotCorrectTeamString = 0;
	uint8 prevDemoMode[3];
	char* msg = radioCommandGetDataPtr(&radioMessage);

	prevDemoMode[0] = msg[2];
	prevDemoMode[1] = msg[5];
	prevDemoMode[2] = msg[8];

	//gotCorrectString = 0;

	msg[0] = 'g';
	msg[1] = 'r';
	msg[4] = 'g';
	msg[7] = 'b';

	for (i=0; i<3; i++) // parse message for each team
	{
		gotCorrectTeamString = 0;
		bitNum = i * 4; // the bit number that the message starts for each team
		for (j=bitNum; j<bitNum+2; j++) {
			if (arguments[j] == 0) {
				return;
			}
			parseStr[j-bitNum] = arguments[j];
		}

		joystickVal = atoi_hex8(parseStr);
		msg[3 + i * 3] = joystickVal;

		for (j=bitNum+2; j<bitNum+4; j++) {
			if (arguments[j] == 0) {
				return;
			}
			parseStr[j-bitNum-2] = arguments[j];
		}

		buttonVal = atoi_hex8(parseStr);
		//radiomsg[4 + i * 3] = buttonVal;

		if (joystickVal != 0 || buttonVal != 0)
		{
			gotCorrectTeamString = 1;
			runMode[i] = RUNMODE_ACTIVE;
		} else if (runMode[i] != RUNMODE_IDLE) {
			//runMode[i] = RUNMODE_WAIT;
			runMode[i] = RUNMODE_IDLE;		// hack to get rid of screensaver
		}

		switch(runMode[i]) {
		case RUNMODE_ACTIVE:
			demoMode[i] = DEMOMODE_IDLE;

			if (gotCorrectTeamString) {
				switch (buttonVal) {
				case BUTTON_NONE:
					demoMode[i] = DEMOMODE_FOLLOW;
					break;
				case BUTTON_ONE:
					demoMode[i] = DEMOMODE_FLOCK;
					break;
				case BUTTON_TWO:
					demoMode[i] = DEMOMODE_CLUSTER;
					break;
				case BUTTON_THREE:
					demoMode[i] = DEMOMODE_DISPERSE;
					break;
	//			case BUTTON_ONE + BUTTON_TWO: // example for more button combinations in the future
	//				demoMode = DEMOMODE_IDLE;
	//				break;
				default:
					demoMode[i] = prevDemoMode[i];
					break;
				}
			}
			break;
		case RUNMODE_IDLE:
			demoMode[i] = DEMOMODE_IDLE;
			break;
		default:
			break;
		}

		msg[2 + i * 3] = demoMode[i];
	}

	radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessage);
//	ledsSetPattern(LED_ALL, LED_PATTERN_MANUAL, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	ledObstacleBits(joystickVal, LED_RED, 25);
}

uint32 difference32(uint32 smaller, uint32 bigger) {
	if (smaller <= bigger) {
		return (bigger - smaller);
	} else {
		return (bigger + 0xFFFF - smaller + 1);
	}
}

#define NAVIGATION_BEACON		0
#define NAVIGATION_NAVIGATOR	1

// behaviors run every 50ms.
void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 demoModeXmit = DEMOMODE_IDLE;
	uint8 demoModeXmitCounter = 0;
	uint32 neighborRoundPrev = 0;
    boolean newNbrData;
	NbrList nbrList;
	int32 i, j, k, n;
	Beh behOutput, behMove, behIRObstacle, behBump, behRadio;
	BroadcastMessage broadcastMessage;
	uint8 buttonRedOld = 0;
	uint8 buttonGreenOld = 0;
	uint8 buttonBlueOld = 0;
	uint8 buttonRed, buttonGreen, buttonBlue;
	uint8 navigationMode = NAVIGATION_BEACON;
	uint32 ticks;
	uint32 ticksOld;
	uint32 waitStart[3] = {0}, loopStart[3] = {0};
	uint8 toLoop[3] = {0}; // boolean for whether you should be looping through modes or not

	//waitStart = 0;
	//loopStart = 0;
	//toLoop = 0; // default is false

	char prevString[SERIAL_INPUT_STRING_SIZE];
	char testString[SERIAL_INPUT_STRING_SIZE];

	char msg[RADIO_COMMAND_MESSAGE_DATA_LENGTH];
	uint32 msgSize;
	uint32 msgQuality;

	uint8 bumpPrint = 0;

	SerialCmd serialCmdUI;
	serialCommandAdd(&serialCmdUI, "UI", serialCmdUIFunc);

	systemPrintStartup();
	systemPrintMemUsage();
	ticks = ticksOld = osTaskGetTickCount();
	radioCommandAddQueue(&radioCmdRemoteControl, "snake-RC", 1);

	for (;;) {

//		if (gotCorrectString) {
//			runMode = RUNMODE_ACTIVE;
//		} else if (runMode != RUNMODE_IDLE) {
//			runMode = RUNMODE_WAIT;
//		}

		for (i = 0; i < 3; i++) {
			switch (runMode[i]) {
			case RUNMODE_ACTIVE:
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				waitStart[i] = ticks;
				toLoop[i] = 0;
				break;
			case RUNMODE_WAIT:
				if (toLoop[i]) {
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					if (difference32(loopStart[i], ticks) > LOOP_TIME) {
						demoMode[i] = (demoMode[i] + 1)%DEMOMODE_COUNT; // change this to faster bit-wise op later
						loopStart[i] = ticks;
					}
				} else {
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
					demoMode[i] = DEMOMODE_IDLE;
					if (difference32(waitStart[i], ticks) > WAIT_TIME) {
						loopStart[i] = ticks;
						demoMode[i] = DEMOMODE_CLUSTER;
						toLoop[i] = 1;
					}
				}
				break;
			case RUNMODE_IDLE:
				ledsSetPattern(LED_GREEN, LED_PATTERN_CLAW, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			default:
				break;
			}
		}

//		if (demoModeXmitCounter == 0) {
//			radio_send_message(radiomsg, RADIO_MESSAGE_LENGTH);
//			demoModeXmitCounter = 10;
//		}
//		demoModeXmitCounter--;

		buttonRed = buttons_get(BUTTON_RED);
		buttonBlue = buttons_get(BUTTON_BLUE);

		if (buttonRed) {
			for(i=0; i<3; i++) {
				waitStart[i] = ticks;
				toLoop[i] = 0; // default is false
				runMode[i] = RUNMODE_WAIT;
				demoMode[i] = DEMOMODE_IDLE;
			}
			//demoMode = DEMOMODE_IDLE;
		} else if (buttonBlue) {
			for(i=0; i<3; i++) {
				runMode[i] = RUNMODE_IDLE;
				demoMode[i] = DEMOMODE_IDLE;
			}
			//demoMode = DEMOMODE_IDLE;
		}

		ticks = osTaskGetTickCount();

//		if((ticks - ticksOld) > DEMO_IDLE_LENGTH){
//			ticksOld = ticks;
//			demoMode++;
//			demoMode &= 0x03; //mod4
//			if(demoMode == 0) //TODO: maj5 added this hack because without it it goes through
//		//all the demomodes except disperse, but it also goes through DEMOMODE_IDLE (unneeded)
//		//with hack it loops through just fine
//				demoMode = DEMOMODE_DISPERSE;
//			demoModeXmit = demoMode;
//		}

		//behOutput = behMove = behIRObstacle = behBump = behRadio = behInactive;
		//neighborsGetMutex();
		//nbrListCreate(&nbrList);

		//cprintf("Demo Mode %u\n", demoMode);
		//cprintf("demoModeXmit %u\n", demoModeXmit);

//		if (bumpPrint == 0) {
//			cprintf("bump 0x%02X\n", bumpSensorsGetBits());
//			bumpPrint = 5;
//		}
//		bumpPrint--;


//		newNbrData = FALSE;
//		if (neighborsGetRound() != neighborRoundPrev) {
//			newNbrData = TRUE;
//			neighborRoundPrev = neighborsGetRound();
//		}

//		// check for user input on the buttons
//		buttonRed = buttons_get(BUTTON_RED);
//		buttonGreen = buttons_get(BUTTON_GREEN);
//		buttonBlue = buttons_get(BUTTON_BLUE);

//		if (buttonRed & !buttonRedOld) {
//			if (demoMode != DEMOMODE_REMOTE) {
//				demoMode = DEMOMODE_REMOTE;
//				demoModeXmit = DEMOMODE_IDLE;
//			}
//		}
//
//		if (demoMode == DEMOMODE_REMOTE) {
////			if (buttonGreen && !buttonGreenOld) {
////				if (demoModeXmit > DEMOMODE_IDLE) {
////					demoModeXmit--;
////				}
////			}
////			if (buttonBlue && !buttonBlueOld) {
////				if (demoModeXmit < DEMOMODE_MIDPOINT) {
////					demoModeXmit++;
////				}
////			}
//			if (demoModeXmitCounter == 0) {
//				remoteControlSendDemoMode(demoModeXmit);
//				demoModeXmitCounter = 10;
//			}
//			demoModeXmitCounter--;
////			if (buttonRed) {
////				remoteControlAccelRemote();
////			}
//			if (demoModeXmit <= 4) {
//				ledsSetPattern(LED_GREEN, LED_PATTERN_COUNT, LED_BRIGHTNESS_HIGH, demoModeXmit + 1);
//			} else {
//				ledsSetPattern(LED_BLUE, LED_PATTERN_COUNT, LED_BRIGHTNESS_HIGH, (demoModeXmit - 5) + 1);
//			}
//			neighborsXmitEnable(FALSE);
//			behOutput = behInactive;
//		} else {
			//neighborsXmitEnable(TRUE);
//		}

//		buttonRedOld = buttonRed;
//		buttonGreenOld = buttonGreen;
//		buttonBlueOld = buttonBlue;

		//broadcastMsgUpdateLeaderElection(&broadcastMessage);

//		behRadioControl(&behRadio, &demoMode);
//		behBumpReflect(&behBump, DEMO_TV, BUMPMOVE_REFLECT_DISTANCE);
//		behMoveForward(&behMove, DEMO_TV);
//		behIRObstacleAvoid(&behIRObstacle, DEMO_TV);


//		if (demoMode != DEMOMODE_MIDPOINT) {
//			navigationMode = NAVIGATION_BEACON;
//		}

		//demoMode = DEMOMODE_FOLLOW;
//
//		switch (demoMode) {
////		case DEMOMODE_IDLE: {
////			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
////			break;
////		}
//		case DEMOMODE_FOLLOW: {
//			behFollowPredesessor(&behOutput, &nbrList, DEMO_TV);
//			if(behBump.active) {
//				if(behOutput.active) {
//					ledsSetPattern(LED_RED, LED_PATTERN_BLINK, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//					cprintf("FTL bump active output active ");
//				} else {
//					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//					cprintf("FTL bump active ");
//				}
//				behOutput = behBump;
//			}else if(behOutput.active) {
//				// following a robot in front of you
//				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
//				cprintf("FTL robot ");
//			} else {
//				// no bumps, no predcessor.  avoid walls
//				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
//				cprintf("FTL no bumps no pred, avoid walls ");
//				if(behRadio.active) {
//					behOutput = behRadio;
//				} else if(behIRObstacle.active) {
//					behOutput = behIRObstacle;
//				} else {
//					behOutput = behMove;
//				}
//			}
//			break;
//		}
//		case DEMOMODE_FLOCK: {
//			behFlock(&behOutput, &nbrList, DEMO_TV_FLOCK);
//			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);
//			if (behBump.active) {
//				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_TURBO);
//				cprintf("flock bump active");
//				behOutput = behBump;
//				if(!behRadio.active) {
//					//remoteControlSendMessage(0, behBump.rv);
//				}
//			} else if (behRadio.active) {
//				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//				cprintf("flock radio active");
//				behOutput.rv += behRadio.rv;
//				behOutput.tv += behRadio.tv;
//			}
////			else if (behIRObstacle.active) {
////				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
////				behOutput = behIRObstacle;
////				if(!behRadio.active) {
////					//remoteControlSendMessage(0, behIRObstacle.rv);
////				}
////			} else if (behRadio.active) {
////				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
////				behOutput.rv += behRadio.rv;
////				behOutput.tv += behRadio.tv;
////			} else {
////				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
////			}
//			break;
//		}
//		case DEMOMODE_CLUSTER: {
//			//behBumpReflect(&behBump, DEMO_TV_CLUSTER, BUMPMOVE_REFLECT_DIST_SHORT);
////			behCluster(&behOutput, &nbrList, DEMO_TV_CLUSTER);
//			behMJCluster(&behOutput, &nbrList, DEMO_TV_CLUSTER);
//
//			if (behBump.active) {
//				setClusterLight(LED_PATTERN_PULSE, LED_RATE_SLOW);
//				behOutput = behBump;
//				//clusterPlayVictorySound(1);
//			}
//			break;
//		}
//		case DEMOMODE_DISPERSE: {
//			behDisperse(&behOutput, &nbrList, DEMO_TV_CLUSTER, DEMO_DISPERSE_SIZE);
//			if (behOutput.active) {
//				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//				cprintf("disperse active\n");
//
//			} else {
//				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//				cprintf("disperse not active\n");
//			}
//			if (behBump.active) {
//				behOutput = behBump;
//			}
//			break;
//		}
//		case DEMOMODE_IDLE: {
//			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//		}
////		case DEMOMODE_MIDPOINT: {
////			int16 thetaGoal;
////			NbrList nbrListGuides;
////			if(broadcastMsgGetSourceID(&broadcastMessage) == roneID) {
////				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
////				if(behBump.active){
////					bumpSound();
////				}
////			} else {
////				if (navigationMode == NAVIGATION_NAVIGATOR) {
////					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
////				} else {
////					ledsSetPattern(LED_RED, LED_PATTERN_COUNT, LED_BRIGHTNESS_LOW, broadcastMsgGetHops(&broadcastMessage));
////				}
////			}
////
////			if (buttonRed || buttonGreen || buttonBlue) {
////				navigationMode = NAVIGATION_NAVIGATOR;
////			}
////			if (navigationMode == NAVIGATION_NAVIGATOR) {
////				nbrListPickGuides(&nbrListGuides, &broadcastMessage);
////				midAngleNavigation(&behOutput, &nbrListGuides, DEMO_TV);
////				if (behBump.active) {
////					behOutput = behBump;
////				}
////			}
////			break;
////		}
////		default:
////			break;
////		case DEMOMODE_REMOTE: {
////			break;
////		}
//		}
		//neighborsPutMutex();
		//motorSetBeh(&behOutput);
		//osTaskDelayUntil(&lastWakeTime, GAME_TASK_PERIOD);
		osTaskDelay(1);
	}
}

//void commander(void){
//	char msg[RADIO_MESSAGE_LENGTH];
//	uint32 msgSize;
//	uint32 msgQuality;
//	char sendMode;
//	ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOWEST, LED_RATE_SLOW);
//
//	while (!buttons_get(BUTTON_RED) && !buttons_get(BUTTON_GREEN) && !buttons_get(BUTTON_BLUE)) {
//		osTaskDelay(20);
//	}
//	if (buttons_get(BUTTON_RED)) {
//		sendMode = DEMOMODE_CLUSTER;
//	}
//	if (buttons_get(BUTTON_GREEN)) {
//		sendMode = DEMOMODE_FLOCK;
//	}
//	if (buttons_get(BUTTON_BLUE)) {
////		sendMode = DEMOMODE_MIDPOINT;
//	}
//	msg[1] = sendMode;
//	packRadioCmds2(msg); //FIXME verify that getMessage is parsing this
//	radio_send_message(msg, RADIO_MESSAGE_LENGTH);
//
////	sendMode = DEMOMODE_IDLE;
//	osTaskDelay(1000); //wait 1 second before next command
//}
//
