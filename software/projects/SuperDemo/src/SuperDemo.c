///*
// * SuperDemo.c
// *
// *  Created on: Sept 19, 2011
// *      Author: lyncas
// */
////#include <stdio.h>
////#include <stdlib.h>
//
//#include "roneos.h"
//#include "ronelib.h"
//
//#include "SuperDemoSongs.h"
//
//#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
//#define BEHAVIOR_TASK_PERIOD			50
//#define DEMO_TV							80
//#define DEMO_TV_FLOCK					50
//#define DEMO_TV_CLUSTER					50
//#define DEMO_TV_CLUSTER_BUMP			50
//#define DEMO_DISPERSE_SIZE				120  //160 not good//  360 about 10cm
//
//#define DEMOMODE_IDLE					0
//#define DEMOMODE_FOLLOW					1
//#define DEMOMODE_FLOCK					2
//#define DEMOMODE_CLUSTER				3
//#define DEMOMODE_DISPERSE				4
//#define DEMOMODE_MIDPOINT				5
//#define DEMOMODE_REMOTE					7
//
//#define COMMAND_RADIO_XMIT_PERIOD 		500
//
//void backgroundTask(void* parameters);
//void behaviorTask(void* parameters);
//void commander(void);
//
//RadioCmd radioCmdRemoteControl;
//
//int main(void) {
//	systemPreInit();
//    systemInit();
//    neighborsInit(600);
//    neighborsXmitEnable(FALSE);
//
//    //FIXME - background task cuts off sound chip
//	//osTaskCreate(backgroundTask, "background", 256, NULL, BACKGROUND_TASK_PRIORITY);
//	osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
//
//	/* Start the scheduler. */
//	osTaskStartScheduler();
//
//    /* Will only get here if there was insufficient memory to create the idle
//    task. This is a very bad thing, and should not happen.*/
//	return 0;
//}
//
//// the background task runs all the time.  Put slow stuff here, like compute intensive functions
//// or large data transfers, like getting the Yun's map from the robot.
//void backgroundTask(void* parameters) {
//	for (;;) {
//		// delay to let other tasks run at same priority
//		osTaskDelay(100);
//    }
//}
//
//
//boolean isEven(uint8 number){
//	if(number % 2 == 0){
//		return TRUE;
//	}
//	return FALSE;
//}
//
//
//void setClusterLight(uint8 pattern, uint8 rate) {
//	if (isEven(roneID)) {
//		ledsSetPattern(LED_RED, pattern, LED_BRIGHTNESS_LOWEST, rate);
//	} else {
//		ledsSetPattern(LED_BLUE, pattern, LED_BRIGHTNESS_LOWEST, rate);
//	}
//}
//
//
//uint8 getNumOfNeighbors(NbrList* nbrListPtr) {
//	uint8 even, odd, w;
//	even = 0;
//	odd = 0;
//	for(w = 0; w < nbrListPtr->size; w++){
//		if(isEven(nbrListPtr->nbrs[w]->ID)){
//			even += 1;
//		}else{
//			odd += 1;
//		}
//	}
//	if (isEven(roneID)) {
//		return even;
//	} else {
//		return odd;
//	}
//
//}
//
//
//uint16 clusterTargetBrg(NbrList* nbrListPtr){
//	int32 w, x, y, theta;
//	uint16 targetBrg = 0;
//	Nbr* nbrPtr;
//
//	x = 0;
//	y = 0;
//	if(isEven(roneID)){
//		for(w = 0; w < nbrListPtr->size; w++){
//			nbrPtr = nbrListPtr->nbrs[w];
//
//			if(isEven(nbrListPtr->nbrs[w]->ID)){
//				theta = (int32)nbrPtr->bearing;
//				x += cosMilliRad(theta);
//				y += sinMilliRad(theta);
//				//targetBrg = nbrListPtr->nbrs[w]->bearing; //OLD way
//				//return targetBrg;
//			}
//		}
//	}
//	else{
//		for(w = 0; w < nbrListPtr->size; w++){
//			nbrPtr = nbrListPtr->nbrs[w];
//			if(!isEven(nbrListPtr->nbrs[w]->ID)){
//				theta = (int32)nbrPtr->bearing;
//				x += cosMilliRad(theta);
//				y += sinMilliRad(theta);
//				//targetBrg = nbrListPtr->nbrs[w]->bearing; //OLD way
//				//return targetBrg;
//			}
//		}
//	}
//	targetBrg = atan2MilliRad(y, x);//NEW WAY //FIXME - this doesn't work
//
//	return targetBrg;
//}
//
//
//Beh* behCluster(Beh* behPtr, NbrList* nbrListPtr, int32 tv){
//	int32 theta;
//	Nbr* nbrPtr;
//	uint8 w, numSameNeighbors;
//	uint16 targetBrg;
//
//	setClusterLight(LED_PATTERN_CIRCLE, LED_RATE_SLOW);
//	numSameNeighbors = getNumOfNeighbors(nbrListPtr);
//	if (numSameNeighbors > 0) {
//
//
//		targetBrg = clusterTargetBrg(nbrListPtr);
//		rvBearingController(behPtr, (int32) targetBrg, CLUSTER_RV_GAIN);
//		behPtr->tv = tv;
//		behPtr->active = TRUE;
//	} else {
//		behPtr->tv = 0;
//		behPtr->rv = 0;
//		behPtr->active = TRUE;
//	}
//	return behPtr;
//}
//
//Beh* behDisperse(Beh* behPtr, NbrList* nbrListPtr, int32 tv, uint16 disperseSize){
//	Nbr* nbrPtr;
//	uint8 i, numSameNeighbors;
//	uint16 targetBrg;
//	int32 x, y, theta, d;
//
//	x = 0;
//	y = 0;
//	for(i = 0; i < nbrListPtr->size; i++){
//		nbrPtr = nbrListPtr->nbrs[i];
//		x += cosMilliRad(nbrPtr->bearing);
//		y += sinMilliRad(nbrPtr->bearing);
//	}
//
//	d = sqrtInt(x*x + y*y);
//	if(d > MILLIRAD_TRIG_SCALER * disperseSize / 100) {
//		theta = (int32)normalizeAngleMilliRad(atan2MilliRad(y, x) + MILLIRAD_PI);
//		rvBearingController(behPtr, theta, CLUSTER_RV_GAIN);
//		behPtr->tv = tv;
//		behPtr->active = TRUE;
//	} else {
//		behPtr->tv = 0;
//		behPtr->rv = 0;
//		behPtr->active = FALSE;
//	}
//	return behPtr;
//}
//
//
//
//#define NAVIGATION_BEACON		0
//#define NAVIGATION_NAVIGATOR	1
//
//// behaviors run every 50ms.
//void behaviorTask(void* parameters) {
//	uint32 lastWakeTime = osTaskGetTickCount();
//	uint8 demoMode = DEMOMODE_IDLE;
//	uint8 demoModeXmit = DEMOMODE_IDLE;
//	uint8 demoModeXmitCounter = 0;
//	uint32 neighborRoundPrev = 0;
//    boolean newNbrData;
//	NbrList nbrList;
//	int32 i, j, k, n;
//	Beh behOutput, behMove, behIRObstacle, behBump, behRadio;
//	BroadcastMessage broadcastMessage;
//	uint8 buttonRedOld = 0;
//	uint8 buttonGreenOld = 0;
//	uint8 buttonBlueOld = 0;
//	uint8 buttonRed, buttonGreen, buttonBlue;
//	uint8 navigationMode = NAVIGATION_BEACON;
//
//	char msg[RADIO_MESSAGE_LENGTH];
//	uint32 msgSize;
//	uint32 msgQuality;
//
//	uint8 bumpPrint = 0;
//
//	systemPrintStartup();
//	systemPrintMemUsage();
//	radioCommandAddQueue(&radioCmdRemoteControl, 1);
//
//	for (;;) {
//		behOutput = behMove = behIRObstacle = behBump = behRadio = behInactive;
//		neighborsGetMutex();
//		nbrListCreate(&nbrList);
//
//		if (bumpPrint == 0) {
//			cprintf("bump 0x%02X\n", bumpSensorsGetBits());
//			bumpPrint = 5;
//		}
//		bumpPrint--;
//
//
//		newNbrData = FALSE;
//		if (neighborsGetRound() != neighborRoundPrev) {
//			newNbrData = TRUE;
//			neighborRoundPrev = neighborsGetRound();
//		}
//
//		// check for user input on the buttons
//		buttonRed = buttons_get(BUTTON_RED);
//		buttonGreen = buttons_get(BUTTON_GREEN);
//		buttonBlue = buttons_get(BUTTON_BLUE);
//
//		if (buttonRed & !buttonRedOld) {
//			if (demoMode != DEMOMODE_REMOTE) {
//				demoMode = DEMOMODE_REMOTE;
//				demoModeXmit = DEMOMODE_IDLE;
//			}
//		}
//
//		if (demoMode == DEMOMODE_REMOTE) {
//			if (buttonGreen && !buttonGreenOld) {
//				if (demoModeXmit > DEMOMODE_IDLE) {
//					demoModeXmit--;
//				}
//			}
//			if (buttonBlue && !buttonBlueOld) {
//				if (demoModeXmit < DEMOMODE_MIDPOINT) {
//					demoModeXmit++;
//				}
//			}
//			if (demoModeXmitCounter == 0) {
//				remoteControlSendDemoMode(&radioCmdRemoteControl, demoModeXmit);
//				demoModeXmitCounter = 10;
//			}
//			demoModeXmitCounter--;
//			if (buttonRed) {
//				remoteControlAccelRemote(&radioCmdRemoteControl);
//			}
//			if (demoModeXmit <= 4) {
//				ledsSetPattern(LED_GREEN, LED_PATTERN_COUNT, LED_BRIGHTNESS_HIGH, demoModeXmit + 1);
//			} else {
//				ledsSetPattern(LED_BLUE, LED_PATTERN_COUNT, LED_BRIGHTNESS_HIGH, (demoModeXmit - 5) + 1);
//			}
//			neighborsXmitEnable(FALSE);
//			behOutput = behInactive;
//		} else {
//			neighborsXmitEnable(TRUE);
//		}
//
//		buttonRedOld = buttonRed;
//		buttonGreenOld = buttonGreen;
//		buttonBlueOld = buttonBlue;
//
//		broadcastMsgUpdateLeaderElection(&broadcastMessage);
//
//		behRadioControl(&radioCmdRemoteControl, &behRadio, &demoMode);
//		behBumpReflect(&behBump, DEMO_TV, BUMPMOVE_REFLECT_DISTANCE);
//		behMoveForward(&behMove, DEMO_TV);
//		behIRObstacleAvoid(&behIRObstacle, DEMO_TV);
//
//
//		if (demoMode != DEMOMODE_MIDPOINT) {
//			navigationMode = NAVIGATION_BEACON;
//		}
//
//		//demoMode = DEMOMODE_FOLLOW;
//
//		switch (demoMode) {
//		case DEMOMODE_IDLE: {
//			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//			break;
//		}
//		case DEMOMODE_FOLLOW: {
//			behFollowPredesessor(&behOutput, &nbrList, DEMO_TV);
//			if(behBump.active) {
//				if(behOutput.active) {
//					ledsSetPattern(LED_RED, LED_PATTERN_BLINK, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//				} else {
//					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//				}
//				behOutput = behBump;
//			}else if(behOutput.active) {
//				// following a robot in front of you
//				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
//			} else {
//				// no bumps, no predcessor.  avoid walls
//				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
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
//				behOutput = behBump;
//				if(!behRadio.active) {
//					//remoteControlSendMessage(0, behBump.rv);
//				}
//			} else if (behRadio.active) {
//				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
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
//			behCluster(&behOutput, &nbrList, DEMO_TV_CLUSTER);
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
//			} else {
//				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//			}
//			if (behBump.active) {
//				behOutput = behBump;
//			}
//			break;
//		}
//		case DEMOMODE_MIDPOINT: {
//			int16 thetaGoal;
//			NbrList nbrListGuides;
//			if(broadcastMsgGetSourceID(&broadcastMessage) == roneID) {
//				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
//				if(behBump.active){
//					//bumpSound();
//				}
//			} else {
//				if (navigationMode == NAVIGATION_NAVIGATOR) {
//					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
//				} else {
//					ledsSetPattern(LED_RED, LED_PATTERN_COUNT, LED_BRIGHTNESS_LOW, broadcastMsgGetHops(&broadcastMessage));
//				}
//			}
//
//			if (buttonRed || buttonGreen || buttonBlue) {
//				navigationMode = NAVIGATION_NAVIGATOR;
//			}
//			if (navigationMode == NAVIGATION_NAVIGATOR) {
//				nbrListPickGuides(&nbrListGuides, &broadcastMessage);
////				midAngleNavigation(&behOutput, &nbrListGuides, DEMO_TV);
//				if (behBump.active) {
//					behOutput = behBump;
//				}
//			}
//			break;
//		}
//		default:
//		case DEMOMODE_REMOTE: {
//			break;
//		}
//		}
//		neighborsPutMutex();
//		motorSetBeh(&behOutput);
//		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
//	}
//}
//
//
////void commander(void){
////	char msg[RADIO_MESSAGE_LENGTH];
////	uint32 msgSize;
////	uint32 msgQuality;
////	char sendMode;
////	ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOWEST, LED_RATE_SLOW);
////
////	while (!buttons_get(BUTTON_RED) && !buttons_get(BUTTON_GREEN) && !buttons_get(BUTTON_BLUE)) {
////		osTaskDelay(20);
////	}
////	if (buttons_get(BUTTON_RED)) {
////		sendMode = DEMOMODE_CLUSTER;
////	}
////	if (buttons_get(BUTTON_GREEN)) {
////		sendMode = DEMOMODE_FLOCK;
////	}
////	if (buttons_get(BUTTON_BLUE)) {
////		sendMode = DEMOMODE_MIDPOINT;
////	}
////	msg[1] = sendMode;
////	packRadioCmds2(msg); //FIXME verify that getMessage is parsing this
////	radio_send_message(msg, RADIO_MESSAGE_LENGTH);
////
////	sendMode = DEMOMODE_IDLE;
////	osTaskDelay(1000); //wait 1 second before next command
////}
//
//
