/**
 * @file SuperDemo_noremote.c
 * @brief Code for follow-the-leader, flocking, clustering
 * @since Sept 19, 2011
 * @author MRSL
 */

#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

//#include "SuperDemoSongs.h"

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50
#define DEMO_TV							80
#define DEMO_TV_FLOCK					50
#define DEMO_TV_CLUSTER					50
#define DEMO_TV_CLUSTER_BUMP			50
#define DEMO_TV_SORT					30
#define DEMO_DISPERSE_SIZE				120  //about 3.3 cm //160 not good - why?//  360 about 10cm //
#define DEMO_FLOCK_RV_GAIN				40

#define DEMOTEST_TV_MIN  				0
#define DEMOTEST_TV_MAX  				120
#define DEMOTEST_TV_STEP				20

#define DEMOMODE_TEST					-1
#define DEMOMODE_IDLE					0
#define DEMOMODE_FOLLOW					1
#define DEMOMODE_FLOCK					2
#define DEMOMODE_CLUSTER				3
#define DEMOMODE_DISPERSE				4
#define DEMOMODE_BUBBLESORT				5
#define DEMOMODE_MAX					(DEMOMODE_BUBBLESORT + 1)


#define COMMAND_RADIO_XMIT_PERIOD 		500


#define CLUSTER_BUMP_TV			30
#define CLUSTER_TV				40
#define CLUSTER_RV_GAIN			40
#define CLUSTER_SEEK			0
#define CLUSTER_COMPLETE		1


#define CLUSTER_EVEN			0
#define CLUSTER_ODD				1

#define CLUSTER_MIN				2
#define CLUSTER_MAX 			3

#define CLUSTER_COUNT_DOWN 		100

#define CLUSTER_TASK_PERIOD		70//100 -should 100 be removed or is it useful for a test purpose (which should then be specified)

#define DISPERSE_SEEK			0
#define DISPERSE_STAY			1

#define DISPERSE_TV_HIGH		40
#define DISPERSE_TV_MEDIUM		30
#define DISPERSE_TV_LOW			20
#define DISPERSE_RV_GAIN		50

#define DISPERSE_TASK_PERIOD	70


void commander(void);

RadioCmd radioCmdRemoteControl;


/**
 *	@brief robots move towards each other
 *
 *	@param	behPtr
 *	@param	nbrListPtr for the list of neighbors
 *	@param	tv desired translational velocity
 *	@returns the updated behPtr
 */
Beh* behCluster(Beh* behPtr, NbrList* nbrListPtr, int32 tv){
	int32 alpha;

	alpha = nbrListAverageBearing(nbrListPtr);
	if (nbrListGetSize(nbrListPtr) > 0) {
		rvBearingController(behPtr, alpha, CLUSTER_RV_GAIN);
		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->tv = 0;
		behPtr->rv = 0;
		//TODO: why is this active here?
		behPtr->active = TRUE;
	}
	return behPtr;
}


/**
 *	@brief robots move away from each other
 *
 *	Tells you whether the specified button is on or off
 *	@param	behPtr
 *	@param	nbrListPtr
 *	@param	tv
 *	@returns behPtr
 */
Beh* behDisperse(Beh* behPtr, NbrList* nbrListPtr, int32 tv){
	int32 alpha;

	alpha = normalizeAngleMilliRad2(nbrListAverageBearing(nbrListPtr) + MILLIRAD_PI);
	if (nbrListGetSize(nbrListPtr) > 0) {
		rvBearingController(behPtr, alpha, CLUSTER_RV_GAIN);
		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->tv = 0;
		behPtr->rv = 0;
		behPtr->active = TRUE;
	}
	return behPtr;
}



/**
 * @brief Determines which demo mode is running
 *
 * Behaviors run every 50ms
 * @returns void
 */
void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	int8 demoMode = DEMOMODE_IDLE;
	int8 demoModeXmit = DEMOMODE_IDLE;
	uint8 demoModeXmitCounter = 0;
	uint32 neighborRoundPrev = 0;
    boolean newNbrData;
	NbrList nbrList;
	Nbr* nbrPtr;
	int32 i, j, k, n;
	Beh behOutput, behMove, behIRObstacle, behBump, behRadio;
	BroadcastMessage broadcastMessage;
	uint8 buttonRedOld = 0;
	uint8 buttonGreenOld = 0;
	uint8 buttonBlueOld = 0;
	uint8 buttonRed, buttonGreen, buttonBlue;

	radioCommandSetSubnet(2);
	NbrData demoModeXmitMsg;

	// init the neighbor system wth a 300 ms update period
    neighborsInit(300);

    // print startup message and thread memory usage
	systemPrintStartup();
	systemPrintMemUsage();

	// create a radio message for manual remote control.  set it up to have a queue size of one message deep
	//  the alternative is touse radioCommandAddCallback() which will call a call back function when the appropriate message appears
	radioCommandAddQueue(&radioCmdRemoteControl, "SD-Remote", 1);
	//nbrDataCreateIR(&demoModeXmitMsg, "demoMode", 8, 0);
	nbrDataCreate(&demoModeXmitMsg, "demoMode", 8, 0);

	//### minID/maxID
	uint8 minID=roneID,maxID=roneID;
	NbrData minIDMsg,maxIDMsg;
	nbrDataCreate(&minIDMsg,"minid",8,0);
	nbrDataSet(&minIDMsg,minID);
	nbrDataCreate(&maxIDMsg,"maxid",8,0);
	nbrDataSet(&maxIDMsg,maxID);
	//### minHop/maxHop
	uint8 minHop=0,maxHop=0;
	NbrData minHopMsg,maxHopMsg;
	nbrDataCreate(&minHopMsg,"minhop",8,0);
	nbrDataSet(&minHopMsg,minHop);
	nbrDataCreate(&maxHopMsg,"maxhop",8,0);
	nbrDataSet(&maxHopMsg,maxHop);
	//### minTime/maxTime
	uint8 minNonce=0,maxNonce=0;
	NbrData minNonceMsg,maxNonceMsg;
	nbrDataCreate(&minNonceMsg,"minnonce",8,0);
	nbrDataSet(&minNonceMsg,minNonce);
	nbrDataCreate(&maxNonceMsg,"maxnonce",8,0);
	nbrDataSet(&maxNonceMsg,maxNonce);
	//### minLast/maxLast
	uint8 minLastID=roneID,maxLastID=roneID;
	uint8 minLastNonce=0,maxLastNonce=0;
	uint16 minLastRound=0,maxLastRound=0;
	//### minRouter/maxRouter
	Nbr *minRouter=NULL, *maxRouter=NULL;

	//### test mode
	uint8 bumpSensorBits, *irObstaclesBitMatrix;
	int16 demotest_tv = 0;
	uint8 demotest_counter = 0;
	uint8 demotest_flag = 0;
	char rprintbuffer[256];
	uint8 rprintoffset = 0;
	uint32 neighborRound = 0;



	for (;;) {
		//### host mode
		//if (rprintfIsHost()) {
		//	ledsSetBinary(0xff,0xff,0xff);
		//	osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		//	continue;
		//}

		behOutput = behMove = behIRObstacle = behBump = behRadio = behInactive;
		neighborsGetMutex();
		nbrListCreate(&nbrList);
		irObstaclesBitMatrix = irObstaclesGetBitMatrix();
		bumpSensorBits = bumpSensorsGetBits();

		// check to see if
		newNbrData = FALSE;
		if (neighborsGetRound() != neighborRoundPrev) {
			newNbrData = TRUE;
			neighborRoundPrev = neighborsGetRound();
		}

		//get the current max demomode
		demoModeXmit = nbrDataGet(&demoModeXmitMsg);
		//###
		minID=nbrDataGet(&minIDMsg);
		maxID=nbrDataGet(&maxIDMsg);
		minHop=nbrDataGet(&minHopMsg);
		maxHop=nbrDataGet(&maxHopMsg);
		minNonce=nbrDataGet(&minNonceMsg);
		maxNonce=nbrDataGet(&maxNonceMsg);

		// check for user input on the buttons
		buttonRed = buttonsGet(BUTTON_RED);
		buttonGreen = buttonsGet(BUTTON_GREEN);
		buttonBlue = buttonsGet(BUTTON_BLUE);
		if(buttonRed && buttonGreen && buttonBlue) {
			demoModeXmit = DEMOMODE_TEST;
			demotest_tv = 0;
		} else if (demoMode == DEMOMODE_TEST && !buttonGreenOld && buttonGreen) {	//### green button for slower
			demotest_tv -= DEMOTEST_TV_STEP;
		} else if (demoMode == DEMOMODE_TEST && !buttonBlueOld && buttonBlue) {	//### blue button for faster
			demotest_tv += DEMOTEST_TV_STEP;
		} else if ((buttonRed & !buttonRedOld) || (buttonGreen & !buttonGreenOld) || (buttonBlue & !buttonBlueOld)) {
			demoModeXmit++;
		}
		buttonRedOld = buttonRed;
		buttonGreenOld = buttonGreen;
		buttonBlueOld = buttonBlue;

		//look for a new max demo mode among neighbors
		for(i = 0; i < nbrListGetSize(&nbrList); i++){
			nbrPtr = nbrListGetNbr(&nbrList, i);
			if (nbrDataGetNbr(&demoModeXmitMsg, nbrPtr) > demoModeXmit) {
				demoModeXmit = nbrDataGetNbr(&demoModeXmitMsg, nbrPtr);
			}
			//###
			uint8 minRemoteID=nbrDataGetNbr(&minIDMsg, nbrPtr);
			if ((minRemoteID < minID || (minRemoteID == minID && nbrDataGetNbr(&minHopMsg, nbrPtr) < minHop)) && (minRemoteID!=minLastID||nbrDataGetNbr(&minNonceMsg, nbrPtr)!=minLastNonce)) {
				minID = minRemoteID;
				minHop = nbrDataGetNbr(&minHopMsg, nbrPtr)+1;
				minRouter = nbrPtr;
				minNonce = nbrDataGetNbr(&minNonceMsg, nbrPtr);
				minLastID=minID;
				minLastNonce=minNonce;
				minLastRound=(uint16)neighborsGetRound();
			}
			uint8 maxRemoteID=nbrDataGetNbr(&maxIDMsg, nbrPtr);
			if ((maxRemoteID > maxID || (maxRemoteID == maxID && nbrDataGetNbr(&maxHopMsg, nbrPtr) < maxHop)) && (maxRemoteID!=maxLastID||nbrDataGetNbr(&maxNonceMsg, nbrPtr)!=maxLastNonce)) {
				maxID = maxRemoteID;
				maxHop = nbrDataGetNbr(&maxHopMsg, nbrPtr)+1;
				maxRouter = nbrPtr;
				maxNonce = nbrDataGetNbr(&maxNonceMsg, nbrPtr);
				maxLastID=maxID;
				maxLastNonce=maxNonce;
				maxLastRound=(uint16)neighborsGetRound();
			}
		}
		// set and xmit the new max demomode
		// changing the demo mode on one robot changes the other robots
		nbrDataSet(&demoModeXmitMsg, demoModeXmit);

		//###
		if(minID==roneID) {
			minHop=0;
			minRouter=NULL;
			minNonce=(uint8)neighborsGetRound();
		} else if((uint16)neighborsGetRound()-minLastRound>10) {
			minID=roneID;
			minHop=0;
			minRouter=NULL;
			minNonce=(uint8)neighborsGetRound();
		}
		if(maxID==roneID) {
			maxHop=0;
			maxRouter=NULL;
			maxNonce=(uint8)neighborsGetRound();
		} else if((uint16)neighborsGetRound()-maxLastRound>10) {
			maxID=roneID;
			maxHop=0;
			maxRouter=NULL;
			maxNonce=(uint8)neighborsGetRound();
		}
		nbrDataSet(&minIDMsg,minID);
		nbrDataSet(&maxIDMsg,maxID);
		nbrDataSet(&minHopMsg,minHop);
		nbrDataSet(&maxHopMsg,maxHop);
		nbrDataSet(&minNonceMsg,minNonce);
		nbrDataSet(&maxNonceMsg,maxNonce);

		demoMode = demoModeXmit % DEMOMODE_MAX;

		broadcastMsgUpdateLeaderElection(&broadcastMessage, &nbrList);

		//behBumpReflect(&behBump, DEMO_TV, BUMPMOVE_REFLECT_DISTANCE);
		behBumpAvoid(&behBump, DEMO_TV, BUMPMOVE_REFLECT_DISTANCE);
		behMoveForward(&behMove, DEMO_TV);
		behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, DEMO_TV, &nbrList, FALSE);


		switch (demoMode) {
		case DEMOMODE_FOLLOW: {
			behFollowPredesessor(&behOutput, &nbrList, DEMO_TV);
			if(behOutput.active) {
				// you are following a robot in front of you  blink red
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			} else {
				// no bumps, no predecessor.  just drive around and avoid walls and obstacles
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
				if(behRadio.active) {
					behOutput = behRadio;
				} else if(behIRObstacle.active) {
					behOutput = behIRObstacle;
				} else {
					behOutput = behMove;
				}
			}
			behSubsume(&behOutput, &behIRObstacle, &behBump);
			break;
		}
		case DEMOMODE_FLOCK: {
			behFlock_gain(&behOutput, &nbrList, DEMO_TV_FLOCK, DEMO_FLOCK_RV_GAIN);
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			if (behBump.active) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
				behOutput = behBump;
				if(!behRadio.active) {
					//remoteControlSendMessage(0, behBump.rv);
				}
			} else if (behRadio.active) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
				behOutput.rv += behRadio.rv;
				behOutput.tv += behRadio.tv;
			}
			behSubsume(&behOutput, &behIRObstacle, &behBump);
			break;
		}
		case DEMOMODE_DISPERSE: {
			behDisperse(&behOutput, &nbrList, DEMO_TV_CLUSTER);
			behSubsume(&behOutput, &behIRObstacle, &behBump);
			if (behOutput.active) {
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
			} else {
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			}
			break;
		}
		case DEMOMODE_CLUSTER: {
			behCluster(&behOutput, &nbrList, DEMO_TV_CLUSTER);
			behSubsume(&behOutput, &behIRObstacle, &behBump);
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			break;
		}
		case DEMOMODE_BUBBLESORT: {
			//### get smaller/larger neighbor
			Nbr* smallerNbrPtr=NULL;
			Nbr* largerNbrPtr=NULL;
			for (i = 0; i < nbrListGetSize(&nbrList); i++) {
				nbrPtr = nbrListGetNbr(&nbrList, i);
				if(nbrPtr->ID < roneID && (smallerNbrPtr==NULL || nbrPtr->ID > smallerNbrPtr->ID)) {
					smallerNbrPtr=nbrPtr;
//					if(nbrPtr->ID <= minID) {
//						minID = nbrPtr->ID;
//						minHop = 1;
//						minRouter = nbrPtr;
//						minNonce = nbrDataGetNbr(&minNonceMsg, nbrPtr);
//						minLastID=minID;
//						minLastNonce=minNonce;
//						minLastRound=(uint16)neighborsGetRound();
//					}
				}
				if(nbrPtr->ID > roneID && (largerNbrPtr==NULL || nbrPtr->ID < largerNbrPtr->ID)) {
					largerNbrPtr=nbrPtr;
//					if(nbrPtr->ID >= maxID) {
//						maxID = nbrPtr->ID;
//						maxHop = 1;
//						maxRouter = nbrPtr;
//						maxNonce = nbrDataGetNbr(&minNonceMsg, nbrPtr);
//						maxLastID=maxID;
//						maxLastNonce=minNonce;
//						maxLastRound=(uint16)neighborsGetRound();
//					}
				}
			}
			if(smallerNbrPtr!=NULL && largerNbrPtr!=NULL) {	//### normal
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				//### move between smaller and larger
				behMoveBetweenNbrRange(&behOutput, smallerNbrPtr, largerNbrPtr, DEMO_TV_SORT);
				behSubsume(&behOutput, &behOutput, &behBump);
				cprintf("!NORM\tR=%d\tS=%d\tID=%d\tL=%d\t\n",neighborsGetRound(),smallerNbrPtr->ID,roneID,largerNbrPtr->ID);
			} else if(smallerNbrPtr==NULL && largerNbrPtr!=NULL && minID==roneID) {	//### global minimum
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				//### stop
				behStop(&behOutput);
				cprintf("!GMIN\tR=%d\tS=%d\tID=%d\tL=%d\t\n",neighborsGetRound(),0,roneID,largerNbrPtr->ID);
			} else if(smallerNbrPtr!=NULL && largerNbrPtr==NULL && maxID==roneID) {	//### global maximum
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				//### stop
				behStop(&behOutput);
				cprintf("!GMAX\tR=%d\tS=%d\tID=%d\tL=%d\t\n",neighborsGetRound(),smallerNbrPtr->ID,roneID,0);
			} else if(smallerNbrPtr==NULL && largerNbrPtr!=NULL) {	//### local minimum
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
				//### move to smallest source
				behOrbitRange(&behOutput, minRouter, DEMO_TV, 150);
				behSubsume(&behOutput, &behOutput, &behBump);
				cprintf("!LMIN\tR=%d\tS=%d\tID=%d\tL=%d\t\n",neighborsGetRound(),0,roneID,largerNbrPtr->ID);
			} else if(smallerNbrPtr!=NULL && largerNbrPtr==NULL) {	//### local maximum
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
				//### move to largest source
				behOrbitRange(&behOutput, maxRouter, DEMO_TV, 150);
				behSubsume(&behOutput, &behOutput, &behBump);
				cprintf("!LMAX\tR=%d\tS=%d\tID=%d\tL=%d\t\n",neighborsGetRound(),smallerNbrPtr->ID,roneID,0);
			} else {	//### disconnected
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
				cprintf("!DISC\tR=%d\tS=%d\tID=%d\tL=%d\t\n",neighborsGetRound(),0,roneID,0);
			}
			cprintf(">MIN\tID=%d\tHop=%d\tRout=%d\tTime=%d\t(%d,%d,%d)\n",minID,minHop,(minRouter?minRouter->ID:0),minNonce,minLastID,minLastNonce,minLastRound);
			cprintf("<MAX\tID=%d\tHop=%d\tRout=%d\tTime=%d\t(%d,%d,%d)\n",maxID,maxHop,(maxRouter?maxRouter->ID:0),maxNonce,maxLastID,maxLastNonce,maxLastRound);
			break;
		}
		case DEMOMODE_TEST: {
			demotest_tv = bound(demotest_tv, DEMOTEST_TV_MIN, DEMOTEST_TV_MAX);
			behMoveForward(&behOutput, demotest_tv);
			demotest_counter++;
			if(demotest_counter >= 3) {
				demotest_counter = 0;
				demotest_flag = 1 - demotest_flag;
			}
			uint8 ledR = 0;
			if(irObstaclesBitMatrix[0] & 0x01) ledR += 0x04;
			if(irObstaclesBitMatrix[1] & 0x02) ledR += 0x08;
			if(irObstaclesBitMatrix[2] & 0x04) ledR += 0x10;
			if(irObstaclesBitMatrix[3] & 0x08) ledR += 0x01;
			uint8 ledG = 0;
			if(bumpSensorBits & 0x03) ledG += 0x08;
			if(bumpSensorBits & 0x0c) ledG += 0x10;
			if(bumpSensorBits & 0x30) ledG += 0x02;
			if(bumpSensorBits & 0xc0) ledG += 0x04;
			uint8 ledB = 0;
			if(irObstaclesBitMatrix[4] & 0x10) ledB += 0x01;
			if(irObstaclesBitMatrix[5] & 0x20) ledB += 0x02;
			if(irObstaclesBitMatrix[6] & 0x40) ledB += 0x04;
			if(irObstaclesBitMatrix[7] & 0x80) ledB += 0x08;
			if(demotest_flag) {
				ledR += 0x02;
				ledG += 0x01;
				ledB += 0x10;
			}
			ledsSetBinary(ledR,ledG,ledB);
			if(neighborsNewRoundCheck(&neighborRound)) {
				rprintoffset = 0;
				rprintoffset += sprintf(rprintbuffer, "%u,%lu;", roneID, neighborRound);
				rprintf("%s\n", rprintbuffer);
				rprintoffset = 0;
			}
			break;
		}
		default:
		case DEMOMODE_IDLE: {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			// disable the bump behavior
			behBump = behInactive;
			break;
		}
		}

		if(demoMode != DEMOMODE_IDLE && demoMode != DEMOMODE_BUBBLESORT && demoMode != DEMOMODE_TEST) {
			behSubsume(&behOutput, &behIRObstacle, &behBump);
		}

		if (rprintfIsHost()) {	//###
			behOutput = behInactive;
		}

		neighborsPutMutex();
		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

// TODO: Get functional
//void commander(void){
//	char msg[RADIO_MESSAGE_LENGTH_RAW];
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
//		sendMode = DEMOMODE_MIDPOINT;
//	}
//	msg[1] = sendMode;
//	packRadioCmds2(msg); //FIXME verify that getMessage is parsing this
//	radioSendMessage(msg, RADIO_MESSAGE_LENGTH_RAW);
//
//	sendMode = DEMOMODE_IDLE;
//	osTaskDelay(1000); //wait 1 second before next command
//}


/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();
	systemPrintStartup();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}

