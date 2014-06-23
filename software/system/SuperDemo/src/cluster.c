///*
// * cluster.c
// *
// *  Created on: Sept 19, 2011
// *      Author: lyncas
// */
//
////particleFilter
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//
//#include "roneos.h"
//#include "ronelib.h"
//
//
////TODO: i don't know how to calculate this into the right form....
//#define BEARING_ERROR_MARGIN	0
//
//
//uint8 getEvenOrOdd(uint8 number) {
//	if (number %2 == 0) {
//		return CLUSTER_EVEN;
//	} else {
//		return CLUSTER_ODD;
//	}
//}
//
//
//void clusterPlayVictorySound(uint8 num) {
//	// plays different chords as victory sound depending on the number of neighbors
//	uint8 startNote, instrument, duration, volume;
//	volume = 200;
//	duration = 150;
//	instrument = piano;
//	if (num < 12) {
//		startNote = middleC+num;
//	} else {
//		startNote = middleC+12;
//	}
//	startNote -= 1;
//	noteOnWithDelay(startNote, volume, piano, duration);
//	noteOnWithDelay(startNote+ENote, volume, piano, duration);
//	noteOnWithDelay(startNote+GNote, volume, piano, duration);
//	noteOnWithDelay(startNote+oneOctave, volume, piano, duration);
//}
//
//
//void clusterPlayStartSoundShort(void) {
//	uint8 baseDuration = 125;
//	uint8 startNote = middleC+oneOctave;
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote+FsNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote+DsNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote+FsNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote+DsNote, 200, trumpet, baseDuration);
//	noteOffAll();
//}
//
//
//
//void clusterPlayStartSound(void) {
//	uint8 baseDuration = 125;
//	uint8 startNote = middleC+oneOctave;
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote+FsNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote+DsNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote+FsNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote+DsNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote+FsNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote+DsNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote+FsNote, 200, trumpet, baseDuration*2);
//	noteOnWithDelay(startNote+DsNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(startNote-oneOctave+BNote, 200, trumpet, baseDuration*4);
//	noteOffAll();
//}
//
//void clusterDemo(void){
//	uint32 neighborRoundPrev = 0;
//	uint32 lastWakeTime = osTaskGetTickCount();
//
//	boolean newNbrData;
//	NbrList nbrList;
//	int32 i, j, k, n, clusterCnt;
//	Beh behMove, behIRObstacle, behBump, behRadio;
//	uint8 demoMode = CLUSTER_SEEK;
//	uint8 robotMode;
//	uint8 numSameNeighbors;		// number of neighbors of the same kind (positive/negative)
//
//	char msg[RADIO_MESSAGE_LENGTH];
//	uint32 msgSize;
//	uint32 msgQuality;
//
//	clusterPlayStartSound();
//	if (isEven(roneID)) {
//		robotMode = CLUSTER_EVEN;
//	} else {
//		robotMode = CLUSTER_ODD;
//	}
//	clusterCnt = 0;
//
//	for (;;) {
//		// get the neighbors mutex so that the data does not change during processing
//		newNbrData = FALSE;
//		neighborsGetMutex();
//		nbrListCreate(&nbrList);
//		neighborsPutMutex();
//
//		//calculate neighbors of the same kind in the beginning
//		numSameNeighbors = getNumOfNeighbors(&nbrList);
//		behMove = behIRObstacle = behBump = behRadio = behInactive;
//
//		// check to see if the neighbor round has changed.  If so, process the new neighbor info
//		if (neighborsGetRound() != neighborRoundPrev) {
//			uint32 nbrPeriod;
//			newNbrData = TRUE;
//			neighborRoundPrev = neighborsGetRound();
//			/*if(numSameNeighbors > CLUSTER_MIN){
//				//saturate to avoid slowing down the nbr system
//				//neighborsSetPeriod(neighborsMinNbrPeriod(CLUSTER_MIN));
//			}else{
//				//neighborsSetPeriod(neighborsMinNbrPeriod(numSameNeighbors));
//			}*/
//		}
//
//		// no user input.  put the robot in other modes based on the number of neighbors
//		neighborsXmitEnable(TRUE);
//
//		// run the behaviors
//		switch (demoMode) {
//		case CLUSTER_SEEK: {
//			setClusterLight(LED_PATTERN_PULSE, LED_RATE_SLOW);
//			Nbr* nbrPtr = nbrListGetFirst(&nbrList);
//			if (getEvenOrOdd(nbrPtr->ID) == robotMode) {
//				behMoveToNbr(&behMove, nbrPtr, CLUSTER_TV);
//			}
//			behBumpBackoff(&behBump, CLUSTER_BUMP_TV);
//			if (behBump.active) {
//				setClusterLight(LED_PATTERN_COUNT, 1);
//				behMove = behBump;
//			}
//			break;
//		}
//		case CLUSTER_COMPLETE: {
//			setClusterLight(LED_PATTERN_CIRCLE, LED_RATE_SLOW);
//			clusterCnt++;
//			if(clusterCnt > CLUSTER_COUNT_DOWN){
//				clusterCnt = 0;
//				if (numSameNeighbors <= CLUSTER_MIN) {
//					ledsSetPattern(LED_ALL, LED_PATTERN_BLINK, LED_BRIGHTNESS_LOWEST, LED_RATE_FAST);
//					demoMode = robotMode;
//				}
//				behMove.active = FALSE;
//			}
//			break;
//		}
//		default:
//			break;
//		}
//		motorSetBeh(&behMove);
//
//		if(behBump.active){
//			if(numSameNeighbors > CLUSTER_MIN){
//				// robot has enough neighbors and feels a bump - he's satisfied!
//				ledsSetPattern(LED_ALL, LED_PATTERN_BLINK, LED_BRIGHTNESS_LOWEST, LED_RATE_FAST);
//				if (demoMode == CLUSTER_SEEK) {
//					clusterPlayVictorySound(numSameNeighbors);
//					demoMode = CLUSTER_COMPLETE;
//				}
//			}
//		}
//
//		if(radio_get_message(msg, &msgSize, &msgQuality) ){
//			break; //out of FOR LOOP, anytime a message is broadcast
//		}
//
//		// delay until the next behavior period
//		osTaskDelayUntil(&lastWakeTime, CLUSTER_TASK_PERIOD);
//	 }
//}
//
//
//boolean sameBearing(uint16 bearing1, uint16 bearing2) {
//	// see if the two bearings are the same, allowing some margin
//	uint16 bearingDiff;
//	if(bearing1 > bearing2) {
//		bearingDiff = bearing1 - bearing2;
//	} else {
//		bearingDiff = bearing2 - bearing1;
//	}
//	if (bearingDiff < BEARING_ERROR_MARGIN) {
//		return TRUE;
//	} else {
//		return FALSE;
//	}
//}
//
//uint8 getDisperseSpeed(uint8 numNeighbors) {
//	// give robot different speed depending on the number of robots they have
//	// more neighbors = more stuck in the middle = move slower so they don't
//	// keep chasing the others
//	// also blink different lights in pulse according to the speed
//	if (numNeighbors < 3) {
//		ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_SLOW);
//		return DISPERSE_TV_HIGH;
//	} else if (numNeighbors < 6) {
//		ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_SLOW);
//		return DISPERSE_TV_MEDIUM;
//	} else {
//		ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_SLOW);
//		return DISPERSE_TV_LOW;
//	}
//}
//
//void disperseDemo(void) {
//	uint32 lastWakeTime = osTaskGetTickCount();
//	NbrList nbrList;
//	int16 currentBearing, toBearing;
//	int16 prevBearing = 0;
//	uint8 disperseCount = 0;
//	uint8 i;
//	uint8 disperseMode = DISPERSE_SEEK;
//	uint8 disperseSpeed;
//	Beh beh;
//
//	for (;;) {
//		// copied this part from clustering..
//		neighborsGetMutex();
//		nbrListCreate(&nbrList);
//		neighborsPutMutex();
//		// TODO: set the neighbor period?
//		neighborsXmitEnable(TRUE);
//
//		int16 bearings[nbrList.size];
//		// create the bearing list
//		for (i = 0; i < nbrList.size; i++) {
//			bearings[i] = nbrList.nbrs[i]->bearing;
//		}
//
//		currentBearing = averageArrayAngle(bearings, nbrList.size);
//		if (sameBearing(currentBearing, prevBearing)) {
//			disperseCount++;
//		}
//		if (disperseCount > DISPERSE_COUNT_MAX){
//			// have gone long enough without changing direction
//			disperseMode = DISPERSE_STAY;
//		}
//		prevBearing = currentBearing;
//
//		disperseSpeed = getDisperseSpeed(nbrList.size);
//		// TODO: probably need to add in some bump constraint
//		switch(disperseMode) {
//		case DISPERSE_SEEK:
//			toBearing = (currentBearing+MILLIRAD_PI)%MILLIRAD_2PI;
//			rvBearingController(&beh, toBearing, DISPERSE_RV_GAIN);
//			beh.tv = disperseSpeed;
//			beh.active = TRUE;
//			break;
//		case DISPERSE_STAY:
//			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOWEST, LED_RATE_FAST);
//			beh.active = FALSE;
//			break;
//		default:
//			break;
//		}
//		motorSetBeh(&beh);
//
//		// delay until the next behavior period
//		osTaskDelayUntil(&lastWakeTime, DISPERSE_TASK_PERIOD);
//	}
//}
