/*
 * test_bumpSkirtWallAngle.c
 *
 *     Created on: 07/30/2012
 *         Author: Mathew Jellins, Lauren Schmidt
 *        Summary: This code calculates a tree based on the weight.
 *        Weight is determined by how close walls are, and a node can be excluded if a wall is to close.
 *        A path can be calculated to the source from any node.
 */
#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD			750
#define MOTION_TV_MIN  					0
#define MOTION_TV_DEFAULT				40
#define MOTION_TV_MAX  					120
#define MOTION_TV_STEP					20
#define BUMP_RELECT_DISTANCE			15
#define MAJOR_AXIS						25
#define MAJOR_AXIS_BITS					2
#define MINOR_AXIS						10
#define MINOR_AXIS_BITS					175
#define SWITCH_DELAY					5
#define SAFETY_CHECK_TIME				5
#define OBJECT_SIZE_WAIT				1000
#define SMWEIGHT_WAIT					2000

#define MODE_IDLE		 0
#define MODE_BUMPIN		 1
#define SHOW_TREE		 2
#define SOURCE			 3
#define SHOW_TREE_SOURCE 4
#define CHANGE_OBJECT_SIZE 5
#define LEDS			 6

#define UNSAFE			 0
#define MINORSAFE		 1
#define SAFE			 2
#define TRANSPORT        3

uint32 majorAxis = 25;
uint32 majorAxisBits = 100;
uint32 minorAxis = 10;
uint32 minorAxisBits = 250;

NbrData messagePower;
uint8 ledG;

void rprintNbr(Nbr* nbrPtr) {
	uint8 i;
	if (nbrPtr) {
		// print orientation matrix for user viewing
		cprintf("\n nbr=%d", nbrGetID(nbrPtr));
		irCommsOrientationBitMatrixPrint(nbrPtr->orientationsBitsMatrix,
				nbrPtr->rangeBits);

		//print out processed bearing/orientation
		cprintf("brg=%5d, ort=%5d, rng=%5d\n", nbrGetBearing(nbrPtr),
				nbrGetOrientation(nbrPtr), nbrGetRange(nbrPtr));

		//print raw nbr orientation matrix
		rprintf(" n,%d,%d", nbrGetID(nbrPtr),
				nbrDataGetNbr(&messagePower, nbrPtr));
		for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
			rprintf(",%d", nbrPtr->orientationsBitsMatrix[i]);
		}
		for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
			rprintf(",%d", nbrPtr->rangeBits[i]);
			if (nbrPtr->rangeBits[i] > ledG) {
				ledG = nbrPtr->rangeBits[i];
			}
		}
	}
}

void navigationLEDsSet(uint8 redCount, uint8 greenCount, uint8 blueCount,
		uint8 brightness) {
	uint8 ledIdx = 0;
	uint8 i;
	uint8 brightnessRed = brightness;

	// disable animation control
	ledsSetPattern(LED_ALL, LED_PATTERN_MANUAL, LED_BRIGHTNESS_MED,
			LED_RATE_MED);

	for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
		if (i < blueCount) {
			ledsSetSingle(ledIdx++, brightness);
		} else {
			ledsSetSingle(ledIdx++, 0);
		}
	} //set blue LEDs
	for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
		if (i < greenCount) {
			ledsSetSingle(ledIdx++, brightness);
		} else {
			ledsSetSingle(ledIdx++, 0);
		}
	} //set green LEDs
	if (redCount > LED_NUM_ELEMENTS_PER_COLOR) {
		redCount -= LED_NUM_ELEMENTS_PER_COLOR;
		brightnessRed = brightness * 4;
	} //if more red supposed to be on than on the robot
	for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
		if (i < redCount) {
			ledsSetSingle(ledIdx++, brightnessRed);
		} else {
			ledsSetSingle(ledIdx++, 0);
		}
	} //set red LEDs
} //navigationLEDsSet()
// behaviors run every 50ms.  They should be designed to be short, and terminate quicky.
// they are used for robot control.  Watch this space for a simple behavior abstraction
// to appear.

void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint8 irObsBits, parentID, parentType;
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput;
	boolean printNow = FALSE;
	boolean printPath = FALSE;
	boolean printSource = FALSE;
	uint8 i, j, showTree;
	uint32 neighborRound = 0;
	NbrList nbrList;
	Nbr* nbrPtr;
	Nbr* bestParentPtr;
	Nbr* prevParentPtr = 0;
	uint8 switchDelay = 0;
	uint32 timer = 0;

	boolean partOfTree = FALSE;
	char* typeName = "";
	uint8 type = 0;

	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(2);

	NbrData msgNbrType;
	NbrData msgParentID;
	NbrData msgShowTree;
	NbrDataFloat msgWeight;

	nbrDataCreate(&msgNbrType, "type", 2, UNSAFE);
	nbrDataCreate(&msgParentID, "parentID", 7, 0);
	nbrDataCreate(&msgShowTree, "showTree", 1, 0);
	nbrDataCreateFloat(&msgWeight, "weight");

	uint16 IRXmitPower = IR_COMMS_POWER_MAX/3;
	uint8 avgRound = 0;
	uint16 numBits;
	int32 bearingToWall, minorSafeTheta;
	uint8 bits;
	int32 newRV = 0, newRV1 = 0, newRV2 = 0;
	int32 distSum = 0;
	float bestDist = 90000.0;
	uint8 safeMinorWeight = 2;
	uint32 objectSizeDelay = 0;
	uint8 sum = 0;

	uint32 avg;
	char tempstrFloat[4];
	float prevWeight = -1.0;

	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW,
					LED_RATE_MED);
			continue;
		} //end if host
		else {
			/*** INIT STUFF ***/
			behOutput = behInactive;
			neighborsGetMutex();
			//printNow = TRUE;
			printNow = neighborsNewRoundCheck(&neighborRound);    // We do not understand when printnow is true?

			nbrListCreate(&nbrList);

			irCommsSetXmitPower(IRXmitPower);

			nbrListCreate(&nbrList);

			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			numBits = 0;
			partOfTree = FALSE;

			/*** DETERMINE TYPE IF NOT SOURCE ***/
			if (printNow && (navigationMode != SOURCE)) {

				irObsBits = irObstaclesGetBits();
				numBits = 0;
				avgRound++;

				//Determining number of obs bits and adding to sum
				for (j = 0; j < IR_COMMS_NUM_OF_RECEIVERS; ++j) {
					if ((irObsBits & 1) == 1) {
						numBits++;
					}
					irObsBits = irObsBits >> 1;
				}
				sum = sum + numBits;

				//If we have waited enough time, take average and classify robot
				if (avgRound % SAFETY_CHECK_TIME == 0) {
					avg = (sum * 100) / SAFETY_CHECK_TIME;
					if (avg <= minorAxisBits && avg > majorAxisBits) {
						typeName = "minor safe only";
						type = 1;
						ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
								LED_BRIGHTNESS_MED, LED_RATE_MED);
					} else if (avg <= majorAxisBits) {
						typeName = "safe";
						type = 2;
						ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,
								LED_BRIGHTNESS_MED, LED_RATE_MED);
					} else {
						typeName = "unsafe";
						type = 0;
						ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
								LED_BRIGHTNESS_MED, LED_RATE_MED);
					}
					//rprintf(" avg %d, type %s",avg, typeName);
					nbrDataSet(&msgNbrType, type);
					sum = 0;
				}
				//rprintf(" avg bits %d, type %d \n",avg, type);
			} //determine type

			/*** DECIDE IF TREE SHOULD BE BUILT ***/
			if (type == 0) {
				//we are unsafe, stop being in tree - no parent, highest weight possible
				broadcastMessage.active = FALSE;
				nbrDataSet(&msgNbrType, type);
				nbrDataSet(&msgParentID, 0);
				nbrDataSetFloat(&msgWeight, -1.0);
			} else {

				/*** Choosing Parent ***/
				if (navigationMode != SOURCE) {
					float weight = -1.0;
					float bestWeight = -1.0;
					float bestParentWeight = -1.0;
					bestParentPtr = 0;
					//For all parents in list

					/***********Algorithm 1*****************/

					for (i = 0; i < nbrList.size; i++) {
						nbrPtr = nbrList.nbrs[i];
						parentType = nbrDataGetNbr(&msgNbrType, nbrPtr);
						if (parentType == TRANSPORT) {
							//This is a transport robot, don't try to pick as parent
							continue;
						}
						if (broadcastMsgGetSourceIDNbr(&broadcastMessage,nbrPtr) == 0) {
							// this neighbor has no message, continue.
							continue;
						} else {
							//Check if parent is safe or minor safe, also checks to exclude weights that are -1 (No weight),
							if ((parentType == 1 || parentType == 2)&& (nbrDataGetNbrFloat(&msgWeight, nbrPtr)> -1.0)) {
								//weight = the neighbors weight + distance to neighbor
								if (printNow) {
									//cprintf("p%d r%d\n",nbrPtr->ID);
								}
								weight = nbrDataGetNbrFloat(&msgWeight, nbrPtr)	+ (nbrPtr->range) / 10.0;
								if (nbrDataGetNbr(&msgNbrType, nbrPtr) == 1) {
									//if neighbor is safe minor, add weight for angle
									minorSafeTheta = 3141 + nbrGetOrientation(nbrPtr)- nbrGetBearing(nbrPtr);
									minorSafeTheta = abs(normalizeAngleMilliRad2(minorSafeTheta));
									weight = weight	+ minorSafeTheta * safeMinorWeight *1.0;
									//weight = weight + abs(3141 + nbrGetOrientation(nbrPtr) - nbrGetBearing(nbrPtr))*safeMinorWeight;
									//weight = weight + 2.4;
								}
								if (bestParentPtr == 0) {
									//If we have no parent yet, choose as parent
									bestParentPtr = nbrPtr;
									bestParentWeight = weight;
								} else {
									if (weight < bestParentWeight) {
										//If new weight is less then weight of old parent, make new parent
										bestParentPtr = nbrPtr;
										bestParentWeight = weight;
									} else if (weight == bestParentWeight && weight >-1.0 ) {
										//If weight equals weight of old parent, choose the one that is safe
										if (nbrDataGetNbr(&msgNbrType, bestParentPtr) == 1 && nbrDataGetNbr(&msgNbrType, nbrPtr) == 2) {
											bestParentPtr = nbrPtr;
											bestParentWeight = weight;
										}
									} //end else weight ==
								} //end else bestparent != null
							} //end if parent is safe and  in  tree
						}
					} //endfor checking all potential parents

					/*** DETERMINE IF PARENT SWITCH ALLOWED ***/

					if (switchDelay == SWITCH_DELAY) {
						//if we've waited long enough to switch
						switchDelay = 0;
						if (bestParentPtr != 0) {
							//if we have a parent selected
						/*	if (bestParentPtr->ID == prevParentPtr->ID) {
								bestDist = bestDist * (1.0 - .1)
										+ bestParentPtr->range * .1;
								sprintf(tempstrFloat, "%.2f", bestDist);
								rprintf(" av%s ", tempstrFloat);
							} else {
								bestDist = bestParentPtr->range;
								sprintf(tempstrFloat, "%.2f", bestDist);
								rprintf(" nw%s ", tempstrFloat);
							}
							//prevParentPtr = bestParentPtr;
							prevWeight = nbrDataGetNbrFloat(&msgWeight,	prevParentPtr);
							parentType = nbrDataGetNbr(&msgNbrType,	bestParentPtr);
							if (parentType == MINORSAFE) {
								//if our parent is minor safe, add weighted angle penalty
								//otherwise, no penalty for safe parents
								minorSafeTheta = 3141 + nbrGetOrientation(prevParentPtr) - nbrGetBearing(prevParentPtr);
								minorSafeTheta = abs(normalizeAngleMilliRad2(minorSafeTheta));
								bestWeight = bestWeight	+ minorSafeTheta * safeMinorWeight * 1.0;
								bestWeight = bestWeight + (bestDist) / 10.0;
							} else {
								//add only distance to weight if parent is safe
								bestWeight = bestWeight + (bestDist) / 10.0;
								//prevWeight = prevWeight + 1.0;
							}
							prevWeight = bestWeight;
							prevParentPtr = bestParentPtr;


							*/
							prevParentPtr = bestParentPtr;
							prevWeight = bestParentWeight;


							//Set parent ID and new weight
							nbrDataSet(&msgParentID, prevParentPtr->ID);
							nbrDataSetFloat(&msgWeight, prevWeight);

						} else {
							//we now don't have a new parent after waiting
							nbrDataSet(&msgParentID, 0);
							nbrDataSetFloat(&msgWeight, -1.0);
							prevParentPtr = 0;
							prevWeight = -1.0;
						}

					} else {
						//haven't waited long enough to switch
						if (prevParentPtr != 0) {
							//if we had a parent previously, keep info
							nbrDataSet(&msgParentID, prevParentPtr->ID);
							nbrDataSetFloat(&msgWeight, prevWeight);
							if (printNow) {
								//rprintf(" p%d w%d",prevParentPtr->ID, prevWeight);
							}
						} else {
							//if we don't have a parent, continue to be orphaned
							nbrDataSet(&msgParentID, 0);
							nbrDataSetFloat(&msgWeight, -1.0);
							prevParentPtr = 0;
							prevWeight = -1.0;

						}
					}
					if (printNow)
						switchDelay++;
					/*
					 if(switchDelay == SWITCH_DELAY){
					 //if we've waited long enough to switch
					 switchDelay = 0;
					 if(bestParentPtr != NULL){
					 //if we have a parent selected
					 bestDist = distSum/SWITCH_DELAY;
					 distSum = 0;

					 rprintf(" av%d " ,bestDist);

					 //prevParentPtr = bestParentPtr;
					 prevWeight = nbrDataGetNbrFloat(&msgWeight, prevParentPtr);
					 parentType = nbrDataGetNbr(&msgNbrType, bestParentPtr);
					 if(parentType == MINORSAFE){
					 //if our parent is minor safe, add weighted angle penalty
					 //otherwise, no penalty for safe parents
					 minorSafeTheta = 3141 + nbrGetOrientation(prevParentPtr) - nbrGetBearing(prevParentPtr);
					 minorSafeTheta = abs(normalizeAngleMilliRad2(minorSafeTheta));
					 bestWeight = bestWeight + minorSafeTheta*safeMinorWeight;
					 bestWeight = bestWeight + (bestDist)/10;
					 } else{
					 //add only distance to weight if parent is safe
					 bestWeight = bestWeight + (bestDist)/10;
					 //prevWeight = prevWeight + 1.0;
					 }
					 if((prevWeight <= 0.0) || (bestWeight < prevWeight)){
					 prevWeight = bestWeight;
					 prevParentPtr = bestParentPtr;

					 }
					 //Set parent ID and new weight
					 nbrDataSet(&msgParentID, prevParentPtr->ID);
					 nbrDataSetFloat(&msgWeight, prevWeight);

					 }
					 else{
					 //we now don't have a parent after waiting
					 nbrDataSet(&msgParentID, 0);
					 nbrDataSetFloat(&msgWeight, -1.0);
					 prevParentPtr = NULL;
					 }

					 } else {
					 //haven't waited long enough to switch
					 if(bestParentPtr != NULL){
					 if(printNow){
					 distSum = distSum + (bestParentPtr->range);
					 rprintf("ds%d bp%d ",bestParentPtr->range, bestParentPtr->ID);
					 }
					 }
					 if(prevParentPtr!=NULL){
					 //if we had a parent previously, keep info
					 nbrDataSet(&msgParentID, prevParentPtr->ID);
					 nbrDataSetFloat(&msgWeight, prevWeight);
					 if(printNow){
					 //rprintf(" p%d w%d",prevParentPtr->ID, prevWeight);
					 }
					 } else {
					 //if we don't have a parent, continue to be orphaned
					 nbrDataSet(&msgParentID, 0);
					 nbrDataSetFloat(&msgWeight, -1.0);
					 }
					 }
					 if(printNow) switchDelay++;
					 */

					/*** DETERMINE IF PART OF IDEAL TREE PATH ***/
					if (navigationMode != SHOW_TREE_SOURCE) {
						nbrDataSet(&msgShowTree, 0);
						navigationMode = MODE_IDLE;
					}
					else
					{
						nbrDataSet(&msgShowTree, 1);
					}

					partOfTree = FALSE;
					for (i = 0; i < nbrList.size; ++i) {
						nbrPtr = nbrList.nbrs[i];
						if (nbrDataGetNbr(&msgNbrType, nbrPtr) == TRANSPORT) {
							//Ignore the transport robots
							continue;
						}
						if (nbrDataGetNbr(&msgParentID, nbrPtr) == roneID) {
								//nbrDataGetFloat(&msgWeight)	< nbrDataGetNbrFloat(&msgWeight, nbrPtr)) {
							showTree = nbrDataGetNbr(&msgShowTree, nbrPtr);
							if (showTree == 1) {

								nbrDataSet(&msgShowTree, 1);
								partOfTree = TRUE;
								if (navigationMode != SHOW_TREE_SOURCE) {
								navigationMode = SHOW_TREE;
								continue;

								}
								//if show tree mode has been activated on a child, determine if part of path
								//parentID = nbrDataGetNbr(&msgParentID, nbrPtr);
							/*	if (parentID == roneID) {
									//If child in path has chosen me as parent, now part of path
									nbrDataSet(&msgShowTree, 1);
									partOfTree = TRUE;
									if (navigationMode != SHOW_TREE_SOURCE) {
										navigationMode = SHOW_TREE;
									}
								} else {
									//If child is part of path but has not chosen you, get out of path
									if (navigationMode != SHOW_TREE_SOURCE) {
										if (navigationMode != CHANGE_OBJECT_SIZE) {

											navigationMode = MODE_IDLE;

										}
										nbrDataSet(&msgShowTree, 0);
									}
								}*/
							}

						} //end of processing neighbors

							if (partOfTree != TRUE){

								if (navigationMode != SHOW_TREE_SOURCE) {
									if (navigationMode != CHANGE_OBJECT_SIZE) {

										navigationMode = MODE_IDLE;

									}

									nbrDataSet(&msgShowTree, 0);
								}
							}
						}
				} else {
					//is source
					if (navigationMode != SHOW_TREE_SOURCE) {
						navigationMode = SOURCE;
						nbrDataSet(&msgNbrType, 2);

					}
					nbrDataSet(&msgParentID, 0);
					nbrDataSetFloat(&msgWeight, 0.0);
					type = 2; //source is safe by default
				}

			} //else build tree

			/*** ROTATE ***/
			//If unsafe, simply rotate to face parallel to obstacle
			if (type == UNSAFE) {
				bearingToWall = irObstaclesGetBearing();
				bits = irObstaclesGetBits();
				if (bits != 0) {
					bearingToWall = bearingToWall - 3141;

					//if wall is to the right
					if (bearingToWall >= 0) {
						newRV = (bearingToWall - 1571) / 4;
						behSetRv(&behOutput, newRV);
						//if wall is normal to robot (within 10 degrees), stop rotating
						if (abs(newRV) < 100) {
							behSetTvRv(&behOutput, 0, 0);
						}
					} //end right wall
					  //if wall is to the left
					else {
						newRV = (bearingToWall + 1571) /4;
						behSetTvRv(&behOutput, 0, newRV);
						//if wall is normal to robot, stop rotating
						if (abs(newRV) < 100) {
							behSetTvRv(&behOutput, 0, 0);
						}
					} // end left wall
				} // end if bearing != 0
				else {
					behSetTvRv(&behOutput, 0, 0);
					//ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				} //End else no wall
			} else if (type == MINORSAFE && navigationMode >0 && navigationMode != SOURCE) {
				bearingToWall = irObstaclesGetBearing();
				bearingToWall = normalizeAngleMilliRad2(bearingToWall);
				//cprintf("b %d ",bearing);
				bits = irObstaclesGetBits();
				int32 alpha1 = normalizeAngleMilliRad2(bearingToWall - 1571);
				int32 alpha2 = normalizeAngleMilliRad2(bearingToWall + 1571);
				if (bits != 0) {
					if (prevParentPtr) {
						if (nbrDataGetNbr(&msgNbrType, prevParentPtr)== MINORSAFE) {
							int32 bearingParent = nbrGetBearing(prevParentPtr);
							bearingParent = normalizeAngleMilliRad2(
									bearingParent);
							int32 b1 = smallestAngleDifference(alpha1,	bearingParent);
							int32 b2 = smallestAngleDifference(alpha2,	bearingParent);
							//cprintf("a1 %d a2 %d",alpha1, alpha2);
							if (abs(b2) < abs(b1)) {
								//cprintf(" a2");
								newRV = normalizeAngleMilliRad2(alpha2)/4;
							} else {
								//cprintf(" a1");
								newRV = normalizeAngleMilliRad2(alpha1)/4;
							}
							behSetRv(&behOutput, newRV);
							//if wall is normal to robot (within 10 degrees), stop rotating
							if (abs(newRV) < 100) {
								behSetTvRv(&behOutput, 0, 0);
							}
						}

						else if (nbrDataGetNbr(&msgNbrType, prevParentPtr)== SAFE) {
							int32 bearingParent = normalizeAngleMilliRad(nbrGetBearing(prevParentPtr));
							bearingToWall = (irObstaclesGetBearing());
							//	bearing = bearing - 3141;
							if (bearingToWall >= 0 ) {
								alpha1 = normalizeAngleMilliRad(bearingToWall - 1571);
								alpha2 = normalizeAngleMilliRad(bearingToWall - 1571 + 3141);


								//newRV1 = normalizeAngleMilliRad2(normalizeAngleMilliRad2(bearing - 1571)- normalizeAngleMilliRad2(bearingParent)) / 1.5;
								//newRV2 = normalizeAngleMilliRad2(normalizeAngleMilliRad2(bearing + 1571)- normalizeAngleMilliRad2(bearingParent)) / 1.5;

								//behSetRv(&behOutput,newRV);
								//if wall is normal to robot (within 10 degrees), stop rotating
								//if (abs(newRV) < 100){
								//	behSetTvRv(&behOutput, 0, 0);
								//}
							} //end right wall
							  //if wall is to the left
							else {

								alpha1 = normalizeAngleMilliRad(bearingToWall + 1571);
								alpha2 = normalizeAngleMilliRad(bearingToWall + 1571 + 3141);


								//newRV1 = normalizeAngleMilliRad2(normalizeAngleMilliRad2(bearingToWall + 1571)- normalizeAngleMilliRad2(bearingParent)) / 1.5;
								//newRV2 = normalizeAngleMilliRad2(normalizeAngleMilliRad2(bearingToWall- 1571)- normalizeAngleMilliRad2(bearingParent)) / 1.5;
							}
							newRV1 = normalizeAngleMilliRad2(abs(bearingParent - alpha1));
							newRV2 = normalizeAngleMilliRad2(abs(bearingParent - alpha2));
							if (abs(newRV2) < abs(newRV1))
							{								//cprintf(" a2");
								newRV =  normalizeAngleMilliRad2(alpha2) /4;
							} else {
								//cprintf(" a1");

								newRV = normalizeAngleMilliRad2(alpha1) /4;
							}

							//behSetTvRv(&behOutput, 0, newRV);
							//if wall is normal to robot, stop rotating
							//if (abs(newRV) < 100){
							//	behSetTvRv(&behOutput, 0, 0);
							//	}
							//} // end left wall

							if (abs(newRV) < 100) {
								behSetTvRv(&behOutput, 0, 0);
							} else {

								behSetTvRv(&behOutput, 0, newRV);

							}


						} //end of safe parent
						//behSetRv(&behOutput,newRV);
						//if wall is normal to robot (within 10 degrees), stop rotating


					}
					//if no parent, align with wall

				} // end if bearing != 0
				  //no wall is sensed
				else {
					behSetTvRv(&behOutput, 0, 0);
				} //End else no wall
				  //cprintf("\n");
			}
			//If you are safe and not the source, minimize bearing between you and parent
			if (type == SAFE && navigationMode != SOURCE) {
				if (prevParentPtr) {
					behFaceNbr(&behOutput, prevParentPtr);
				} else {
					behOutput = behInactive;
				}
			}

			/*** READ BUTTONS ***/

			if (buttonsGet(BUTTON_RED)) {
				//Show the generated tree
				navigationMode = SHOW_TREE_SOURCE; // the robot that is in the start position
				nbrDataSet(&msgShowTree, 1);
			} else if (buttonsGet(BUTTON_GREEN)) {
				//Set goal robot
				navigationMode = SOURCE; // the robot that is in the goal position
				type = 2; // source is always assumed to be safe
				broadcastMsgSetSource(&broadcastMessage, TRUE);
				nbrDataSetFloat(&msgWeight, 0.0);
			} else if (buttonsGet(BUTTON_BLUE) && navigationMode != SOURCE) {
				/*
				 * Hold blue button for period of time. LED patterns during this time indicate setting.
				 *
				 * RED PULSE - mj 5 bits, mi 200 bits
				 * GREEN PULSE - mj 3 bits, mi 250 bits
				 * BLUE PULSE - (default) mj 2 bits, mi 175 bits
				 * RED CIRCLE - sm weight = 4
				 * GREEN CIRCLE - sm weight = 10
				 * BLUE CIRCLE - sm weight = 20
				 *
				 * If you want to go back and reset a setting, wait for all LEDs to flash.
				 * Once they are dont flashing, you can restart menu selection. If, for example, you want
				 * a sm weight of 10 and an object bit of 3 mj and 250 minor, hold the blue button until the
				 * leds circle in green. Stop, wait for all leds to flash, then hold the blue button again until
				 * the green leds pulse.
				 */
				navigationMode = CHANGE_OBJECT_SIZE;
				rprintf("d%d ", objectSizeDelay);
				objectSizeDelay++;
				if (objectSizeDelay < 50) {
					rprintf("rs");
					majorAxisBits = 250; //35 cm
					minorAxisBits = 400; //15 cm
					//objectSizeDelay = OBJECT_SIZE_WAIT;
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, LED_RATE_FAST);

				} else if (objectSizeDelay < 100) {
					rprintf("gs");
					majorAxisBits = 300; //30 cm
					minorAxisBits = 350; //20 cm
					//objectSizeDelay = OBJECT_SIZE_WAIT;
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, LED_RATE_FAST);

				} else if (objectSizeDelay < 150) {
					rprintf("bs");
					majorAxisBits = 200; //45 cm
					minorAxisBits = 300; //30 cm
					//objectSizeDelay = OBJECT_SIZE_WAIT;

					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, LED_RATE_FAST);

				} else if (objectSizeDelay < 200) {
					rprintf("rw");
					safeMinorWeight = 4;
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, LED_RATE_FAST);

				} else if (objectSizeDelay < 250) {
					rprintf("gw");
					safeMinorWeight = 10;
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, LED_RATE_FAST);

				} else if (objectSizeDelay < 300) {
					rprintf("bw");
					safeMinorWeight = 20;
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, LED_RATE_FAST);
				}
				//rprintf("mj%d mi%d",majorAxisBits,minorAxisBits);
				//rprintf("smw%d",safeMinorWeight);
				//rprintf("d%d\n",objectSizeDelay);
				else if (objectSizeDelay > 0) {
					navigationMode = CHANGE_OBJECT_SIZE;
				} else {
					navigationMode = LEDS;
				}
			}

			switch (navigationMode) {
			case CHANGE_OBJECT_SIZE: {
				/*
				 * ALL LED PULSE --> WAITING FOR OBJECT SIZE CHANGE
				 * ALL LED ON --> WAITING FOR SM TO SAFE WEIGHT CHANGE
				 */
				timer++;
				if (timer > 400) {
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				}
				if (timer > 425) {
					timer = 0;
					navigationMode = LEDS;
					objectSizeDelay = 0;
				}
				break;
			}

			case SOURCE: {
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW,
						LED_RATE_FAST);
				broadcastMsgSetSource(&broadcastMessage, TRUE);
				nbrDataSet(&msgNbrType, 2);
				//if(printNow) rprintf(" source w%d t%d",prevWeight,type);
				break;
			} //end TREE SOURCE
			case SHOW_TREE_SOURCE: {
				//if(printNow){rprintf("tree");}
				//IF the red button was pushed making you the end of the tree (start position)
				if (type == 1) {
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				} else if (type == 2) {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				}

				else {
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				}

				break;
			}
			case SHOW_TREE: {
				//if(printNow){rprintf("tree");}
				if (partOfTree) {
					//You are part of the tree, turn on blue LEDS
					if (type == 1) {
						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
								LED_BRIGHTNESS_LOW, LED_RATE_FAST);
					} else if (type == 2) {
						ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE,
								LED_BRIGHTNESS_LOW, LED_RATE_FAST);
					} else if (type == 0) {
						ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
								LED_BRIGHTNESS_MED, LED_RATE_MED);

					}

				}
				break;

			}
			case MODE_IDLE: {
				//navigationMode = LEDS;
				//do nothing
				break;
			}
			case LEDS: {
				if (type == 1) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, LED_RATE_MED);
				}
				if (type == 2) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE,
							LED_BRIGHTNESS_MED, LED_RATE_MED);
				}
				if (type == 0) {
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
							LED_BRIGHTNESS_MED, LED_RATE_MED);
				}
				break;
			}
			default: {
				//do nothing
				if (printNow) {
					//rprintf(" nothing");
				}
			}

			} //end switch

			/*** FINAL STUFF ***/
			motorSetBeh(&behOutput);
			neighborsPutMutex();

			//end radio printing
			//if (printNow) rprintf("\n");
			if (printNow) {
				printPath = 0;
				if ((navigationMode == SHOW_TREE)
						|| (navigationMode == SHOW_TREE_SOURCE)) {
					printPath = 1;
				}
				printSource = 0;
				if (navigationMode == SOURCE) {
					printSource = 1;
				}
				//Prints
				//nbrDataGetNbrFloat(&msgWeight, prevParentPtr)
				sprintf(tempstrFloat, "%.2f", nbrDataGetFloat(&msgWeight));
				//s = is source, pt = in path, t = safety type, p = parent, w = weight, h = hops
				rprintf("s%d pt%d t%d p%d w%s\n", printSource, printPath, type,
						msgParentID.value, tempstrFloat);
			}
			// delay until the next behavior period
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
			lastWakeTime = osTaskGetTickCount();
		} //end not host
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
