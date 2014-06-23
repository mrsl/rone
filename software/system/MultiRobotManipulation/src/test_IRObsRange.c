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

#define NEIGHBOR_ROUND_PERIOD			2000
#define MOTION_TV_MIN  					0
#define MOTION_TV_DEFAULT				40
#define MOTION_TV_MAX  					120
#define MOTION_TV_STEP					20
#define BUMP_RELECT_DISTANCE			15
#define MAJOR_AXIS						25
#define MAJOR_AXIS_BITS					2
#define MINOR_AXIS						10
#define MINOR_AXIS_BITS					175
#define SWITCH_DELAY					3
#define SAFETY_CHECK_TIME				2

#define MODE_IDLE		 0
#define MODE_BUMPIN		 1
#define SHOW_TREE		 2
#define SOURCE			 3
#define SHOW_TREE_SOURCE 4
#define CHANGE_WIEGHT 	 5


NbrData messagePower;
uint8 ledG;


void rprintNbr(Nbr* nbrPtr) {
	uint8 i;
	if(nbrPtr) {
		// print orientation matrix for user viewing
		cprintf("\n nbr=%d", nbrGetID(nbrPtr));
		irCommsOrientationBitMatrixPrint(nbrPtr->orientationsBitsMatrix);

		//print out processed bearing/orientation
		cprintf("brg=%5d, ort=%5d, rng=%5d\n", nbrGetBearing(nbrPtr), nbrGetOrientation(nbrPtr), nbrGetRange(nbrPtr));

		//print raw nbr orientation matrix
		rprintf(" n,%d,%d", nbrGetID(nbrPtr), nbrDataGetNbr(&messagePower, nbrPtr));
		for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
			rprintf(",%d", nbrPtr->orientationsBitsMatrix[i]);
		}
		/*for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
			rprintf(",%d", nbrPtr->rangeBits[i]);
			if(nbrPtr->rangeBits>ledG) {
				ledG=nbrPtr->rangeBits;
			}
		}*/
	}
}

void navigationLEDsSet(uint8 redCount, uint8 greenCount, uint8 blueCount, uint8 brightness) {
	uint8 redBIN;
	uint8 greenBIN;
	uint8 blueBIN;
	switch (redCount){
		case 0: {
			redBIN = 0;
			break;
		}
		case 1: {
			redBIN = 1;
			break;
		}
		case 2: {
			redBIN = 3;
			break;
		}
		case 3: {
			redBIN = 7;
			break;
		}
		case 4: {
			redBIN = 15;
			break;
		}
		case 5: {
			redBIN = 31;
			break;
		}
	}
	switch (greenCount){
		case 0: {
			greenBIN = 0;
			break;
		}
		case 1: {
			greenBIN = 1;
			break;
		}
		case 2: {
			greenBIN = 3;
			break;
		}
		case 3: {
			greenBIN = 7;
			break;
		}
		case 4: {
			greenBIN = 15;
			break;
		}
		case 5: {
			greenBIN = 31;
			break;
		}
	}
	switch (blueCount){
		case 0: {
			blueBIN = 0;
			break;
		}
		case 1: {
			blueBIN = 1;
			break;
		}
		case 2: {
			blueBIN = 3;
			break;
		}
		case 3: {
			blueBIN = 7;
			break;
		}
		case 4: {
			blueBIN = 15;
			break;
		}
		case 5: {
			blueBIN = 31;
			break;
		}
	}
	ledsSetBinary(redBIN,greenBIN,blueBIN);
}





// public put in header file
/*
typedef struct NbrDataFloat {
	NbrData msgFloat0;
	NbrData msgFloat1;
	NbrData msgFloat2;
	NbrData msgFloat3;
} NbrDataFloat;


// private leave in c file
typedef union FloatConvert {
	float val;
	uint8 bytes[4];
} FloatConvert;


void nbrDataCreateFloat(NbrDataFloat* msgPtr, const char* name) {
	nbrDataCreate(&msgPtr->msgFloat0, name, 8, 0);
	nbrDataCreate(&msgPtr->msgFloat1, name, 8, 0);
	nbrDataCreate(&msgPtr->msgFloat2, name, 8, 0);
	nbrDataCreate(&msgPtr->msgFloat3, name, 8, 0);
}


// pack a 32-bit float into 4 8-bit messages
void nbrDataSetFloat(NbrDataFloat* msgPtr, float val) {
	FloatConvert fc;
	fc.val = val;
	nbrDataSet(&msgPtr->msgFloat0, fc.bytes[0]);
	nbrDataSet(&msgPtr->msgFloat1, fc.bytes[1]);
	nbrDataSet(&msgPtr->msgFloat2, fc.bytes[2]);
	nbrDataSet(&msgPtr->msgFloat3, fc.bytes[3]);
}

// pack a 32-bit float into 4 8-bit messages
float nbrDataGetNbrFloat(NbrDataFloat* msgPtr, Nbr* nbrPtr) {
	FloatConvert fc;
	fc.bytes[0] = nbrDataGetNbr(&msgPtr->msgFloat0, nbrPtr);
	fc.bytes[1] = nbrDataGetNbr(&msgPtr->msgFloat1, nbrPtr);
	fc.bytes[2] = nbrDataGetNbr(&msgPtr->msgFloat2, nbrPtr);
	fc.bytes[3] = nbrDataGetNbr(&msgPtr->msgFloat3, nbrPtr);
	return fc.val;
}

float nbrDataGetFloat(NbrDataFloat* Float) {
	FloatConvert fc;
	fc.bytes[0] = nbrDataGet(&(Float->msgFloat0));
	fc.bytes[1] = nbrDataGet(&(Float->msgFloat1));
	fc.bytes[2] = nbrDataGet(&(Float->msgFloat2));
	fc.bytes[3] = nbrDataGet(&(Float->msgFloat3));
	return fc.val;
}*/

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint8 irObsBits, parentID, parentType;
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput;
	boolean printNow = FALSE;
	boolean printPath = FALSE;
	boolean printSource = FALSE;
	uint8 i,j, showTree;
	uint32 neighborRound = 0;
	NbrList nbrList;
	Nbr* nbrPtr;
	Nbr* bestParentPtr;
	Nbr* prevParentPtr = NULL;
	uint8 switchDelay = 0;

	boolean partOfTree = FALSE;
	char* typeName = "";
	uint8 type = 0;

	int32 ROT_WEIGHT;
	float TRANS_WEIGHT = 2.4;
	uint8 WEIGHT_MOD_ONES;
	uint8 WEIGHT_MOD_DECIMAL;
	boolean BOUNCE_RED = TRUE;
	boolean BOUNCE_BLUE = TRUE;
	int32 tempBear;
	int32 tempOrien;

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


	nbrDataCreate(&msgNbrType, "type", 2, 0);
	nbrDataCreate(&msgParentID, "parentID", 7, 0);
	nbrDataCreate(&msgShowTree, "showTree", 1, 0);
	nbrDataCreateFloat(&msgWeight, "weight");

	uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;
	uint8 avgRound = 0;
	uint16 numBits;

	uint32 avg;
	char tempstrFloat[4];
	char tempBEARING[4];

	int16 bearing;
	int16 newRV = 0;

	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		}//end if host
		else{
			/*** INIT STUFF ***/
			behOutput = behInactive;
			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);

			nbrListCreate(&nbrList);

			irCommsSetXmitPower(IRXmitPower);

			nbrListCreate(&nbrList);


			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			numBits = 0;
			partOfTree = FALSE;


			/*** DETERMINE TYPE IF NOT SOURCE ***/
			if (printNow && (navigationMode != SOURCE)) {
				uint8 sum = 0;

				irObsBits = irObstaclesGetBits();
				numBits = 0;
				avgRound++;

				//Determining number of obs bits and adding to sum
				for (j = 0; j < IR_COMMS_NUM_OF_RECEIVERS; ++j){
					if((irObsBits & 1) == 1){
						numBits++;
					}
					irObsBits = irObsBits>>1;
				}
				sum = sum + numBits;


				//If we have waited enough time, take average and classify robot
				//Assume source is safe
				if(avgRound%SAFETY_CHECK_TIME==0){
					avg = (sum*100)/SAFETY_CHECK_TIME;
					if(avg <= MINOR_AXIS_BITS && avg > MAJOR_AXIS_BITS){
							typeName = "minor safe only";
							type = 1;
							ledsSetPattern(LED_GREEN,LED_PATTERN_CIRCLE,LED_BRIGHTNESS_MED,LED_RATE_MED);
					} else if(avg <= MAJOR_AXIS_BITS){
							typeName = "safe";
							type = 2;
							ledsSetPattern(LED_GREEN,LED_PATTERN_PULSE,LED_BRIGHTNESS_MED,LED_RATE_MED);
					} else{
							typeName = "unsafe";
							type = 0;
							ledsSetPattern(LED_RED,LED_PATTERN_CIRCLE,LED_BRIGHTNESS_MED,LED_RATE_MED);
					}
					//rprintf(" avg %d, type %s",avg, typeName);
					nbrDataSet(&msgNbrType, type);
					sum = 0;
				} //Speciel type for source
				//rprintf(" avg bits %d, type %d \n",avg, type);
			} /*else { //determine type
				nbrDataSet(&msgNbrType, 3);
			}*/

			/*** DECIDE IF TREE SHOULD BE BUILT ***/
			if(type == 0){
				//we are unsafe, stop being in tree - no parent, highest weight possible
				broadcastMessage.active = FALSE;
				nbrDataSet(&msgParentID, 0);
				nbrDataSetFloat(&msgWeight, -1.0);
			} else {

				/*** Choosing Parent ***/
				if (navigationMode != SOURCE){
					float weight = 0.0;
					float prevWeight = 0.0;
					float bestParentWeight = 0.0;
					bestParentPtr = NULL;
					//For all parents in list

					/***********Algorithm 1*****************/

					for (i = 0; i < nbrList.size; i++){
						nbrPtr = nbrList.nbrs[i];
						parentType = nbrDataGetNbr(&msgNbrType, nbrPtr);
						if (broadcastMsgGetSourceIDNbr(&broadcastMessage, nbrPtr)== NULL){
							// this neighbor has no message, continue.
							continue;
						} else {
							//Check if parent is safe or minor safe, also checks to exclude wieghts that are -1 (No weight),
							if ((parentType == 1 || parentType == 2 || parentType == 3) &&(nbrDataGetNbrFloat(&msgWeight, nbrPtr) != -1)){
								weight = nbrDataGetNbrFloat(&msgWeight, nbrPtr);
								if (nbrDataGetNbr(&msgNbrType, nbrPtr) == 2){
									//If it ends in safe Safe
									weight = weight + 1;
								} else if (type == 2){
									//Safe to Minor Safe
									weight = weight + 1.571;
								}else if (type == 1){
									//Minor Safe to Minor Safe
									tempOrien = abs(nbrGetOrientation(nbrPtr));
									if(tempOrien > 1571){tempOrien = 3141- tempOrien;}

									tempBear = abs(nbrGetBearing(nbrPtr));
									if(tempBear > 1571){tempBear = 3141 - tempBear;}

									ROT_WEIGHT = tempBear + tempOrien;
									weight = weight + (1 * TRANS_WEIGHT) + (ROT_WEIGHT /1000.0);
								}else{
									weight = weight + 1;
								}

								if (bestParentPtr==NULL){
									//If we have no parent yet, choose as parent
									bestParentPtr = nbrPtr;
									bestParentWeight = weight;
								} else {
									if(weight < bestParentWeight){
										//If new weight is less then wieght of old parent, make new parent
										bestParentPtr = nbrPtr;
										bestParentWeight = weight;
									} else if(weight == bestParentWeight){
										//If weight equals weight of old parent, choose the one that is safe
										if (nbrDataGetNbr(&msgNbrType, bestParentPtr) == 1){
											bestParentPtr = nbrPtr;
											bestParentWeight = weight;
										}
									}//end else weight ==
								}//end else bestparent != null
							}//end if parent is safe and  in  tree
						}
					} //endfor checking all potential parents



					/*** DETERMINE IF SWITCH ALLOWED ***/
					if(printNow) switchDelay++;
					if(switchDelay == SWITCH_DELAY){
						//if we've waited long enough to switch
						switchDelay = 0;
						if(bestParentPtr != NULL){
							//if we have a parent selected
							prevParentPtr = bestParentPtr;
							prevWeight = (nbrDataGetNbrFloat(&msgWeight, bestParentPtr));
							parentType = nbrDataGetNbr(&msgNbrType, bestParentPtr);

							if (nbrDataGetNbr(&msgNbrType, bestParentPtr) == 2){
								//If it ends in safe Safe
								prevWeight = prevWeight + 1;
							} else if (type == 2){
								//Safe to Minor Safe
								prevWeight = prevWeight + 1.571;
							}else if (type == 1){
								//Minor Safe to Minor Safe
								tempOrien = abs(nbrGetOrientation(bestParentPtr));
								if(tempOrien > 1571){tempOrien = 3141- tempOrien;}

								tempBear = abs(nbrGetBearing(bestParentPtr));
								if(tempBear > 1571){tempBear = 3141 - tempBear;}

								ROT_WEIGHT = tempBear + tempOrien;
								prevWeight = prevWeight + (1 * TRANS_WEIGHT) + (ROT_WEIGHT /1000.0);
							}else{
								prevWeight = prevWeight + 1;
							}

							//Set parent ID and new weight
							nbrDataSet(&msgParentID, bestParentPtr->ID);
							nbrDataSetFloat(&msgWeight, prevWeight);

							//error in weight decision, should never be this high
							if(prevWeight > 1000){
								nbrDataSet(&msgParentID, 0);
								nbrDataSetFloat(&msgWeight, -1.0);
								prevParentPtr = NULL;
							}

							if(printNow){
								//rprintf(" sp%d w%d",prevParentPtr->ID, prevWeight);
							}
						} else{
							//we now don't have a parent after waiting
							nbrDataSet(&msgParentID, 0);
							nbrDataSetFloat(&msgWeight, -1.0);
							prevParentPtr = NULL;
							if(printNow){
								//rprintf(" no sp");
							}
						}

					} else {
						//haven't waited long enough to switch
						if(prevParentPtr!=NULL){
							//if we had a parent previously, keep info
							nbrDataSet(&msgParentID, prevParentPtr->ID);
							//nbrDataSetFloat(&msgWeight, prevWeight);
							if(printNow){
								//rprintf(" p%d w%d",prevParentPtr->ID, prevWeight);
							}
						} else {
							//if we don't have a parent, continue to be orphaned
							nbrDataSet(&msgParentID, 0);
							nbrDataSetFloat(&msgWeight, -1.0);
							if(printNow){
								//rprintf(" no p");
							}
						}
					}


					/*** DETERMINE IF PART OF IDEAL TREE PATH ***/
					if (navigationMode != SHOW_TREE_SOURCE){
						nbrDataSet(&msgShowTree, NULL);
						if(navigationMode != CHANGE_WIEGHT){
							navigationMode = MODE_IDLE;
						}
					}
					//rprintf("niebors: ");
					for (i = 0; i < nbrList.size; ++i) {
						nbrPtr = nbrList.nbrs[i];
						//rprintf("id: %d ",nbrPtr->ID);
						if(nbrDataGetFloat(&msgWeight) < nbrDataGetNbrFloat(&msgWeight, nbrPtr) ){
							showTree = nbrDataGetNbr(&msgShowTree, nbrPtr);
							if(showTree == 1){
								//if show tree mode has been activated on a child, determine if part of path
								parentID = nbrDataGetNbr(&msgParentID, nbrPtr);
								if (parentID == roneID) {
									//If child in path has chosen me as parent, now part of path
									nbrDataSet(&msgShowTree, 1);
									partOfTree = TRUE;
									if (navigationMode != SHOW_TREE_SOURCE){
										if(navigationMode != CHANGE_WIEGHT){
											navigationMode = SHOW_TREE;
										}
									}
								}
							}

						}
					} //end of processing neighbors
					//rprintf("\n");
				}else {
					//is source
					if ((navigationMode != SHOW_TREE_SOURCE)||(navigationMode != CHANGE_WIEGHT)){
						navigationMode = SOURCE;
					}
					nbrDataSet(&msgParentID, NULL);
					nbrDataSetFloat(&msgWeight, 0.0);
					type = 2; //source is safe by default
				}

			}//else build tree

			/*** ROTATE ***/

			/*** READ BUTTONS ***/
			if(navigationMode != CHANGE_WIEGHT){
				if (!buttonsGet(BUTTON_RED)) {
					BOUNCE_RED = TRUE;
				}
				if (buttonsGet(BUTTON_RED)) {
					if(BOUNCE_RED){
						BOUNCE_RED = FALSE;
						navigationMode = CHANGE_WIEGHT;
					}
				}else if (buttonsGet(BUTTON_GREEN)) {
					//Set goal robot
					navigationMode = SOURCE;
					broadcastMsgSetSource(&broadcastMessage, TRUE);
					nbrDataSetFloat(&msgWeight, 0.0);
				} else if (buttonsGet(BUTTON_BLUE)) {
					//Show the generated tree
					navigationMode = SHOW_TREE_SOURCE;
					nbrDataSet(&msgShowTree, 1);
				}
			}

			switch (navigationMode) {

			case SOURCE: {
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				broadcastMsgSetSource(&broadcastMessage, TRUE);
				//if(printNow) rprintf(" source w%d t%d",prevWeight,type);
				break;
			} //end TREE SOURCE
			case SHOW_TREE_SOURCE: {
				broadcastMsgSetSource(&broadcastMessage, FALSE);

				//if(printNow){rprintf("tree");}
				//IF the red button was pushed making you the begining of the tree
				if(type==1){
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				}
				break;
			}
			case SHOW_TREE: {
				broadcastMsgSetSource(&broadcastMessage, FALSE);

				//if(printNow){rprintf("tree");}
				if(partOfTree){
					//You are part of the tree, turn on blue LEDS
					if(type==1){
						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
					} else {
						ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
					}
				}
				break;

			}
			case CHANGE_WIEGHT: {
				//ledsSetPattern(LED_ALL, LED_PATTERN_BLINK, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				WEIGHT_MOD_ONES = TRANS_WEIGHT;
				WEIGHT_MOD_DECIMAL = (TRANS_WEIGHT*10 - WEIGHT_MOD_ONES * 10)/2;
				navigationLEDsSet(0,WEIGHT_MOD_ONES, WEIGHT_MOD_DECIMAL, LED_BRIGHTNESS_LOW);
				if(!buttonsGet(BUTTON_BLUE)) {
					BOUNCE_BLUE = TRUE;
				}
				if(buttonsGet(BUTTON_BLUE)) {
					if(BOUNCE_BLUE){
						TRANS_WEIGHT = TRANS_WEIGHT + .201;
						BOUNCE_BLUE = FALSE;
					}
					if(TRANS_WEIGHT > 6){
						TRANS_WEIGHT = 1;
					}
				}
				if (!buttonsGet(BUTTON_RED)) {
					BOUNCE_RED = TRUE;
				}
				if (buttonsGet(BUTTON_RED)) {
					if(BOUNCE_RED){
						navigationMode = MODE_IDLE;
						BOUNCE_RED = FALSE;
					}
				}
				break;
			}
			case MODE_IDLE: {
				//do nothing
				break;
			}
			default:{
				//do nothing
				if(printNow){
					//rprintf(" nothing");
				}
			}

			}//end switch

			/**** ADJUST TO WALL *****/

			bearing = irObstaclesGetBearing();
			behOutput = behInactive;
			if(bearing != 0){
				bearing = bearing - 3141;
				//if wall is to the right
				if(bearing >= 0){
					newRV = (bearing - 1571)/ 1.5;
					//cprintf(" rv = %d\n",newRV);
					behSetTvRv(&behOutput, 0, newRV);
					//if wall is apporzimatly 90 degrees
					if (abs(newRV) < 250){
						behSetTvRv(&behOutput, 0, 0);
					}
				}//end right wall
				//if wall is to the left
				else{
					newRV = (bearing + 1571) / 1.5;
					//cprintf(" rv = %d\n",newRV);
					behSetTvRv(&behOutput, 0, newRV);
					//if wall is apporximatly 90 degrees
					if (abs(newRV) < 250){
						behSetTvRv(&behOutput, 0, 0);
					}
				} // end left wall
			} else {
				behSetTvRv(&behOutput, 0, newRV);
			}

			/*** FINAL STUFF ***/
			if(type == 1 && navigationMode != SOURCE){
				motorSetBeh(&behOutput);
			}
			else{
				behOutput = behInactive;
				motorSetBeh(&behOutput);
			}
			neighborsPutMutex();

			//end radio printing
			//if (printNow) rprintf("\n");
			if (printNow){

				printPath = 0;
				if ((navigationMode == SHOW_TREE) || (navigationMode == SHOW_TREE_SOURCE) ){
					printPath = 1;
				}
				printSource = 0;
				if(navigationMode == SOURCE){
					printSource = 1;
				}
				//Prints
				//nbrDataGetNbrFloat(&msgWeight, prevParentPtr)
				sprintf(tempstrFloat,"%.2f",nbrDataGetFloat(&msgWeight));
				//s = is source, pt = in path, t = safety type, p = parent, w = weight, h = hops
				rprintf("s%d pt%d t%d p%d w%s h%d\n",printSource,printPath, type,msgParentID.value,tempstrFloat, broadcastMsgGetHopsNbr(&broadcastMessage,prevParentPtr)+1);

			}
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
