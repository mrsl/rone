/*
 * @file neighbors.c
 * @brief used to maintain information about network neighbors, sets up data storage and callbacks
 * @since Mar 2, 2011
 * @author: jamesm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "roneos.h"
#include "neighborsInternal.h"
//#include "../src/crc/crctest.h"	//TESTING



/******** Defines ********/
#define NEIGHBORS_IGNORE_LIST_SIZE	8
#define RECIEVE_CALLBACK_SIZE		8

// the mask limits the signal bits to only compare the last few neighbor cycles
//#define NBR_SIGNALBITS_MASK 					0x0F
#define NBR_SIGNALBITS_ROUNDS_DEFAULT			4
#define NBR_SIGNALBITS_MIN_ACTIVE_DEFAULT 		2
#define NBR_SIGNALBITS_MAX_INACTIVE_DEFAULT 	0

#define NEIGHBOR_RANGE_IIR_GAIN					50
#define NEIGHBOR_ANGLES_IIR_GAIN				75

//### range bits moving average
//#define RBMA_PERIOD 3


/******** Functions ********/
static void neighborsTask(void* parameters);


/******** Variables ********/
static uint32 neighborPeriod = NEIGHBOR_PERIOD_DEFAULT;
//static uint32 neighbor_timeout = NEIGHBOR_PERIOD_DEFAULT * NEIGHBOR_TIMEOUT_ROUNDS;
static uint8 nbrSignalBitsRounds;
static uint8 nbrSignalBitsMask;
static uint8 nbrSignalBitsMinActive;
static uint8 nbrSignalBitsMaxInactive;
static uint32 obstacle_timeout = OBSTACLE_TIMEOUT_ROUNDS;
static boolean neighborXmitEnable = TRUE;
static boolean neighbor_recieve_enable = TRUE;
static IRRangeData obstacleData;

static NbrDatabase nd;
static void(*receiveCallbacks[RECIEVE_CALLBACK_SIZE])(NbrDatabase* ndPtr) = {0};
static osSemaphoreHandle neighborsMutex;

static uint8 neighborIgnoreCount = 0;
static uint8 neighborsIgnoreList[NEIGHBORS_IGNORE_LIST_SIZE];
static int32 nbrAnglesIIRTimeConstant = NEIGHBOR_ANGLES_IIR_GAIN;
static int32 nbrRangeIIRTimeConstant = NEIGHBOR_RANGE_IIR_GAIN;

#ifdef RONE_IRBEACON
static boolean beaconHighPowerID = FALSE;
#endif

/* This is used to set the neighbor period when new neighbor messages are added. */
//TODO make an API to access this
//extern uint8 irCommsMessageBitCount;

/******** Functions ********/



/*
 * @defgroup <Neighbor System>
 * @{
 * 	Neighbor system allows a robot to communicate with its neighbors.\n
 * 	Function neighborsTask is performed constantly at every WHAT.\n
 *
 * }@
 */





/*
 * @brief Disable neighbor xmit/recv.
 *
 * @returns void
 */
void neighborsDisable(void) {
	neighborXmitEnable = FALSE;
	neighbor_recieve_enable = FALSE;
}


/*
 * @brief Enable neighbor to transmit messages.
 *
 * @param neighbor_xmit_enable_arg a boolean that allows enable or not
 * @returns void
 */
void neighborsXmitEnable(boolean neighbor_xmit_enable_arg) {
	neighborXmitEnable = neighbor_xmit_enable_arg;
}


/*
 * @brief Initialize neighbors and start neighbors task.
 *
 * Initializes neighbor period, neighbor timeout, obstacle timeout. Initialize neighborData
 * Sets message length.
 * Puts 7-bit roneID in message.
 * Semaphore implementing neighborsMutex created.
 * @param neighbor_period_arg the neighbor period in rounds
 * @returns void
 */
void neighborsInit(uint32 neighbor_period_arg) {
	uint32 val;
	neighborsSetPeriod(neighbor_period_arg);
	neighborsSetTimeoutRounds(NBR_SIGNALBITS_ROUNDS_DEFAULT, NBR_SIGNALBITS_MIN_ACTIVE_DEFAULT, NBR_SIGNALBITS_MAX_INACTIVE_DEFAULT);

	nd.nbrsSize = 0;
	nd.round = 0;

	nbrDataCreateIR(&nbrMsgID, "ID", 7, roneID);
	neighborsMutex = osSemaphoreCreateMutex();
	osTaskCreate(neighborsTask, "neighbors", 2048, NULL, NEIGHBORS_TASK_PRIORITY );
}


/*
 * @brief Set neighbor period, neighbor timeout, and obstacle timeout proportional to argument.
 *
 * @param neighbor_period_arg the neighbor period length in rounds
 * @returns void
 */
void neighborsSetPeriod(uint32 neighbor_period_arg) {
	neighborPeriod = neighbor_period_arg;
	obstacle_timeout = OBSTACLE_TIMEOUT_ROUNDS;
}


/*
 * @brief Set time constant of neighbor IIR filters
 *
 * @param FIlter time constant. max is 100.  Is based on x/100.  10 is slow, 90 is fast
 * @returns void
 */
void neighborsSetFilterTimeConstants(int32 anglesTimeConstant, int32 rangeTimeConstant) {
    nbrAnglesIIRTimeConstant = bound(anglesTimeConstant, 0, 100);
    nbrRangeIIRTimeConstant = bound(rangeTimeConstant, 0, 100);
}


/*
 * @brief Set neighbor period, neighbor timeout, and obstacle timeout proportional to argument.
 *
 * @param neighbor_period_arg the neighbor period length in rounds
 * @returns void
 */
void neighborsSetTimeoutRounds(uint8 timeoutRounds, uint8 minActive, uint8 maxInactive) {
	uint8 i;
	nbrSignalBitsMask = 0;

	timeoutRounds = bound(timeoutRounds, 1, 8);
	for (i = 0; i < timeoutRounds - 1; ++i) {
		nbrSignalBitsMask |= 1;
		nbrSignalBitsMask <<= 1;
	}
	nbrSignalBitsMask |= 1;
	nbrSignalBitsMinActive = minActive;
	nbrSignalBitsMaxInactive = maxInactive;
	//neighbor_timeout = neighbor_period * neighbor_timeout_arg;
}


/*
 * @brief Tries add neighborID to list of neighbors to ignore
 *
 * @param neighborID the neighbor we want to ignore (no longer monitor)
 * @returns void
 */
void neighborsIgnore(uint8 neighborID) {
	if (neighborIgnoreCount < NEIGHBORS_IGNORE_LIST_SIZE) {
		neighborsIgnoreList[neighborIgnoreCount++] = neighborID;
	}
}


/*
 * @brief Get neighbor period.
 *
 * @returns void
 */
uint32 neighborsGetPeriod(void) {
	return neighborPeriod;
}

void neighborsAddReceiveCallback(void(*receiveCallbackArg)(NbrDatabase* ndPtr)) {
	uint8 i;

	for (i = 0; i < RECIEVE_CALLBACK_SIZE; ++i) {
		if(receiveCallbacks[i] == NULL) {
			receiveCallbacks[i] = receiveCallbackArg;
			break;
		}
	}
	if (i == RECIEVE_CALLBACK_SIZE) {
		error("too many receive callbacks");
	}
}


/*
 * @brief Print information on neighbor (and information of neighbor's neighbors).
 *
 * Print roneID and neighbor's ID, bear, orientation, orientation valid
 * Print name and value of each neighbor message.
 * @param nbrPtr neighbor pointer
 * @returns void
 */
void nbrPrint(Nbr* nbrPtr) {
	uint8 i;
	NbrData* nbrMsgPtr;
	char bitString[18];

	if (nbrPtr) {
		cprintf("%2d:nbr%2d brg %5d, ont %5d, range %d\n",
				roneID,
				nbrGetID(nbrPtr),
				nbrGetBearing(nbrPtr),
				nbrGetOrientation(nbrPtr),
				nbrGetRange(nbrPtr));
		irCommsOrientationBitMatrixPrint(nbrPtr->orientationsBitsMatrix);
		cprintf("bitCount=%d\n", nbrGetOrientationBitsCount(nbrPtr));
		cprintf("brgBits=%s\n", bitString8(bitString, nbrGetReceiverBits(nbrPtr)));
		cprintf("ortBits=%s\n", bitString8(bitString, nbrGetTransmitterBits(nbrPtr)));
		cprintf("rngBits=%s\n", bitString16(bitString, nbrGetRangeBits(nbrPtr)));
	}
}


/*
 * @brief Print the obstacle data from the IR sensors.
 *
 * @returns void
 */
void obstaclePrint(void) {
	char string[18];

    cprintf("obstacle %d: brgBits %s rngbitCount %d\n", roneID, bitString8(string, obstacleData.receiverBits), obstacleData.rangeBitCount);
	cprintf("rngBits %s%s\n", bitString16(string, obstacleData.rangeBits));
	irCommsOrientationBitMatrixPrint(obstacleData.orientationsBitsMatrix);
}


/*
 * @brief Print header and neighbor data.
 *
 * Print header once.
 * Print id, time, round; neighbor's ID, bearing, update time; neighbor's neighbor's ID, bearing, update time.
 * @param nbrPtr neighbor pointer
 * @param round the round number
 * @returns void
 */
void nbrPrintData(Nbr* nbrPtr, uint32 round) {
	uint8 i;
	static boolean printedHeader = FALSE;
	if (!printedHeader) {
		cprintf("id,time,round,nbrID,bearing,updateTime\n");
		printedHeader = TRUE;
	}
	cprintf("nd,%d,%d,%d,%d,%d,%d",
			roneID,osTaskGetTickCount(),round, nbrPtr->ID, nbrPtr->bearing, nbrPtr->updateTime);
	cprintf("\n");
}


static boolean neighborsIgnoreCheck(uint8 neighborID) {
	uint8 i;

	for (i = 0; i < neighborIgnoreCount; ++i) {
		if (neighborsIgnoreList[i] == neighborID){
			return TRUE;
		}
	}
	return FALSE;
}


static int16 angleHistoryVote(int16 angleHistory[]) {
	int16 diff01 = abs(smallestAngleDifference(angleHistory[0], angleHistory[1]));
	int16 diff12 = abs(smallestAngleDifference(angleHistory[1], angleHistory[2]));
	int16 diff02 = abs(smallestAngleDifference(angleHistory[0], angleHistory[2]));

	if ((diff01 < NBR_ANGLE_MAX_DELTA_PER_ROUND) || (diff02 < (NBR_ANGLE_MAX_DELTA_PER_ROUND * 2)))  {
		// angleHistory[0] is within threshold of ah[1] or ah[2].  It's probably correct.  Return it.
		return angleHistory[0];
	} else {
		// angleHistory[0] is far from the previous values.  Maybe we missed a packet.  Return the previous value angleHistory[1]
		return angleHistory[1];
	}
}



// called every time an IR receiver gets a message.
// classifies the message as either an obstacle, or data about a neighbor
static void processNbrMessage(IRCommsMessage* irMsgPtr) {
	uint8 nbrID, i;
	Nbr* nbrPtr;
	boolean newNbr = FALSE;
//	uint8 receiverBits = irMsgPtr->receiverBits;	// the IR receiver(s) that detected this message
//	uint8 orientationBits = irMsgPtr->orientationBits; // the IR transmitter(s) that sent this message
	uint8 orientationBits;

//	if (receiverBits != 0) {
		nbrID = neighborMessageUnpackID(irMsgPtr->data);
		if (neighborsIgnoreCheck(nbrID)) {
			// this neighbor is on the ignore list.  Ghost neighbor.  Ignore.
			return;
		}
		if (nbrID == roneID) {
			// you have received your own neighbor message.  Process it as an obstacle.
			obstacleData.receiverBits = irMsgPtr->receiverBits;
			obstacleData.rangeBitCount = 0;
			for (i = 0; i < IR_COMMS_NUM_OF_TRANSMITTERS; ++i) {
				orientationBits = irMsgPtr->orientationBitMatrix[i];
				obstacleData.orientationsBitsMatrix[i] = orientationBits;
				obstacleData.rangeBits = irMsgPtr->rangeBits;
				if (orientationBits) {
					obstacleData.rangeBitCount += bitsCount(orientationBits);
				}
			}
			obstacleData.updateRound = nd.round;
			return;
		}
		nbrPtr = nbrsGetWithID(nbrID);
		if (!nbrPtr) {
			// Couldn't find the nbr on the list. Make a new one if there is room
			if (nd.nbrsSize < NEIGHBOR_MAX) {
				nbrPtr = &nd.nbrs[nd.nbrsSize++];
				nbrPtr->ID = nbrID;
				nbrPtr->orientation = 0;
				nbrPtr->orientationValid = FALSE;
				nbrPtr->signalBits = 0;
				nbrPtr->range = 0;
				nbrPtr->active = FALSE;
				nbrMsgRadioAddNbr(nbrPtr->ID);
				newNbr = TRUE;
			}
		}
		if (nbrPtr) {
			// we run this code for all neighbors, a new one or an old one to update
			nbrDataIRMessageUnpack(irMsgPtr, nbrPtr);
			nbrPtr->updateTime = irMsgPtr->timeStamp;
			nbrPtr->updateRound = nd.round;

			// or the orientation and receive bits down to a row and col vector for orientation and bearing
			nbrPtr->receiverBits = irMsgPtr->receiverBits;
			uint8 tempOrientationBits  = 0;
			for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
				orientationBits = irMsgPtr->orientationBitMatrix[i];
				nbrPtr->orientationsBitsMatrix[i] = orientationBits;
				tempOrientationBits |= orientationBits;
			}
			// see if you received any orientation information.  If so, update the orientationBits
			if (tempOrientationBits != 0) {
				nbrPtr->orientationBits = tempOrientationBits;
				nbrPtr->orientationValid = TRUE;
			} else {
				nbrPtr->orientationValid = FALSE;
			}

			//### compute range!
			nbrPtr->rangeBits = irMsgPtr->rangeBits;
			uint16 range = irCommsComputeNbrRange(nbrPtr->rangeBits);

			// Filter the range with a IIR
			//cprintf("@@@ %d\t%d", nbrPtr->range, range);
			//nbrPtr->range = filterIIR(nbrPtr->range, range, RANGE_IIR);
			//cprintf("\t%d\t\t%d\t%d\n", nbrPtr->range, nbrPtr->rangeBits, nbrPtr->ID);
			//nbrPtr->range = range;
			//nbrPtr->range = ((uint16)(nbrPtr->range) + range) >> 1;
			//nbrPtr->range = nbrPtr->range - 1;

			// compute bearing and orientation
			// shift history back
			for (i = (NBR_ANGLE_HISTORY - 1); i > 0 ; i--) {
				nbrPtr->bearingHistory[i] = nbrPtr->bearingHistory[i-1];
				nbrPtr->orientationHistory[i] = nbrPtr->orientationHistory[i-1];
			}
			nbrPtr->bearingHistory[0] = angleFromBitVectorOffset(bitsMaxContiguous(nbrPtr->receiverBits));
			if (nbrIsBeacon(nbrPtr)) {
				nbrPtr->orientationHistory[0] = angleFromBitVectorBeacon(bitsMaxContiguous(nbrPtr->orientationBits));
			} else {
				nbrPtr->orientationHistory[0] = angleFromBitVector(bitsMaxContiguous(nbrPtr->orientationBits));
			}
			nbrPtr->orientationValid = TRUE;

			// vote on the bearing and orientation
			int16 bearing = angleHistoryVote(nbrPtr->bearingHistory);
			int16 orientation = angleHistoryVote(nbrPtr->orientationHistory);

			// Filter the angles and range with a IIR
			if (newNbr) {
				nbrPtr->range = range;
				nbrPtr->bearing = bearing;
				nbrPtr->orientation = orientation;
			} else {
				nbrPtr->range = filterIIR(nbrPtr->range, range, nbrRangeIIRTimeConstant);
				nbrPtr->bearing = filterIIRAngle(nbrPtr->bearing, bearing, nbrAnglesIIRTimeConstant);
				nbrPtr->orientation = filterIIRAngle(nbrPtr->orientation, orientation, nbrAnglesIIRTimeConstant);
			}
			nbrPtr->signalBits |= 1;
		}
//	}
}


uint8 irObstaclesGetBits(void) {
    return obstacleData.receiverBits;
}

uint32 irObstaclesGetRangeBits(void) {
    return obstacleData.rangeBits;
}


uint8* irObstaclesGetBitMatrix(void) {
    return obstacleData.orientationsBitsMatrix;
}


int8 irObstaclesGetBearingBitVector(void) {
	uint8 obstacleBits = irObstaclesGetBits();
	uint8 obstacleBitsGroup = bitsMaxContiguous(obstacleBits);

	return obstacleBitsGroup;
}


int16 irObstaclesGetBearing(void) {
	uint8 obstacleBitsGroup = irObstaclesGetBearingBitVector();
	int16 obstacleBearing = 0;

	if (obstacleBitsGroup != 0) {
		// compute the bump sensor bearing
		obstacleBearing = angleFromBitVectorOffset(obstacleBitsGroup);
	}
	return normalizeAngleMilliRad(obstacleBearing);
}


void obstacleExcludeNbrs(NbrList* nbrListPtr, uint8* obstacleBitsGroupPtr, uint8* obstacleBitsCountPtr, int16* obstacleBearingPtr) {
	uint8 obstacleBits = irObstaclesGetBits();
	uint8 nbrBearingBits;
	uint8 i;
	Nbr* nbrPtr;

	// compute the union of all the nbr bearing bit vectors
	nbrBearingBits = 0;
	for (i = 0; i < nbrListPtr->size; i++) {
		nbrPtr = nbrListPtr->nbrs[i];
		nbrBearingBits |= nbrPtr->receiverBits;
	}

	// subtract the nbr bearing bits from the obstacle bits
	obstacleBits &= ~nbrBearingBits;

	// process to find obstacles - the number of bits reported might be less, but hopefully more acurate
	*obstacleBitsGroupPtr = bitsMaxContiguous(obstacleBits);
	*obstacleBitsCountPtr = bitsCount(*obstacleBitsGroupPtr);
	*obstacleBearingPtr = normalizeAngleMilliRad(angleFromBitVectorOffset(*obstacleBitsGroupPtr));
}


/*
 * @brief Get neighbors mutex.
 *
 * @returns void
 */
void neighborsGetMutex(void) {
	osSemaphoreTake(neighborsMutex, portMAX_DELAY);
}


/*
 * @brief Put neighbors mutex.
 *
 * @returns void
 */
void neighborsPutMutex(void) {
	osSemaphoreGive(neighborsMutex);
}


uint32 xmitDelay=0;
uint32 xmitDelay2=0;


/*
 * @brief The neighbor update system task
 *
 * @returns void
 */
static void neighborsTask(void* parameters) {
	portTickType lastWakeTime, currentTime;
	IRCommsMessage IRMsg;
	uint8 i, j;
	lastWakeTime = osTaskGetTickCount();
	Nbr* nbrPtr;
	uint8 signalBitsCount;
	time_t timer = time(NULL);

	for (;;) {
		// process all the stored IR messages
		if (neighbor_recieve_enable) {
			neighborsGetMutex();
			currentTime = osTaskGetTickCount();

			for (i = 0; i < nd.nbrsSize; i++) {
				// shift the signal bits over
				nd.nbrs[i].signalBits = (nd.nbrs[i].signalBits << 1) & nbrSignalBitsMask;

				// set the new nbrround flag

			}

			// process the new IR messages
			while(irCommsGetMessage(&IRMsg)) {
				processNbrMessage(&IRMsg);
			}

			// make neighbors active or inactive based on the recent message activity
			for (i = 0; i < nd.nbrsSize; ) {
				nbrPtr = &nd.nbrs[i];
				signalBitsCount = bitsCount(nbrPtr->signalBits);
				if(signalBitsCount >= nbrSignalBitsMinActive) {
					nbrPtr->active = TRUE;
					i++;
				} else if ((signalBitsCount < nbrSignalBitsMaxInactive) && (signalBitsCount > 0)){
					nbrPtr->active = FALSE;
					i++;
				} else if (signalBitsCount == 0) {
					// remove the neighbor
					nbrMsgRadioRemoveNbr(nd.nbrs[i].ID);
					nd.nbrsSize--;
					if (i < nd.nbrsSize) {
						nd.nbrs[i] = nd.nbrs[nd.nbrsSize];
					}
				} else {
					i++;
				}
			}

			// clear the obstacle bits if they have not been updated in a while
			if(nd.round > (obstacleData.updateRound + obstacle_timeout)) {
				obstacleData.rangeBits = 0;
				for (i = 0; i < IR_COMMS_NUM_OF_TRANSMITTERS; ++i) {
					obstacleData.orientationsBitsMatrix[i] = 0;
				}
				obstacleData.receiverBits = 0;
			}

			// call any user-defined callbacks.  The NbrNbr system, and external pose system use this
			for (i = 0; i < RECIEVE_CALLBACK_SIZE; ++i) {
				if (receiveCallbacks[i] == NULL) {
					break;
				} else {
					(receiveCallbacks[i])(&nd);
				}
			}

			nd.round++;
			neighborsPutMutex();
		}

		if(neighborXmitEnable) {
			// wait a random offset before xmit
			if (neighborPeriod > NEIGHBOR_XMIT_MIN_DELAY) {
				//xmitDelay = (uint32)rand() % (neighbor_period - NEIGHBOR_XMIT_MIN_DELAY);
				xmitDelay = pseudoRandNumGen(roneID, roneID) % (neighborPeriod - NEIGHBOR_XMIT_MIN_DELAY);
				osTaskDelay(xmitDelay);
			}

			#ifdef RONE_IRBEACON
			if(beaconHighPowerID) {
				nbrDataSet(&nbrMsgID, roneID - 1);
			} else {
				nbrDataSet(&nbrMsgID, roneID);
			}
			irCommsBeaconHighPower(beaconHighPowerID);

			// toggle the high power beacon ID
			beaconHighPowerID = 1 - beaconHighPowerID;
			#endif

			neighborsGetMutex();
			// send the neighbor message
			nbrDataIRMessagePack(&IRMsg);
			irCommsSendMessage(&IRMsg);

			// send the neighbor radio messages
			nbrMsgRadioXmit();

			neighborsPutMutex();
		}
		currentTime = osTaskGetTickCount();
		if (currentTime < (lastWakeTime + neighborPeriod)) {
			osTaskDelayUntil(&lastWakeTime, neighborPeriod);
		}
	}
}


/*
 * Pseudo Randomnumber generator using LCG (Linear congruential generator).
 * Tuned to generate between 0 to about 4000.
 *	Ref) http://en.wikipedia.org/wiki/Linear_congruential_generator
 *	By SeoungKyou Lee.
**/
static unsigned long nxtRandSeed=1;
static boolean isInit = FALSE;
uint32 pseudoRandNumGen(uint32 init, uint32 seed){
	uint32 randMax = RAND_MAX/10;
	if (isInit==FALSE){
		nxtRandSeed = seed * time(NULL);
		isInit = TRUE;
	}
	nxtRandSeed = (((nxtRandSeed *init)%randMax) *init)%randMax + init;
	uint32 res = (uint32)(nxtRandSeed/(2*(randMax/10000)));
	res %= (randMax/10000);
	return res;
}

/*
 * @brief Get neighbor round from neighbor data.
 *
 * @return neighbor round
 */
uint32 neighborsGetRound(void) {
	return(nd.round);
}


/*
 * @brief Check to see if there is a new neighbor round.  Updates the variable at the pointer.
 *
 * @param pointer for previous round
 * @return TRUE if the neighbor round has changed
 */
boolean neighborsNewRoundCheck(uint32* roundOldPtr) {
	boolean val = FALSE;
	if (nd.round != *roundOldPtr) {
		val = TRUE;
	}
	*roundOldPtr = nd.round;
	return val;
}


Nbr* nbrsGetWithID(uint8 nbrID) {
	uint8 i;
	for (i = 0; i < nd.nbrsSize; i++) {
		if (nd.nbrs[i].ID == nbrID) {
			return &nd.nbrs[i];
		}
	}
	return NULL;
}


// fills the provided nbrList with the current set of neighbors
void nbrListCreate(NbrList* nbrListPtr) {
	uint8 i;
	nbrListPtr->size = 0;
	for (i = 0; i < nd.nbrsSize; i++) {
		if (nd.nbrs[i].active) {
			nbrListPtr->nbrs[nbrListPtr->size++] = &nd.nbrs[i];
		}
	}
}


// sets the size of nbrList to 0
void nbrListClear(NbrList* nbrListPtr) {
	nbrListPtr->size = 0;
}


uint8 nbrListGetSize(NbrList* nbrListPtr) {
	return(nbrListPtr->size);
}


Nbr* nbrListGetNbr(NbrList* nbrListPtr, uint8 idx) {
	if (idx < nbrListPtr->size) {
		return nbrListPtr->nbrs[idx];
	} else {
		return NULL;
	}
}



void nbrListPrint(NbrList* nbrListPtr, char* name) {
	uint8 i;

	cprintf("%s={", name);
	for (i = 0; i < nbrListPtr->size; i++) {
		if((nbrListPtr->size > 1) && (i > 0)) {
			cprintf(",");
		}
		cprintf("%02d", nbrListPtr->nbrs[i]->ID);
	}
	cprintf("} \n");
}


/*
 * @brief Prints the IDs and update times for all robots in the neighbor list.
 *
 * Formated as follows:
 * {name, ID updateTime, ID updateTime, ...}
 * @param nbrListPtr pointer for the neighborlist
 * @param name for the list name
 * @returns void
 */
void nbrListPrintDebug(NbrList* nbrListPtr, char* name) {
	uint8 i;

	cprintf("%s={", name);
	for (i = 0; i < nbrListPtr->size; i++) {
		if((nbrListPtr->size > 1) && (i > 0)) {
			cprintf(",");
		}
		cprintf("%02d, 05%d", nbrListPtr->nbrs[i]->ID, nbrListPtr->nbrs[i]->updateTime);
	}
	cprintf("}");
}


/*
 * @brief Returns true if this neighbor is a beacon.
 *
 * @param nbrPtr neighbor pointer
 * @returns true if the neighbor is a IR beacon
 */
boolean nbrIsBeacon(Nbr* nbrPtr) {
	if((nbrGetID(nbrPtr) >= 125) && (nbrGetID(nbrPtr) <= 127)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*
 * @brief Returns true if this neighbor is a beacon.
 *
 * @param nbrPtr neighbor pointer
 * @returns true if the neighbor is a IR beacon
 */
boolean nbrIsRobot(Nbr* nbrPtr) {
	if((nbrGetID(nbrPtr) >= ROBOT_ID_MIN) && (nbrGetID(nbrPtr) <= ROBOT_ID_MAX)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*
 * @brief Get neighbor ID.
 *
 * @param nbrPtr neighbor pointer
 * @returns ID
 */
uint8 nbrGetID(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->ID;
	} else {
		return ROBOT_ID_NULL;
	}
}

/*
 * @brief Get neighbor bearing.
 *
 * @param nbrPtr neighbor pointer
 * @returns returns bearing.  ranges within [-pi, pi)
 */
int32 nbrGetBearing(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->bearing;
	} else {
		return 0;
	}
}

/*
 * @brief Get neighbor orientation.
 *
 * @param nbrPtr neighbor pointer
 * @returns orientation.  ranges within [-pi, pi)
 */
int32 nbrGetOrientation(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->orientation;
	} else {
		return 0;
	}
}

/*
 * @brief Get neighbor range.
 *
 * @param nbrPtr neighbor pointer
 * @returns orientation.  ranges within [-pi, pi)
 */
int16 nbrGetRange(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->range;
	} else {
		return 0;
	}
}

/*
 * @brief Get neighbor orientation valid.
 *
 * @param nbrPtr neighbor pointer
 * @returns whether orientation is valid
 */
boolean nbrGetOrientationValid(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->orientationValid;
	} else {
		return 0;
	}
}

/*
 * @brief Get neighbor range bits.
 *
 * Range bits are recieverBitCount + orientationBitCount
 * @param nbrPtr neighbor pointer
 * @returns range bits
 */
uint32 nbrGetRangeBits(Nbr* nbrPtr) {	//### 32-bit
	if (nbrPtr) {
		return nbrPtr->rangeBits;
	} else {
		return 0;
	}
}


/*
 * @brief Get neighbor range bits.
 *
 * Range bits are recieverBitCount + orientationBitCount
 * @param nbrPtr neighbor pointer
 * @returns range bits
 */
uint8 nbrGetOrientationBitsCount(Nbr* nbrPtr) {
	uint8 i;
	if (nbrPtr) {
		uint8 rangeBitCount = 0;
		for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
			rangeBitCount += bitsCount(nbrPtr->orientationsBitsMatrix[i]);
		}
		//return nbrPtr->receiverBitCount + nbrPtr->orientationBitCount;
		return rangeBitCount;
	} else {
		return 0;
	}
}


/*
 * @brief Get neighbor receiver bits.
 *
 * Receiver bits are the actual receivers the message was received on
 * @param nbrPtr neighbor pointer
 * @returns receiver bits
 */
uint8 nbrGetReceiverBits(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->receiverBits;
	} else {
		return 0;
	}
}


/*
 * @brief Get neighbor transmitter bits.
 *
 * Receiver bits are the actual transmitter the message was received from
 * @param nbrPtr neighbor pointer
 * @returns transmitter bits
 */
uint8 nbrGetTransmitterBits(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->orientationBits;
	} else {
		return 0;
	}
}


/*
 * @brief Get neighbor update time.
 *
 * @param nbrPtr neighbor pointer
 * @returns update time
 */
uint32 nbrGetUpdateTime(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->updateTime;
	} else {
		return 0;
	}
}


/*
 * @brief Get neighbor update round
 *
 * @param nbrPtr neighbor pointer
 * @returns the last round this neighbor was heard from
 */
uint32 nbrGetUpdateRound(Nbr* nbrPtr) {
	if (nbrPtr) {
		return nbrPtr->updateRound;
	} else {
		return 0;
	}
}



