/*
 * consensus.c
 *
 * Finite state machine that performs pairwise gossip consensus.
 *
 *  Created on: Oct 25, 2014
 *      Author: zkk
 */

#include "consensus.h"

uint8 consensusState;			// Current consensus state

NbrData consensusReqID;			// Requested ID for consensus
NbrData consensusAckID;			// Acknowledgment ID for consensus
NbrData consensusNonce;			// Request number (nonce)

/* Previous values from acknowledged robots, so we don't ack them again */
uint8 consensusPrevNonce = CONSENSUS_MAX_NONCE;
uint8 consensusPrevReqID = 0;

uint32 consensusWakeTime;		// Task wake time
uint32 consensusStateTime;		// Time when state began

NbrList consensusNbrList;		// Neighbor list

uint8 consensusFeedback = 1;	// Display information about state?

/**
 * Function to call upon successful gossip. Should operate on the temporary data
 * stored from consensusTempStoreData.
 */
void (*consensusOperation)();

/**
 * Function to temporarily store the neighbors data before consensus, so that
 * there isn't a race condition with new data being read by the neighbor.
 * Argument is the neighbor to store data from
 */
void (*consensusStoreTempData)(uint8 nbrID);

/**
 * Sets the requested ID to a random neighbor from the neighbor list
 */
void consensusSetReqID(void) {
	uint8 randomNbr = rand() % consensusNbrList.size;
	uint8 reqID = nbrListGetNbr(&consensusNbrList, randomNbr)->ID;

	nbrDataSet(&consensusReqID, reqID);
}

/**
 * Sets the acknowledged ID
 */
void consensusSetAckID(uint8 ackID) {
	nbrDataSet(&consensusAckID, ackID);
}

/**
 * Nulls out req and ack ID's for idle state
 */
void consensusSetIDsNull(void) {
	nbrDataSet(&consensusReqID, 0);
	nbrDataSet(&consensusAckID, 0);
}

/**
 * Increments the nonce with wrap-around
 */
void consensusIncNonce(void) {
	uint8 newNonce = nbrDataGet(&consensusNonce);
	newNonce = (newNonce + 1) % CONSENSUS_MAX_NONCE;
	nbrDataSet(&consensusNonce, newNonce);
}

/**
 * Browses neighbor list to find a random neighbor that has requested to gossip
 * with us. Returns their ID so that we can ack them and perform consensus.
 */
uint8 consensusGetAckID(void) {
	uint8 ackID = 0;	// Return ID

	uint8 reqInd = 0;						// Index in request list
	uint8 reqNbrs[consensusNbrList.size];	// Valid requesting nbrs
	uint8 reqNonce[consensusNbrList.size];	// Nonces of requesting nbrs

	/* Iterate over neighbor list */
	uint8 i;
	for (i = 0; i < consensusNbrList.size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(&consensusNbrList, i);

		/* Handle bad neighbors */
		if (!nbrPtr) {
			continue;
		}

		uint8 nbrID = nbrGetID(nbrPtr);

		/* Is the requested ID our ID? */
		if (nbrDataGetNbr(&consensusReqID, nbrPtr) == roneID) {
			uint8 nbrNonce = nbrDataGetNbr(&consensusNonce, nbrPtr);

			/* If this was the neighbor we gossiped with last time, make sure
			 * the request is fresh */
			if (nbrID == consensusPrevReqID) {
				if (nbrNonce == consensusPrevNonce) {
					/* The request isn't fresh! Nonce matches, ignore */
					continue;
				}
			}
			reqNonce[reqInd] = nbrNonce;
			reqNbrs[reqInd++] = nbrID;
		}
	}

	/* Return a random neighbor from this list of requesters */
	if (reqInd) {
		uint8 randInd = rand() % reqInd;
		ackID = reqNbrs[randInd];
		consensusPrevReqID = ackID;
		consensusPrevNonce = reqNonce[randInd];
	}

	return ackID;
}

/**
 * Switches state and sets state time
 */
void consensusSwitchState(uint8 newState) {
	/* Set state variable and beginning time */
	consensusState = newState;
	consensusStateTime = osTaskGetTickCount();

	/* Set LEDs if display mode on */
	if (consensusFeedback) {
		switch (newState) {
		case (CONSENSUS_STATE_IDLE): {
			ledsSetPattern(LED_GREEN, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			break;
		}
		case (CONSENSUS_STATE_REQ): {
			ledsSetPattern(LED_RED, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			break;
		}
		case (CONSENSUS_STATE_ACK): {
			ledsSetPattern(LED_BLUE, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			break;
		}
		}
	}
}

/**
 * Main task, manages state machine of consensus
 */
void consensusTask(void *args) {
	/* Flag for whether consensus has been performed in request mode */
	uint8 requestConsensusFlag;

	/* Initialize state to idle */
	consensusSwitchState(CONSENSUS_STATE_IDLE);

	for (;;) {
		consensusWakeTime = osTaskGetTickCount();

		/* Create neighbor list for this round */
		neighborsGetMutex();
		nbrListCreate(&consensusNbrList);
		neighborsPutMutex();

		switch (consensusState) {
		case (CONSENSUS_STATE_IDLE): {
			/* Handle being in the idle state */
			if (consensusStateTime + CONSENSUS_TIME_IDLE > consensusWakeTime) {
				/* Still in idle state */
				uint8 ackID = consensusGetAckID();

				if (ackID) {
					/* If we have a robot requesting us to gossip, switch to ack
					 * mode */
					consensusSwitchState(CONSENSUS_STATE_ACK);

					/* Set our ack ID to the requesting robot */
					consensusSetAckID(ackID);

					/* Store data for consensus */
					consensusStoreTempData(ackID);
				}
			} else {
				/* Idle state over, change state by rolling the dice, change
				 * either to request or idle */
				if ((rand() % CONSENSUS_RAND_MOD) < CONSENSUS_REQ_PROB) {
					/* Staying in idle mode! */
					consensusSwitchState(CONSENSUS_STATE_IDLE);

				} else {
					/* Going to request mode! */
					consensusSwitchState(CONSENSUS_STATE_REQ);

					/* Increment nonce */
					consensusIncNonce();

					/* Select a random neighbor to gossip with */
					consensusSetReqID();

					/* Set request consensus flag to not done yet */
					requestConsensusFlag = 0;
				}
			}

			break;
		} // End idle state
		case (CONSENSUS_STATE_REQ): {
			/* Handle being in the request state */
			if (consensusStateTime + CONSENSUS_TIME_REQ > consensusWakeTime) {
				/* Still in request state */
				if (requestConsensusFlag) {
					/* We have already performed consensus, so just wait */
					break;
				}

				/* Check whether requested robot has acked back */
				uint8 reqID = nbrDataGet(consensusReqID);
				Nbr *nbrPtr = nbrsGetWithID(reqID);
				if (!nbrPtr) {
					/* Neighbor not found, break and continue to next round */
					break;
				}

				/* The requested robot has set us as acknowledged! */
				if (nbrDataGetNbr(&consensusAckID, nbrPtr) == roneID) {
					/* Store data for consensus, trip flag */
					consensusStoreTempData(reqID);
					requestConsensusFlag = 1;
				}
			} else {
				/* If we managed to get an ack back, perform consensus on the
				 * stored data */
				if (requestConsensusFlag) {
					consensusOperation();
				}

				/* Request state over, head back to idle state */
				consensusSwitchState(CONSENSUS_STATE_IDLE);

				/* Set ID values to null */
				consensusSetIDsNull();
			}

			break;
		} // End request state
		case (CONSENSUS_STATE_ACK): {
			/* Handle being in the ack state */
			if (consensusStateTime + CONSENSUS_TIME_ACK > consensusWakeTime) {
				/* Still in ack state, just wait it out */
			} else {
				/* Perform consensus now that the ack period is over */
				consensusOperation();

				/* Ack state over, head back to idle state */
				consensusSwitchState(CONSENSUS_STATE_IDLE);

				/* Set ID values to null */
				consensusSetIDsNull();
			}

			break;
		} // End ack state
		}

		/* Delay task until next round */
		osTaskDelayUntil(&consensusWakeTime, CONSENSUS_TASK_DELAY);
	}
}

/**
 * Initialize data and task for consensus.
 * Arguments are the two function needed to be provided by the user.
 * First is a function to temporarily store the values of the neighbor data as
 * to prevent race conditions from occurring.
 * Second is the function that performs consensus using your data and the
 * temporarily stored data from the first function call.
 */
void consensusInit(void (*storeTempData)(uint8 nbrID), void (*operation)(void)) {
	/* Set the function to store temporary data to prevent race conditions */
	consensusStoreTempData = storeTempData;

	/* Set the consensus operation to the provided function */
	consensusOperation = operation;

	/* Initialize the neighbor data used for the state machine */
	nbrDataCreate(&consensusReqID, "cReqId", 8, 0);
	nbrDataCreate(&consensusAckID, "cAckId", 8, 0);
	nbrDataCreate(&consensusNonce, "cNonce", 8, 0);

	/* Create the background task */
	osTaskCreate(consensusTask, "consenus", 1536, NULL,
			CONSENSUS_TASK_PRIORITY);
}

