/*
 * consensus.c
 *
 *  Created on: Oct 25, 2014
 *      Author: zkk
 */

#include "consensus.h"

uint8 consensusState;			// Current consensus state

NbrData consensusReqID;			// Requested ID for consensus
NbrData consensusAckID;			// Acknowledgment ID for consensus
NbrData consensusNonce;			// Request number (nonce)
uint8 consensusReqIDValue = 0;	// Requsted ID value
uint8 consensusNonceValue = 0;	// Nonce value

uint8 consensusPrevNonce = CONSENSUS_MAX_NONCE;	// Previous value containers
uint8 consensusPrevReqID = 0;

uint32 consensusWakeTime;	// Task wake time
uint32 consensusStateTime;	// Time when state began

NbrList consensusNbrList;	// Neighbor list

uint8 consensusFeedback = 1;	// Display information about state?

/**
 * Function to call upon successful gossip, argument is the neighbor to perform
 * gossip with.
 */
void (*consensusOperation)(uint8 nbrID);

/**
 * Sets the requested ID to a random neighbor
 */
inline void consensusSetReqID(void) {
	uint8 randomNbr = rand() % consensusNbrList.size;
	uint8 reqID = nbrListGetNbr(&consensusNbrList, randomNbr)->ID;
	consensusReqIDValue = reqID;

	nbrDataSet(&consensusReqID, reqID);
}

/**
 * Sets the acknowledged ID
 */
inline void consensusSetAckID(uint8 ackID) {
	nbrDataSet(&consensusAckID, ackID);
}

/**
 * Nulls out req and ack ID's for idle state
 */
inline void consensusNullIDs(void) {
	nbrDataSet(&consensusReqID, 0);
	nbrDataSet(&consensusAckID, 0);
}

/**
 * Increments the nonce
 */
inline void consensusIncNonce(void) {
	consensusNonceValue++;
	consensusNonceValue %= CONSENSUS_MAX_NONCE;
	nbrDataSet(&consensusNonce, consensusNonceValue);
}

/**
 * Finds a random neighbor that has our ID set to their requested ID
 */
inline uint8 consensusGetAckID(void) {
	uint8 ackID = 0;	// Return value

	uint8 requestingInd = 0;						// Index in request list
	uint8 requestingNbrs[consensusNbrList.size];	// Valid requesting nbrs

	/* Iterate over neighbor list */
	uint8 i;
	for (i = 0; i < consensusNbrList.size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(&consensusNbrList, i);
		uint8 nbrID = nbrGetID(nbrPtr);

		if (nbrDataGetNbr(&consensusReqID, nbrPtr) == roneID) {
			/* Is the requested ID our ID? */

			if (nbrID == consensusPrevReqID) {
				/* If this was the neighbor we gossiped with last time, make
				 * sure the request is fresh */
				if (nbrDataGetNbr(&consensusNonce, nbrPtr) == consensusPrevNonce) {
					/* The request is not fresh! Nonce matches previous,
					 * ignore */
					continue;
				}
			}
			requestingNbrs[requestingInd++] = nbrID;
		}
	}

	/* Return a random neighbor from this list of requesters */
	if (requestingInd) {
		ackID = requestingNbrs[rand() % requestingInd];
	}

	return ackID;
}

/**
 * Switches state and sets state time
 */
void consensusSwitchState(uint8 newState) {
	consensusState = newState;
	consensusStateTime = osTaskGetTickCount();
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
			/* Set LEDs if display mode on */
			ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			/* Handle being in the idle state */
			if (consensusStateTime + CONSENSUS_TIME_IDLE < consensusWakeTime) {
				/* Still in idle state */
				uint8 ackID = consensusGetAckID();

				if (ackID) {
					/* If we have a robot requesting us to gossip, switch to ack
					 * mode */
					consensusSwitchState(CONSENSUS_STATE_ACK);

					/* Set our ack ID to the requesting robot */
					consensusSetAckID(ackID);

					/* Perform consensus */
					consensusOperation(ackID);
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
		}
		case (CONSENSUS_STATE_REQ): {
			/* Set LEDs if display mode on */
			ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			/* Handle being in the request state */
			if (consensusStateTime + CONSENSUS_TIME_REQ < consensusWakeTime) {
				/* Still in request state, check whether requested robot has
				 * acked back */
				Nbr *nbrPtr = nbrsGetWithID(consensusReqIDValue);
				if (nbrPtr) {
					if (nbrDataGetNbr(&consensusAckID, nbrPtr) == roneID) {
						/* The requsted robot has set us as acknowledged! */

						/* Perform consensus, trip flag */
						if (!requestConsensusFlag) {
							consensusOperation(consensusReqIDValue);
							requestConsensusFlag = 1;
						}
					}
				}
			} else {
				/* Request state over, head back to idle state */
				consensusSwitchState(CONSENSUS_STATE_IDLE);

				/* Set ID values to null */
				consensusNullIDs();
			}
			break;
		}
		case (CONSENSUS_STATE_ACK): {
			/* Set LEDs if display mode on */
			ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			/* Handle being in the ack state */
			if (consensusStateTime + CONSENSUS_TIME_ACK < consensusWakeTime) {
				/* Still in ack state */
			} else {
				/* Ack state over, head back to idle state */
				consensusSwitchState(CONSENSUS_STATE_IDLE);

				/* Set ID values to null */
				consensusNullIDs();
			}
			break;
		}
		}

		osTaskDelayUntil(&consensusWakeTime, CONSENSUS_TASK_DELAY);
	}
}

/**
 * Initialize data and task for consensus
 */
void consensusInit(void (*consensusOp)(uint8 nbrID)) {
	consensusOperation = consensusOp;

	nbrDataCreate(&consensusReqID, "cReqId", 8, 0);
	nbrDataCreate(&consensusAckID, "cAckId", 8, 0);
	nbrDataCreate(&consensusNonce, "cNonce", 8, 0);

	osTaskCreate(consensusTask, "consensus", 1536, NULL, CONSENSUS_TASK_PRIORITY);
}

