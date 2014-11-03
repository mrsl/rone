/*
 * consensus.c
 *
 * Finite state machine that performs pairwise gossip consensus. Uses user
 * provided functions to abstract the background consensus task.
 *
 *  Created on: Oct 25, 2014
 *      Author: zkk
 */

#include "consensus.h"

/* Variables */
uint8 consensusState;			// Current consensus state
uint8 consensusStateTime;		// Round counter to track time in state

NbrData consensusReqID;			// Requested ID for consensus
NbrData consensusAckID;			// Acknowledgment ID for consensus
NbrData consensusNonce;			// Request number (nonce)

/* Previous values from acknowledged robots, so we don't ack them again */
uint8 consensusPrevNonce = CONSENSUS_MAX_NONCE;
uint8 consensusPrevReqID = 0;

uint32 consensusWakeTime;		// Task wake time
uint32 consensusStateCount = 0;	// Number of times we have changed state

NbrList consensusNbrList;		// Neighbor list, updated each consensus round

uint8 consensusFeedback = 0;	// Display information about state?

/**
 * Function to call each round. May manipulate something in the data, etc.
 * Parameter is the current consensus state.
 */
void (*consensusRoundOperation)(uint8 state) = NULL;

/**
 * Function to call upon successful gossip. Should operate on the temporary data
 * stored from consensusTempStoreData.
 */
void (*consensusOperation)(void) = NULL;

/**
 * Function to temporarily store the neighbors data before consensus, so that
 * there isn't a race condition with new data being read by the neighbor.
 * Argument is the neighbor to store data from
 */
void (*consensusStoreTempData)(Nbr *nbrPtr) = NULL;

/**
 * Set feedback based on boolean input parameter, if on gives visual feedback
 *
 * @param isOn
 * 		Turns visual LED feedback on or off based on boolean value.
 */
void consensusEnableFeedback(uint8 isOn) {
	consensusFeedback = (isOn) ? 1 : 0;
}

/**
 * Sets the round operation for consensus.
 *
 * Round operation is called at the start of each consensus round regardless of
 * state. Only triggers if a new neighbor update round has occurred.
 *
 * @param void (*roundOperation)(uint8 state)
 * 		A function that updates the consensus data each round, or does some
 * 		other task synchronized with consensus. The parameter given is the
 * 		current consensus state.
 */
void consensusSetRoundOperation(void (*roundOperation)(uint8 state)) {
	consensusRoundOperation = roundOperation;
}

/**
 * Sets the requested ID to a random neighbor from the neighbor list.
 */
void consensusSetReqID(void) {
	uint8 randomNbr = rand() % consensusNbrList.size;
	uint8 reqID = nbrListGetNbr(&consensusNbrList, randomNbr)->ID;

	nbrDataSet(&consensusReqID, reqID);
}

/**
 * Sets the acknowledged ID.
 */
void consensusSetAckID(uint8 ackID) {
	nbrDataSet(&consensusAckID, ackID);
}

/**
 * Nulls out req and ack ID's for idle state.
 */
void consensusSetIDsNull(void) {
	nbrDataSet(&consensusReqID, 0);
	nbrDataSet(&consensusAckID, 0);
}

/**
 * Increments the nonce with wrap-around.
 */
void consensusIncNonce(void) {
	uint8 newNonce = nbrDataGet(&consensusNonce);
	newNonce = (newNonce + 1) % CONSENSUS_MAX_NONCE;
	nbrDataSet(&consensusNonce, newNonce);
}

/**
 * Browses neighbor list to find a random neighbor that has requested to gossip
 * with us. Returns their ID so that we can ack them and perform consensus.
 *
 * Shouldn't be called except from the main task.
 *
 * @return A pointer to a neighbor that is currently requesting to gossip with
 * us.
 */
Nbr *consensusGetReqNbr(void) {
	Nbr *reqNbr = NULL;	// Requesting nbr

	uint8 reqInd = 0;						// Index in request list
	Nbr *reqNbrs[consensusNbrList.size];	// Valid requesting nbrs
	uint8 reqNonce[consensusNbrList.size];	// Nonces of requesting nbrs

	/* Iterate over neighbor list */
	uint8 i;
	for (i = 0; i < consensusNbrList.size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(&consensusNbrList, i);

		/* Handle bad neighbors by ignoring them */
		if (!nbrPtr) {
			continue;
		}

		/* Is the requested ID our ID? */
		if (nbrDataGetNbr(&consensusReqID, nbrPtr) == roneID) {
			uint8 nbrNonce = nbrDataGetNbr(&consensusNonce, nbrPtr);

			/* If this was the neighbor we gossiped with last time, make sure
			 * the request is fresh */
			if (nbrGetID(nbrPtr) == consensusPrevReqID) {
				if (nbrNonce == consensusPrevNonce) {
					/* The request isn't fresh! Nonce matches, ignore */
					continue;
				}
			}

			/* Store data */
			reqNonce[reqInd] = nbrNonce;
			reqNbrs[reqInd] = nbrPtr;

			reqInd++;
		}
	}

	/* Return a random neighbor from this list of requesters */
	if (reqInd) {
		uint8 randInd = rand() % reqInd;
		reqNbr = reqNbrs[randInd];

		consensusPrevReqID = nbrGetID(reqNbr);
		consensusPrevNonce = reqNonce[randInd];
	}

	return reqNbr;
}

/**
 * Check if a new round has occurred.
 *
 * @param oldCountPtr
 * 		Old value of the round count when last checked.
 *
 * @return Boolean whether or not a new round has occurred. Updates value.
 */
uint8 consensusNewStateCheck(uint32 *oldCountPtr) {
	uint8 val = 0;
	if (consensusStateCount != *oldCountPtr) {
		val = 1;
	}
	*oldCountPtr = consensusStateCount;
	return val;
}

/**
 * Switches state and sets state time.
 *
 * Shouldn't be called besides from the main consensus task.
 *
 * @param newState
 * 		The new state to enter. Can be from the set of CONSENSUS_STATE_IDLE,
 * 		CONSENSUS_STATE_REQ, and CONSENSUS_STATE_ACK.
 */
void consensusSwitchState(uint8 newState) {
	/* Set state variable and beginning time */
	consensusState = newState;
	consensusStateTime = 0;
	consensusStateCount++;

	/* Set LEDs if display mode is on */
	if (consensusFeedback) {
		switch (newState) {
		case (CONSENSUS_STATE_IDLE): {
			ledsSetPattern(LED_GREEN, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			audioNoteOn(CONSENSUS_INSTRUMENT, 69, CONSENSUS_VELOCITY, 500);
			break;
		}
		case (CONSENSUS_STATE_REQ): {
			ledsSetPattern(LED_RED, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			audioNoteOn(CONSENSUS_INSTRUMENT, 73, CONSENSUS_VELOCITY, 500);
			break;
		}
		case (CONSENSUS_STATE_ACK): {
			ledsSetPattern(LED_BLUE, LED_PATTERN_ON,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			audioNoteOn(CONSENSUS_INSTRUMENT, 76, CONSENSUS_VELOCITY, 500);
			break;
		}
		}
	}
}

/**
 * Perform consensus, maybe play a tune, print some lines, kick back and relax.
 */
void consensusDoConsensus(void) {
	consensusOperation();

	if (consensusFeedback) {
		audioNoteOn(CONSENSUS_INSTRUMENT, 69, CONSENSUS_VELOCITY, 500);
		audioNoteOn(CONSENSUS_INSTRUMENT, 73, CONSENSUS_VELOCITY, 500);
		audioNoteOn(CONSENSUS_INSTRUMENT, 76, CONSENSUS_VELOCITY, 500);
		audioNoteOn(CONSENSUS_INSTRUMENT, 81, CONSENSUS_VELOCITY, 500);
	}
}

/**
 * Perform the round operation
 */
void consensusDoRoundOperation(void) {
	/* Call the round operation before the round begins */
	consensusRoundOperation(consensusState);

	if (consensusFeedback) {
		audioNoteOffAll();
		audioNoteOn(CONSENSUS_INSTRUMENT, 57, CONSENSUS_VELOCITY, 500);
	}
}

/**
 * Main task, manages state machine of consensus. Calls external functions
 * when relevant. Only runs when there has been an update toh the neighbor
 * subsystem.
 *
 * @param args
 * 		Arguments to the task, not needed.
 */
void consensusTask(void *args) {
	/* Flag for whether consensus has been performed in request mode */
	uint8 requestConsensusFlag;
	uint32 neighborRound;

	/* Initialize state to idle */
	consensusSwitchState(CONSENSUS_STATE_IDLE);

	for (;;) {
		consensusWakeTime = osTaskGetTickCount();

		/* Only check if a new neighbor round has occurred */
		if (!neighborsNewRoundCheck(&neighborRound)) {
			osTaskDelayUntil(&consensusWakeTime, CONSENSUS_TASK_DELAY);
			continue;
		}

		/* Create neighbor list for this round */
		neighborsGetMutex();
		nbrListCreate(&consensusNbrList);

		/* Increment the state round counter */
		consensusStateTime++;

		consensusDoRoundOperation();

		switch (consensusState) {
		case (CONSENSUS_STATE_IDLE): {
			/* Handle being in the idle state */
			if (consensusStateTime <= CONSENSUS_TIME_IDLE) {
				/* Still in idle state */
				Nbr *reqNbr = consensusGetReqNbr();

				if (reqNbr) {
					/* If we have a robot requesting us to gossip, switch to
					 * ack mode */
					consensusSwitchState(CONSENSUS_STATE_ACK);

					/* Set our ack ID to the requesting robot */
					consensusSetAckID(nbrGetID(reqNbr));

					/* Store data for consensus */
					consensusStoreTempData(reqNbr);
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

					/* Increment request nonce */
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
			if (consensusStateTime <= CONSENSUS_TIME_REQ) {
				/* Still in request state */
				if (requestConsensusFlag) {
					/* We have already performed consensus, so just wait */
					break;
				}

				/* Check whether requested robot has acked back */
				uint8 reqID = nbrDataGet(&consensusReqID);
				Nbr *nbrPtr = nbrsGetWithID(reqID);
				if (!nbrPtr) {
					/* Neighbor not found, break and continue to next round */
					break;
				}

				/* The requested robot has set us as acknowledged! */
				if (nbrDataGetNbr(&consensusAckID, nbrPtr) == roneID) {
					/* Store data for consensus, trip flag */
					consensusStoreTempData(nbrPtr);
					requestConsensusFlag = 1;
				}

			} else {
				/* If we managed to get an ack back, perform consensus on the
				 * stored data */
				if (requestConsensusFlag) {
					consensusDoConsensus();
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
			if (consensusStateTime <= CONSENSUS_TIME_ACK) {
				/* Still in ack state, just wait it out */

			} else {
				/* Perform consensus now that the ack period is over */
				consensusDoConsensus();

				/* Ack state over, head back to idle state */
				consensusSwitchState(CONSENSUS_STATE_IDLE);

				/* Set ID values to null */
				consensusSetIDsNull();
			}
			break;
		} // End ack state
		default: {
			/* Something goofy happened, try to recover or something */
			consensusSwitchState(CONSENSUS_STATE_IDLE);

			/* Set ID values to null */
			consensusSetIDsNull();
			break;
		}
		}

		neighborsPutMutex();

		/* Delay task until next round */
		osTaskDelayUntil(&consensusWakeTime, CONSENSUS_TASK_DELAY);
	}
}

/**
 * No-op function used for uninitialized functions
 */
void consensusNoOp(uint8 state) {
}

/**
 * Initialize data and task for consensus. Creates a subsystem task that
 * attempts pairwise gossip based consensus with other robots running the same
 * subsystem.
 *
 * User should initialize the consensus data themselves, as well as a temporary
 * storage data solution. The User then provides functions that are then called
 * when needed by the task. These functions are the store temporary data
 * function, where a neighbor data from a robot is stored in a temporary buffer
 * to prevent race conditions, and a consensus operation, where the user does
 * whatever consensus operation they want with data stored in the neighbor data
 * and temporary storage.
 *
 * @param void (*storeTempData)(Nbr *nbrPtr)
 * 		Temporary data storage function. Stores data from the specified neighbor
 * 		into a temporary buffer. The data in the temporary buffer should be used
 * 		later in the consensus operation to achieve consensus. This is to
 * 		prevent race conditions with the two communicating robots updating their
 * 		data at the same time.
 *
 * @param void (*operation)(void)
 * 		The consensus operation. Should do whatever operation needed between
 * 		your neighbor data value and the temporary storage buffer. Should also
 * 		set the neighbor data to the new value.
 */
void consensusInit(void (*storeTempData)(Nbr *nbrPtr), void (*operation)(void)) {
	/* If a round operation hasn't been set, set it to the default no-op */
	if (consensusRoundOperation == NULL) {
		consensusRoundOperation = consensusNoOp;
	}

	/* Set the function to store temporary data to prevent race conditions */
	consensusStoreTempData = storeTempData;

	/* Set the consensus operation to the provided function */
	consensusOperation = operation;

	/* Initialize the neighbor data used for the state machine */
	nbrDataCreate(&consensusReqID, "cReqId", 8, 0);
	nbrDataCreate(&consensusAckID, "cAckId", 8, 0);
	nbrDataCreate(&consensusNonce, "cNonce", 8, 0);

	/* Create the background task */
	osTaskCreate(consensusTask, "consenus", 2048, NULL, CONSENSUS_TASK_PRIORITY);
}

