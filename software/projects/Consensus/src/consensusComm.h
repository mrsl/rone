/*
 * consensusComm.h
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#ifndef SRC_CONSENSUSCOMM_H_
#define SRC_CONSENSUSCOMM_H_

#include <stdlib.h>
#include "roneos.h"

#define CONSENSUS_DEFAULT_PERIOD		2000
#define CONSENSUS_RESPONSE_TIMEOUT		30
#define CONSENSUS_ACK_TIMEOUT_DELTA		10

#define CONSENSUS_INTER_MESSAGE_DELAY	10

#define CONSENSUS_DEFAULT_ACK_CHANCE	500

#define CONSENSUS_RAND_MOD				1000

#define CONSENSUS_TASK_PRIORITY			(tskIDLE_PRIORITY + 3)

#define CONSENSUS_MAX_MESSAGES 			8
#define CONSENSUS_MAX_DATA_SIZE			(RADIO_COMMAND_MESSAGE_DATA_LENGTH * CONSENSUS_MAX_MESSAGES)

#define CONSENSUS_ACK					0x1337

struct {
	uint32 ackCheck;
	uint8 id;
	uint8 padding[RADIO_COMMAND_MESSAGE_DATA_LENGTH - 5];
} typedef consensusAckMessage;

/**
 * Creates the radio messages for the consensus data
 */
void consensusInitData(void *data, uint8 size);

/**
 * Initializes an operation to be done each round
 *
 * Requires a pointer to a function that takes a pointer to the data
 */
void consensusInitPreConsensusOperation(void (*func)(void *currentData));


/**
 * Initializes an operation to be done each round
 *
 * Requires a pointer to a function that takes a pointer to the data
 */
void consensusInitPostConsensusOperation(void (*func)(void *currentData));

/**
 * Initializes the operation to be done on the consensus data
 *
 * Requires a pointer to a function that takes two pointers to the data to be
 * operated on. Should modify currentData in place.
 */
void consensusInitConsensusOperation(void (*func)(void *currentData, void *newData));

/**
 * Sets consensus period
 */
void consensusSetPeriod(uint32 newPeriod);

/**
 * Sets chance to go into ack mode or idle mode. Lower values are less chance for
 * ack mode.
 */
void consensusSetAckChance(uint16 newAckChance);

/**
 * Initializes consensus sequence
 */
void consensusInit(void *data, uint16 size);

#endif /* SRC_CONSENSUSCOMM_H_ */
