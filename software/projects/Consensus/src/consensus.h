/*
 * consensus.h
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#ifndef SRC_CONSENSUS_H_
#define SRC_CONSENSUS_H_

#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

/**
 *  Task Information
 */
#define CONSENSUS_TASK_DELAY		100
#define CONSENSUS_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)

/**
 *  States
 */
#define CONSENSUS_STATE_IDLE		0
#define CONSENSUS_STATE_REQ			1
#define CONSENSUS_STATE_ACK			2

/**
 *  Timing, based on a number of neighbor rounds
 */
#define CONSENSUS_TIME_IDLE			(3 * neighborsGetPeriod())
#define CONSENSUS_TIME_REQ			(3 * neighborsGetPeriod())
#define CONSENSUS_TIME_ACK			(4 * neighborsGetPeriod())

/**
 *  Other Information
 */
/* The maximum value of the request nonce. */
#define CONSENSUS_MAX_NONCE			100
/* The probability of going into request mode from idle mode.
 * Range is from 0 - CONSENSUS_RAND_MOD */
#define CONSENSUS_REQ_PROB			500
/* The modulus operator applied to random numbers generated for probability */
#define CONSENSUS_RAND_MOD			1000

/* Function Declarations */
void consensusEnableFeedback(uint8 isOn);

void consensusSetRoundOperation(void (*roundOperation)(void));

void consensusInit(void (*storeTempData)(Nbr *nbrPtr), void (*operation)(void));

#endif /* SRC_CONSENSUS_H_ */
