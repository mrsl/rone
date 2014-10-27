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

/* Task Information */
#define CONSENSUS_TASK_DELAY		100
#define CONSENSUS_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)

/* States */
#define CONSENSUS_STATE_IDLE		0
#define CONSENSUS_STATE_REQ			1
#define CONSENSUS_STATE_ACK			2

/* Timing */
#define CONSENSUS_TIME_IDLE			1000
#define CONSENSUS_TIME_REQ			1000
#define CONSENSUS_TIME_ACK			1000

/* Other Information */
#define CONSENSUS_MAX_NONCE			100
#define CONSENSUS_REQ_PROB			500
#define CONSENSUS_RAND_MOD			1000

/* Function Declarations */
void consensusInit(void (*storeTempData)(uint8 nbrID), void (*operation)(void));

#endif /* SRC_CONSENSUS_H_ */
