/*
 * consensus.h
 *
 *  Created on: Oct 11, 2014
 *      Author: Zak
 */

#ifndef SRC_CONSENSUS_H_
#define SRC_CONSENSUS_H_

#include "roneos.h"
#include "ronelib.h"
#include <stdlib.h>

/**
 *  Task Information
 */
#define CONSENSUS_TASK_DELAY		100
#define CONSENSUS_TASK_PRIORITY		(tskIDLE_PRIORITY + 3)

/**
 *  States
 */
#define CONSENSUS_STATE_IDLE		0
#define CONSENSUS_STATE_REQ			1
#define CONSENSUS_STATE_ACK			2

/**
 *  Timing, based on a number of neighbor rounds
 */
#define CONSENSUS_TIME_IDLE			3
#define CONSENSUS_TIME_REQ			4
#define CONSENSUS_TIME_ACK			4

/**
 *  Other Information
 */
/* The maximum value of the request nonce. */
#define CONSENSUS_MAX_NONCE			100

/* The probability of going into request mode from idle mode.
 * Range is from 0 - CONSENSUS_RAND_MOD. */
#define CONSENSUS_REQ_PROB			5000

/* The modulus operator applied to random numbers generated for probability */
#define CONSENSUS_RAND_MOD			10000

/* Musak Feedbak */
#define CONSENSUS_INSTRUMENT		83
#define CONSENSUS_VELOCITY			30

/* Function Declarations */
void consensusDisable(void);

void consensusEnable(void);

uint8 consensusIsEnabled(void);

void consensusSetReqProbability(uint16 prob);

void consensusEnableFeedback(uint8 isOn);

void consensusSetRoundOperation(void (*roundOperation)(uint8 state));

uint8 consensusGetPartner(void);

void consensusSetDisableOperation(void (*disableOperation)(void));

uint8 consensusNewStateCheck(uint32 *oldCountPtr);

void consensusInit(void (*storeTempData)(Nbr *nbrPtr), void (*operation)(void));

#endif /* SRC_CONSENSUS_H_ */
