/*
 * consensusAvg.c
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#include "consensus.h"

uint8 tempValue;	// Temporary storage location
NbrData value;		// Our value

uint8 inputValue;	// The input value for our consensus

/* Function Declaration */
void consensusAverageInit(void);

/**
 * Stores the value of the neighbors data into a temporary storage location
 *
 * @param nbrPtr
 * 		The neighbor whose data we need to store
 */
void consensusAverageStoreTempData(Nbr *nbrPtr) {
	/* Store temporary value */
	tempValue = nbrDataGetNbr(&value, nbrPtr);
}

/**
 * The operation called upon successful gossip. Averages our value and the
 * value stored in the temporary storage area together and sets our value.
 */
void consensusAverageOperation(void) {
	/* Get our value */
	uint8 currentValue = nbrDataGet(&value);

	/* Average our value and temporary value together */
	uint8 newValue = (currentValue + tempValue) / 2;

	/* Set our data */
	nbrDataSet(&value, newValue);
}

/**
 * Prints out our input value and our current value, called at the start of
 * each round.
 */
void consensusAveragePrint(uint8 state) {
	cprintf("%d, %d\n", inputValue, nbrDataGet(&value));
}

/**
 * Initializes average consensus
 */
void consensusAverageInit(void) {
	/* Assign our input value */
	inputValue = rand() % 200;

	/* Create our neighbor data */
	nbrDataCreate(&value, "cAvg", 8, inputValue);

	/* Set the round operation to print out our values */
	consensusSetRoundOperation(consensusAveragePrint);

	/* Initalize the consensus subsystem with our functions */
	consensusInit(consensusAverageStoreTempData, consensusAverageOperation);
}
