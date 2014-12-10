/*
 * consensusAvg.c
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#include "../consensus/consensus.h"
#include "../util/centroidData.h"
#include <stdio.h>

#define INVVALUE -1.0

centroidData tempValue;
centroidNbrData value;

centroidData inputValue;

/**
 * What to do on a consensus disable. Just resets the pipeline to initial
 * state.
 */
void consensusCentroidDisableOperation(void) {
	centroidNbrDataCopy(&inputValue, &value);
}

/**
 * Stores the value of the neighbors data into a temporary storage location
 *
 * @param nbrPtr
 * 		The neighbor whose data we need to store
 */
void consensusCentroidStoreTempData(Nbr *nbrPtr) {
	/* Store temporary value */
	centroidValue x, y;
	centroidNbrDataGetNbr(&x, &y, &value, nbrPtr);

	centroidTransform(&x, &y, nbrPtr);

	centroidDataSet(x, y, &tempValue);
}

/**
 * The operation called upon successful gossip. Averages our value and the
 * value stored in the temporary storage area together and sets our value.
 */
void consensusCentroidOperation(void) {
	/* Get our value */
	centroidValue ox, oy, tx, ty;
	centroidNbrDataGet(&ox, &oy, &value);
	centroidDataGet(&tx, &ty, &tempValue);

	if (tx == INVVALUE || ty == INVVALUE) {
		return;
	}

	centroidValue nx = (ox + tx) / ((centroidValue) 2.0);
	centroidValue ny = (oy + ty) / ((centroidValue) 2.0);

	centroidNbrDataSet(nx, ny, &value);

	centroidDataSet(INVVALUE, INVVALUE, &tempValue);
}

/**
 * Prints out our input value and our current value, called at the start of
 * each round.
 */
void consensusCentroidPrint(uint8 state) {
	char tempBuffer[30];

	centroidValue x, y;
	centroidNbrDataGet(&x, &y, &value);

	sprintf(tempBuffer, "%.3f, %.3f\n", x, y);
	rprintf(tempBuffer);
	rprintfFlush();

	cprintf("pt 3,%d,%d\n", (int16) x, (int16) y);
}

/**
 * Initializes average consensus
 */
void consensusCentroidInit(void) {
	/* Assign our input value */
	centroidDataSet(0, 0, &inputValue);

	/* Create our neighbor data */
	centroidNbrDataCreate(&value);
	centroidNbrDataCopy(&inputValue, &value);

	centroidDataSet(INVVALUE, INVVALUE, &tempValue);

	/* Set the round operation to print out our values */
	consensusSetRoundOperation(consensusCentroidPrint);
	consensusSetDisableOperation(consensusCentroidDisableOperation);

	/* Initialize the consensus subsystem with our functions */
	consensusInit(consensusCentroidStoreTempData, consensusCentroidOperation);
}
