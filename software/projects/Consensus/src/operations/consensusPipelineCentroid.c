/*
 * consensusPipelineCentroid.c
 *
 *  Created on: Nov 8, 2014
 *      Author: Zak
 */

#include "../consensus/consensusPipeline.h"
#include "../util/centroidData.h"
#include <stdio.h>

#define CONSENSUS_PIPELINE_CENTROID_INV		-1.0	// Invalid value
#define CONSENSUS_PIPELINE_CENTROID_SIZE	20		// Size of the pipeline

centroidData inputValue;
centroidData tempValue[CONSENSUS_PIPELINE_CENTROID_SIZE];	// Temporary data array
centroidNbrData value[CONSENSUS_PIPELINE_CENTROID_SIZE];	// The pipeline itself

/**
 * Prints out the contents of the pipeline from head to tail.
 */
void pipelineCentroidPrintPipeline(void) {
	char tempBuffer[30];

	/* Print out the input value and the current value we have */
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	centroidValue x, y;
	centroidNbrDataGet(&x, &y, &value[oldIndex]);

	sprintf(tempBuffer, "%.3f, %.3f\n", x, y);
	rprintf(tempBuffer);
	rprintfFlush();

	cprintf("pt 3,%d,%d\n", (int16) x, (int16) y);
}

/**
 * The input function for average consensus. Continually provides the same
 * input value. Stores data into requested index.
 *
 * @param index
 * 		The position in the array to store the input value into.
 */
void pipelineCentroidInput(uint8 index) {
	centroidNbrDataCopy(&inputValue, &value[index]);
}

/**
 * Stores temporary data from a neighbor's pipeline into our temporary data
 * storage. Also transforms data into our local coordinate frame.
 *
 * @param nbrPtr
 * 		The neighbor to look up data from
 *
 * @param srcIndex
 * 		The index in the neighbors pipeline to save data from
 *
 * @param destIndex
 * 		The destination index in the temporary data storage buffer to save the
 * 		data from.
 */
void pipelineCentroidStoreTempData(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex) {
	/* Store temporary value */
	centroidValue x, y;
	centroidNbrDataGetNbr(&x, &y, &value[srcIndex], nbrPtr);

	centroidTransform(&x, &y, nbrPtr);

	centroidDataSet(x, y, &tempValue[destIndex]);
}



/**
 * Averages two coordinate in our local coordinate frame together.
 *
 * @param index
 * 		The index from the pipeline arrays to use.
 */
void pipelineCentroidOperation(uint8 index) {
	/* Get our value */
	centroidValue ox, oy, tx, ty;
	centroidNbrDataGet(&ox, &oy, &value[index]);
	centroidDataGet(&tx, &ty, &tempValue[index]);

	if (tx == CONSENSUS_PIPELINE_CENTROID_INV
		|| ty == CONSENSUS_PIPELINE_CENTROID_INV) {
		return;
	}

	centroidValue nx = (ox + tx) / ((centroidValue) 2.0);
	centroidValue ny = (oy + ty) / ((centroidValue) 2.0);

	centroidNbrDataSet(nx, ny, &value[index]);

	centroidDataSet(CONSENSUS_PIPELINE_CENTROID_INV,
					CONSENSUS_PIPELINE_CENTROID_INV,
					&tempValue[index]);
}

void pipelineCentroidInit(void) {
	/* Assign our input value */
	centroidDataSet(0, 0, &inputValue);

	/* Initialize our data */
	uint8 i;
	for (i = 0; i < CONSENSUS_PIPELINE_CENTROID_SIZE; i++) {
		centroidNbrDataCreate(&value[i]);
		centroidNbrDataCopy(&inputValue, &value[i]);

		centroidDataSet(CONSENSUS_PIPELINE_CENTROID_INV,
						CONSENSUS_PIPELINE_CENTROID_INV,
						&tempValue[i]);
	}

	consensusPipelineSetPrintFunction(pipelineCentroidPrintPipeline);

	/* Initialize the pipeline */
	consensusPipelineInit(CONSENSUS_PIPELINE_CENTROID_SIZE,
			pipelineCentroidInput,
			pipelineCentroidStoreTempData,
			pipelineCentroidOperation);
}
