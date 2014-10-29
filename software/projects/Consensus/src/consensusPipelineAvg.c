/*
 * consensusPipelineAvg.c
 *
 *  Created on: Oct 28, 2014
 *      Author: zkk
 */

#include "consensusPipeline.h"

#define CONSENSUS_PIPELINE_AVG_SIZE	20			// Size of the pipeline

uint8 tempValue[CONSENSUS_PIPELINE_AVG_SIZE];	// Temporary data array
NbrData value[CONSENSUS_PIPELINE_AVG_SIZE];		// The pipeline itself

uint8 inputValue;								// Input value for the pipeline

/**
 * Function that prints out the number at the index given. Also prints a
 * trailing '-' to seperate values.
 *
 * @param index
 * 		The index to print
 */
void pipelineAveragePrintCell(uint8 index) {
	cprintf("%d,", nbrDataGet(&value[index]));
}

/**
 * Prints out the contents of the pipeline from head to tail.
 */
void pipelineAveragePrintPipeline(void) {
	/* Call the consensus pipeline print function using our print function */
	consensusPipelinePrintPipeline(pipelineAveragePrintCell);
	cprintf("\n");

	/* Print out the input value and the current value we have */
	uint8 oldIndex = consensusPipelineGetOldestIndex();
	cprintf("%d, %d\n", inputValue, nbrDataGet(&value[oldIndex]));
}

/**
 * The input function for average consensus. Continually provides the same
 * input value. Stores data into requested index.
 *
 * @param index
 * 		The position in the array to store the input value into.
 */
void pipelineAverageInput(uint8 index) {
	/* Set the input value */
	nbrDataSet(&value[index], inputValue);

	/* Print the new pipeline */
	pipelineAveragePrintPipeline();
}

/**
 * Stores temporary data from a neighbor's pipeline into our temporary data
 * storage.
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
void pipelineAverageStoreTempData(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex) {
	tempValue[destIndex] = nbrDataGetNbr(&value[srcIndex], nbrPtr);
}

/**
 * The consensus operation that averages two numbers together. Takes data from
 * the temporary storage array and our pipeline and sets the value in our
 * pipeline
 *
 * @param index
 * 		The index from the pipeline arrays to use.
 */
void pipelineAverageOperation(uint8 index) {
	/* Get our current value */
	uint8 currentValue = nbrDataGet(&value[index]);

	cprintf("%d - %d\n", currentValue, tempValue[index]);

	/* Average our value and the temporary value together */
	uint8 newValue = (currentValue + tempValue[index]) / 2;

	/* Set our new value */
	nbrDataSet(&value[index], newValue);
}

void pipelineAverageInit(void) {
	/* Random input value */
	inputValue = (roneID == 105) ? 0 : 100;

	/* Initialize all our neighbor data */
	uint8 i;
	for (i = 0; i < CONSENSUS_PIPELINE_AVG_SIZE; i++) {
		nbrDataCreate(&value[i], "cpAvg", 8, 0);
	}

	/* Initialize the pipeline */
	consensusPipelineInit(CONSENSUS_PIPELINE_AVG_SIZE,
			pipelineAverageInput,
			pipelineAverageStoreTempData,
			pipelineAverageOperation);
}
