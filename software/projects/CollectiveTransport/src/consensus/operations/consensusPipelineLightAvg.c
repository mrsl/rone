/*
 * consensusPipelineAvg.c
 *
 *  Created on: Oct 28, 2014
 *      Author: Zak
 */

#include "../consensusPipeline.h"
#include <stdio.h>

#define CONSENSUS_PIPELINE_LIGHT_AVG_INV	-1.0

#define CONSENSUS_PIPELINE_LIGHT_AVG_SIZE	20			// Size of the pipeline

float tempValue[CONSENSUS_PIPELINE_LIGHT_AVG_SIZE];		// Temporary data array
NbrDataFloat value[CONSENSUS_PIPELINE_LIGHT_AVG_SIZE];	// The pipeline itself

float inputValue;								// Input value for the pipeline


void pipelineLightAverageSetInputValue(void) {
	int32 maxLightValue = 0;

	uint8 i;
	for (i = 0; i < 4; i++) {
		int32 tempLightValue;
		if ((tempLightValue = lightSensorGetValue(i)) > maxLightValue) {
			maxLightValue = tempLightValue;
		}
	}

	inputValue = (float) maxLightValue;
}

/**
 * Prints out the contents of the pipeline from head to tail.
 */
void pipelineLightAveragePrintPipeline(void) {
	char tempBuffer[30];

	/* Print out the input value and the current value we have */
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	sprintf(tempBuffer, "%.7f,%.7f\n", inputValue, nbrDataGetFloat(&value[oldIndex]));
	rprintf(tempBuffer);
	rprintfFlush();
}

/**
 * The input function for average consensus. Continually provides the same
 * input value. Stores data into requested index.
 *
 * @param index
 * 		The position in the array to store the input value into.
 */
void pipelineLightAverageInput(uint8 index) {
	pipelineLightAverageSetInputValue();
	nbrDataSetFloat(&value[index], inputValue);
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
void pipelineLightAverageStoreTempData(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex) {
	tempValue[destIndex] = nbrDataGetNbrFloat(&value[srcIndex], nbrPtr);
}

/**
 * The consensus operation that averages two numbers together. Takes data from
 * the temporary storage array and our pipeline and sets the value in our
 * pipeline
 *
 * @param index
 * 		The index from the pipeline arrays to use.
 */
void pipelineLightAverageOperation(uint8 index) {
	/* Get our current value */
	float currentValue = nbrDataGetFloat(&value[index]);

	/* Get the stored value */
	float theirValue = tempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_LIGHT_AVG_INV) {
		return;
	}

	/* Average our value and the temporary value together */
	float newValue = (currentValue + theirValue) / 2.0;

	/* Set our new value */
	nbrDataSetFloat(&value[index], newValue);

	/* Put invalid value back in place */
	tempValue[index] = CONSENSUS_PIPELINE_LIGHT_AVG_INV;
}

void pipelineLightAverageInit(void) {
	/* Initialize our data */
	uint8 i;
	for (i = 0; i < CONSENSUS_PIPELINE_LIGHT_AVG_SIZE; i++) {
		/* Create neighbor data */
		nbrDataCreateFloat(&value[i], "cpLAvg");
		/* Initialize temp storage to nulls */
		tempValue[i] = CONSENSUS_PIPELINE_LIGHT_AVG_INV;
	}

	consensusPipelineSetPrintFunction(pipelineLightAveragePrintPipeline);

	/* Initialize the pipeline */
	consensusPipelineInit(CONSENSUS_PIPELINE_LIGHT_AVG_SIZE,
			pipelineLightAverageInput,
			pipelineLightAverageStoreTempData,
			pipelineLightAverageOperation);
}
