/*
 * consensusPipelineAvg.c
 *
 *  Created on: Oct 28, 2014
 *      Author: zkk
 */

#include "consensusPipeline.h"

#define CONSENSUS_PIPELINE_AVG_SIZE	5

uint8 tempValue[CONSENSUS_PIPELINE_AVG_SIZE];
NbrData value[CONSENSUS_PIPELINE_AVG_SIZE];

uint8 inputValue = rand() % 200;

void pipelineAveragePrintPipeline(void) {
	cprintf("PLINE: ")
	consensusPipelinePrintPipeline(pipelineAveragePrintCell);
	cprintf("\n");
}

void pipelineAverageInput(uint8 index) {
	nbrDataSet(&value[index], inputValue);
	pipelineAveragePrintPipeline();
}

void pipelineAverageStoreTempData(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex) {
	tempValue[destIndex] = nbrDataGetNbr(&value[srcIndex], nbrPtr);
}

void pipelineAverageOperation(uint8 index) {
	uint8 currentValue = nbrDataGet(&value[index]);
	uint8 newValue = (currentValue + tempValue[index]) / 2;

	nbrDataSet(&value[index], newValue);
}

void pipelineAveragePrintCell(uint8 index) {
	cprintf("%d-", nbrDataGet(&value[index]));
}

void pipelineAverageDataInit(void) {
	uint8 i;
	for (i = 0; i < CONSENSUS_PIPELINE_AVG_SIZE; i++) {
		nbrDataCreate(&value[i], "cpAvg", 8, 0);
	}

	consensusPipelineInit(CONSENSUS_PIPELINE_AVG_SIZE,
			pipelineAverageInput,
			pipelineAverageStoreTempData,
			pipelineAverageOperation);
}
