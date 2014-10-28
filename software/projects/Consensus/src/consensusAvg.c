/*
 * consensusAvg.c
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#include "consensus.h"

uint8 tempValue;
NbrData value;

void averageStoreTempData(Nbr *nbrPtr) {
	/* Store temporary value */
	tempValue = nbrDataGetNbr(&value, nbrPtr);
}

void averageOperation() {
	uint8 currentValue = nbrDataGet(&value);
	uint8 newValue = (currentValue + tempValue) / 2;

	nbrDataSet(&value, newValue);

	cprintf("New value after consensus: %d\n", newValue);
}

void averageDataInit() {
	nbrDataCreate(&value, "cAvg", 8, rand() % 200);

	consensusInit(averageStoreTempData, averageOperation);
}
