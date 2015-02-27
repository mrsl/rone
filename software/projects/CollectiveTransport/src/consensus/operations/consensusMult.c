/*
 * consensusMult.c
 *
 *  Created on: Oct 28, 2014
 *      Author: Golnaz
 */

#include "../consensus.h"

NbrData consensusMultData;
uint8 consensusMultTempData;

void consensusMultStoreTempData(Nbr *nbrPtr) {
	/* Store temporary value */
	consensusMultTempData = nbrDataGetNbr(&consensusMultData, nbrPtr);
}

void consensusMultOperation(void) {
	uint8 ourData = nbrDataGet(&consensusMultData);

	uint8 newValue = ourData * consensusMultTempData;

	nbrDataSet(&consensusMultData, newValue);
}

void consensusMultInit(void) {
	//nbrDataCreate(&consensusMultData, 8, 10);

	consensusInit(consensusMultStoreTempData, consensusMultOperation);
}
