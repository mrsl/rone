/*
 * mrmHelpers.c
 * Helper functions for MRM
 *  Created on: Sep 16, 2014
 *      Author: Zak
 */

#include "globalTreeCOM.h"

// State machine stuff
uint32 startNbrRound = 0;
uint8 mrmFSMState = STATE_IDLE;

navigationData ravg[RAVG_SIZE];
int ravg_ind = 0;
int ravg_size = 0;

void setStartNbrRound(uint32 nbrRound) {
	startNbrRound = nbrRound;
}

boolean isInitStartNbrRound() {
	return (startNbrRound == 0) ? FALSE : TRUE;
}

uint32 getDeltaStartNbrRound(uint32 nbrRound) {
	return nbrRound - startNbrRound;
}

/**
 * Initialize a navigationData struct
 */
void navDataInit(navigationData *navData) {
	navData->centroidX = 0;
	navData->centroidY = 0;
	navData->guideX = 0;
	navData->guideY = 0;
	navData->pivotX = 0;
	navData->pivotY = 0;
}

void copyNavData(navigationData *toCopy, navigationData *toMe) {
	toMe->centroidX = toCopy->centroidX;
	toMe->centroidY = toCopy->centroidY;
	toMe->guideX = toCopy->guideX;
	toMe->guideY = toCopy->guideY;
	toMe->pivotX = toCopy->pivotX;
	toMe->pivotY = toCopy->pivotY;
	toMe->childCountSum = toCopy->childCountSum;
}

int32 mrmIIR(int32 currentVal, int32 newVal, int32 alpha) {
	int32 temp = (newVal * alpha) + ((100 - alpha) * currentVal);
	return temp / 100;
}

void rollingAverageNavData(navigationData *new, navigationData *avg) {
	int i;
	int32 x, y;

	copyNavData(new, &ravg[ravg_ind]);

	ravg_ind = (ravg_ind + 1) % RAVG_SIZE;

	if (ravg_size < RAVG_SIZE) {
		ravg_size++;
	}

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].centroidX;
		y += (int32) ravg[i].centroidY;
	}
	avg->centroidX = (int16) (x / ravg_size);
	avg->centroidY = (int16) (y / ravg_size);

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].pivotX;
		y += (int32) ravg[i].pivotY;
	}
	avg->pivotX = (int16) (x / ravg_size);
	avg->pivotY = (int16) (y / ravg_size);

	x = 0;
	y = 0;
	for (i = 0; i < ravg_size; i++) {
		x += (int32) ravg[i].guideX;
		y += (int32) ravg[i].guideY;
	}
	avg->guideX = (int16) (x / ravg_size);
	avg->guideY = (int16) (y / ravg_size);
}

