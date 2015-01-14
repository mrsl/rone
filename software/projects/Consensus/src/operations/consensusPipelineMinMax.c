/*
 * consensusPipelineMinMax.c
 *
 *  Created on: Dec 10, 2014
 *      Author: Golnaz
 */

#include "../consensus/consensusPipeline.h"
#include "./consensusPipelineMinMax.h"
#include "../util/centroidData.h"
#include <stdio.h>

#define CONSENSUS_PIPELINE_MINMAX_INV	-1.0	// Invalid value
#define CONSENSUS_PIPELINE_MINMAX_SIZE	10

//
centroidData centroidEstInputValue;
centroidData centroidEstTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
centroidNbrData centroidEstValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

//
float posDiffInputValue;
float posDiffTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat posDiffValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

//
float posMultInputValue;
float posMultTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat posMultValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

//
float widthInputValue;
float widthTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat widthValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

//
float diameterInputValue;
float diameterTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat diameterValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

/**
 * Prints out the contents of the pipeline from head to tail.
 */
void consensusPipelineMinMaxPrintValues(void) {
	float cX, cY;
	float pDiff;
	float pMult;
	float cW, cD;

	// Retrieve values
	consensusPipelineMinMaxGetCentroid(&cX, &cY);
	consensusPipelineMinMaxGetPosDiff(&pDiff);
	consensusPipelineMinMaxGetPosMult(&pMult);
	consensusPipelineMinMaxGetWidth(&cW);
	consensusPipelineMinMaxGetDiameter(&cD);

	// Scale appropriately
	if ((abs(pDiff) < 1) && (abs(2 * pMult) < 1)) {
		pDiff = pDiff * 100;
		pMult = pMult * 100;
	}

	// Object orientation
	int16 oO = atan2MilliRad((int32) (2 * pMult), (int32) pDiff) / 2;

	char outBuffer[100];	// Output buffer and format
	char outBufferFormat[50] = "X:%.3f Y:%.3f D:%.3f M:%.3f O:%d W:%.3f D:%.3f\n";
	sprintf(outBuffer, outBufferFormat, cX, cY, pDiff, pMult, oO, cW, cD);

	rprintf(outBuffer);
	rprintfFlush();

	consensusPipelineMinMaxSetPosDiff(cX * cX - cY * cY);
	consensusPipelineMinMaxSetPosMult(cX * cY);

	float tcX = cX;
	float tcY = cY;

	if ((tcX < 1.) && (tcY < 1.)) {
		tcX *= 100.;
		tcY *= 100.;
	}

	float cDist = (float) vectorMag((int32) cX, (int32) cY);
	int16 cBear = atan2MilliRad((int32) tcY ,(int32) tcY);

	int16 ang = sinMilliRad(normalizeAngleMilliRad2(cBear - oO));
	float factor = ((float) ang / (float) MILLIRAD_TRIG_SCALER);

	float w = cDist * factor;

//	sprintf(outBuffer, "%.3f, %d, %d, %d, %.3f, %.3f\n", cDist, cBear, oO, ang, w, factor);
//	cprintf(outBuffer);

	consensusPipelineMinMaxSetWidth(w);
	consensusPipelineMinMaxSetDiameter(cDist);
}

void consensusPipelineMinMaxSetPosMult(float new) {
	posMultInputValue = new;
}

void consensusPipelineMinMaxSetPosDiff(float new) {
	posDiffInputValue = new;
}

void consensusPipelineMinMaxSetWidth(float new) {
	widthInputValue = new;
}

void consensusPipelineMinMaxSetDiameter(float new) {
	diameterInputValue = new;
}

void consensusPipelineMinMaxGetCentroid(float *x, float *y) {
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	centroidNbrDataGet(x, y, &centroidEstValue[oldIndex]);
}

void consensusPipelineMinMaxGetPosDiff(float *x) {
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	*x = nbrDataGetFloat(&posDiffValue[oldIndex]);
}

void consensusPipelineMinMaxGetPosMult(float *x) {
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	*x = nbrDataGetFloat(&posMultValue[oldIndex]);
}

void consensusPipelineMinMaxGetWidth(float *x) {
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	*x = nbrDataGetFloat(&widthValue[oldIndex]);
}

void consensusPipelineMinMaxGetDiameter(float *x) {
	uint8 oldIndex = consensusPipelineGetOldestIndex();

	*x = nbrDataGetFloat(&diameterValue[oldIndex]);
}

void consensusPipelineMinMaxInput(uint8 index) {
	//
	centroidNbrDataCopy(&centroidEstInputValue, &centroidEstValue[index]);

	//
	nbrDataSetFloat(&posDiffValue[index], posDiffInputValue);

	//
	nbrDataSetFloat(&posMultValue[index], posMultInputValue);

	//
	nbrDataSetFloat(&widthValue[index], widthInputValue);

	//
	nbrDataSetFloat(&diameterValue[index], diameterInputValue);
}

void consensusPipelineMinMaxStoreTempData(Nbr *nbrPtr, uint8 srcIndex,
		uint8 destIndex) {

	//
	centroidValue x, y;
	centroidNbrDataGetNbr(&x, &y, &centroidEstValue[srcIndex], nbrPtr);

	centroidTransform(&x, &y, nbrPtr);

	centroidDataSet(x, y, &centroidEstTempValue[destIndex]);

	//
	posDiffTempValue[destIndex] = nbrDataGetNbrFloat(&posDiffValue[srcIndex],
			nbrPtr);

	//
	posMultTempValue[destIndex] = nbrDataGetNbrFloat(&posMultValue[srcIndex],
			nbrPtr);

	//
	widthTempValue[destIndex] = nbrDataGetNbrFloat(&widthValue[srcIndex],
			nbrPtr);

	//
	diameterTempValue[destIndex] = nbrDataGetNbrFloat(&diameterValue[srcIndex],
			nbrPtr);
}

void consensusPipelineMinMaxCentroidOperation(uint8 index) {
	/* Get our value */
	centroidValue oX, oY, tX, tY;
	centroidNbrDataGet(&oX, &oY, &centroidEstValue[index]);
	centroidDataGet(&tX, &tY, &centroidEstTempValue[index]);

	if (tX == CONSENSUS_PIPELINE_MINMAX_INV
			|| tY == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	centroidValue nx = (oX + tX) / ((centroidValue) 2.0);
	centroidValue ny = (oY + tY) / ((centroidValue) 2.0);

	centroidNbrDataSet(nx, ny, &centroidEstValue[index]);

	centroidDataSet(CONSENSUS_PIPELINE_MINMAX_INV,
	CONSENSUS_PIPELINE_MINMAX_INV, &centroidEstTempValue[index]);
}

void consensusPipelineMinMaxAvgOperation(uint8 index) {
	/* Get our current value */
	float currentValue = nbrDataGetFloat(&posDiffValue[index]);

	/* Get the stored value */
	float theirValue = posDiffTempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	/* Average our value and the temporary value together */
	float newValue = (currentValue + theirValue) / 2.0;

	/* Set our new value */
	nbrDataSetFloat(&posDiffValue[index], newValue);

	/* Put invalid value back in place */
	posDiffTempValue[index] = CONSENSUS_PIPELINE_MINMAX_INV;

	///////////////

	/* Get our current value */
	currentValue = nbrDataGetFloat(&posMultValue[index]);

	/* Get the stored value */
	theirValue = posMultTempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	/* Average our value and the temporary value together */
	newValue = (currentValue + theirValue) / 2.0;

	/* Set our new value */
	nbrDataSetFloat(&posMultValue[index], newValue);

	/* Put invalid value back in place */
	posMultTempValue[index] = CONSENSUS_PIPELINE_MINMAX_INV;

	///////////////

	/* Get our current value */
	currentValue = nbrDataGetFloat(&widthValue[index]);

	/* Get the stored value */
	theirValue = widthTempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	/* Find largest */
	newValue = (currentValue > theirValue) ? currentValue : theirValue;

	/* Set our new value */
	nbrDataSetFloat(&widthValue[index], newValue);

	/* Put invalid value back in place */
	widthTempValue[index] = CONSENSUS_PIPELINE_MINMAX_INV;

	///////////////

	/* Get our current value */
	currentValue = nbrDataGetFloat(&diameterValue[index]);

	/* Get the stored value */
	theirValue = diameterTempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	/* Find largest */
	newValue = (currentValue > theirValue) ? currentValue : theirValue;

	/* Set our new value */
	nbrDataSetFloat(&diameterValue[index], newValue);

	/* Put invalid value back in place */
	diameterTempValue[index] = CONSENSUS_PIPELINE_MINMAX_INV;
}

void consensusPipelineMinMaxOperation(uint8 index) {
	consensusPipelineMinMaxCentroidOperation(index);
	consensusPipelineMinMaxAvgOperation(index);
}

void consensusPipelineMinMaxInit(void) {
	/* Assign our input value */
	centroidDataSet(0, 0, &centroidEstInputValue);

	/* Initialize our data */
	uint8 i;
	for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
		centroidNbrDataCreate(&centroidEstValue[i]);
		centroidNbrDataCopy(&centroidEstInputValue, &centroidEstValue[i]);

		centroidDataSet(CONSENSUS_PIPELINE_MINMAX_INV,
		CONSENSUS_PIPELINE_MINMAX_INV, &centroidEstTempValue[i]);
	}

	/* Random input value */
	posDiffInputValue = CONSENSUS_PIPELINE_MINMAX_INV;

	/* Initialize our data */
	for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
		/* Create neighbor data */
		nbrDataCreateFloat(&posDiffValue[i], "cpAvg");
		/* Initialize temp storage to nulls */
		posDiffTempValue[i] = CONSENSUS_PIPELINE_MINMAX_INV;
	}

	/* Random input value */
	posMultInputValue = CONSENSUS_PIPELINE_MINMAX_INV;

	/* Initialize our data */
	for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
		/* Create neighbor data */
		nbrDataCreateFloat(&posMultValue[i], "cpAvg");
		/* Initialize temp storage to nulls */
		posMultTempValue[i] = CONSENSUS_PIPELINE_MINMAX_INV;
	}


	/* Random input value */
	widthInputValue = CONSENSUS_PIPELINE_MINMAX_INV;

	/* Initialize our data */
	for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
		/* Create neighbor data */
		nbrDataCreateFloat(&widthValue[i], "cpAvg");
		/* Initialize temp storage to nulls */
		widthTempValue[i] = CONSENSUS_PIPELINE_MINMAX_INV;
	}

	/* Random input value */
	diameterInputValue = CONSENSUS_PIPELINE_MINMAX_INV;

	/* Initialize our data */
	for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
		/* Create neighbor data */
		nbrDataCreateFloat(&diameterValue[i], "cpAvg");
		/* Initialize temp storage to nulls */
		diameterTempValue[i] = CONSENSUS_PIPELINE_MINMAX_INV;
	}

	consensusPipelineSetPrintFunction(consensusPipelineMinMaxPrintValues);

	/* Initialize the pipeline */
	consensusPipelineInit(CONSENSUS_PIPELINE_MINMAX_SIZE,
			consensusPipelineMinMaxInput, consensusPipelineMinMaxStoreTempData,
			consensusPipelineMinMaxOperation);
}

