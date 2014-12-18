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

/**
 *
 */
centroidData centroidEstInputValue;
centroidData centroidEstTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
centroidNbrData centroidEstValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

/**
 *
 */
float posDiffInputValue;
float posDiffTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat posDiffValue[CONSENSUS_PIPELINE_MINMAX_SIZE];

/**
 *
 */
float posMultInputValue;
float posMultTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat posMultValue[CONSENSUS_PIPELINE_MINMAX_SIZE];




/**
 * width is defined as the minimum width of the object
 */
float widthInputValue;
float widthTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat widthValue[CONSENSUS_PIPELINE_MINMAX_SIZE];


/**
 * Diameter is defined as the maximum width(diameter) of the object
 */
float diameterInputValue;
float diameterTempValue[CONSENSUS_PIPELINE_MINMAX_SIZE];
NbrDataFloat diameterValue[CONSENSUS_PIPELINE_MINMAX_SIZE];




/**
 * Prints out the contents of the pipeline from head to tail.
 */
void consensusPipelineMinMaxPrintValues(void) {
	float centroidX, centroidY;
	float posDiff;
	float posMult;

	consensusPipelineMinMaxGetCentroid(&centroidX, &centroidY);

	consensusPipelineMinMaxGetPosDiff(&posDiff);
	consensusPipelineMinMaxGetPosMult(&posMult);
	if ((abs(posDiff)<1) && (abs(2* posMult)<1))
	{
		posDiff = posDiff*100;
		posMult = posMult*100;

	}

	int16 object_orient = (1 * atan2MilliRad((int32)(2*posMult), (int32)posDiff))/2;

	char buffer[100];
	sprintf(buffer, "CX:%.3f CY:%.3f PD:%.3f PM:%.3f OD:%d\n", centroidX, centroidY, posDiff, posMult, object_orient);
	//printf(buffer, "CX:%.3f CY:%.3f PD:%..3f PM:%.3f OD:%d\n", centroidX, centroidY, posDiff, posMult, object_orient);

	rprintf(buffer);

	rprintfFlush();
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

void consensusPipelineMinMaxSetPosDiff(float newPosDiff) {
	posDiffInputValue = newPosDiff;
}

void consensusPipelineMinMaxSetPosMult(float newPosMult) {
	posMultInputValue = newPosMult;
}



void consensusPipelineMinMaxSetWidth(float newWidth) {
	widthInputValue = newWidth;
}


void consensusPipelineMinMaxSetDiameter(float newDiameter) {
	diameterInputValue = newDiameter;
}

void consensusPipelineMinMaxInput(uint8 index) {
	//
	centroidNbrDataCopy(&centroidEstInputValue, &centroidEstValue[index]);

	//
	nbrDataSetFloat(&posDiffValue[index], posDiffInputValue);

	//
	nbrDataSetFloat(&posMultValue[index], posMultInputValue);

	//

	nbrDataSetFloat(&widthValue[index], posMultInputValue);

		//

	nbrDataSetFloat(&diameterValue[index], posMultInputValue);

}

void consensusPipelineMinMaxStoreTempData(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex) {
	//
	centroidValue x, y;
	centroidNbrDataGetNbr(&x, &y, &centroidEstValue[srcIndex], nbrPtr);

	centroidTransform(&x, &y, nbrPtr);

	centroidDataSet(x, y, &centroidEstTempValue[destIndex]);

	//
	posDiffTempValue[destIndex] = nbrDataGetNbrFloat(&posDiffValue[srcIndex], nbrPtr);

	//
	posMultTempValue[destIndex] = nbrDataGetNbrFloat(&posMultValue[srcIndex], nbrPtr);

	//
	widthTempValue[destIndex] = nbrDataGetNbrFloat(&widthValue[srcIndex], nbrPtr);

	//
	diameterTempValue[destIndex] = nbrDataGetNbrFloat(&diameterValue[srcIndex], nbrPtr);
}

void consensusPipelineMinMaxCentroidOperation(uint8 index) {
	/* Get our value */
	centroidValue ox, oy, tx, ty;
	centroidNbrDataGet(&ox, &oy, &centroidEstValue[index]);
	centroidDataGet(&tx, &ty, &centroidEstTempValue[index]);

	if (tx == CONSENSUS_PIPELINE_MINMAX_INV
		|| ty == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	centroidValue nx = (ox + tx) / ((centroidValue) 2.0);
	centroidValue ny = (oy + ty) / ((centroidValue) 2.0);

	centroidNbrDataSet(nx, ny, &centroidEstValue[index]);

	centroidDataSet(CONSENSUS_PIPELINE_MINMAX_INV,
					CONSENSUS_PIPELINE_MINMAX_INV,
					&centroidEstTempValue[index]);
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
}


void consensusPipelineWidthDiameterOperation(uint8 index) {
	float currentValue = nbrDataGetFloat(&widthValue[index]);

	/* Get the stored value */
	float theirValue = widthTempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}

	float newValue;

	/* Take the maximum value from our value and the temporary value  */
	if (currentValue < theirValue)
		{ newValue = theirValue;
		}
	else
		{
		 newValue = currentValue;

		}

	/* Set our new value */
	nbrDataSetFloat(&widthValue[index], newValue);

	/* Put invalid value back in place */
	widthTempValue[index] = CONSENSUS_PIPELINE_MINMAX_INV;


	//////
	 currentValue = nbrDataGetFloat(&diameterValue[index]);

	/* Get the stored value */
	 theirValue = diameterTempValue[index];

	/* Stored value is invalid */
	if (theirValue == CONSENSUS_PIPELINE_MINMAX_INV) {
		return;
	}


	/* Take the maximum value from our value and the temporary value together */
	if (currentValue < theirValue)
	{ newValue = theirValue;}
	else
	{
		 newValue = currentValue;

	}

	/* Set our new value */
	nbrDataSetFloat(&diameterValue[index], newValue);

	/* Put invalid value back in place */
	diameterTempValue[index] = CONSENSUS_PIPELINE_MINMAX_INV;


	}




void consensusPipelineMinMaxOperation(uint8 index) {
	consensusPipelineMinMaxCentroidOperation(index);
	consensusPipelineMinMaxAvgOperation(index);
	consensusPipelineWidthDiameterOperation(index);



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
						CONSENSUS_PIPELINE_MINMAX_INV,
						&centroidEstTempValue[i]);
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

	widthInputValue = CONSENSUS_PIPELINE_MINMAX_INV;


	for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
			/* Create neighbor data */
			nbrDataCreateFloat(&widthValue[i], "cpAvg");
			/* Initialize temp storage to nulls */
			widthTempValue[i] = CONSENSUS_PIPELINE_MINMAX_INV;
		}

		/* Random input value */
		diameterInputValue = CONSENSUS_PIPELINE_MINMAX_INV;




		for (i = 0; i < CONSENSUS_PIPELINE_MINMAX_SIZE; i++) {
				/* Create neighbor data */
				nbrDataCreateFloat(&diameterValue[i], "cpAvg");
				/* Initialize temp storage to nulls */
				diameterTempValue[i] = CONSENSUS_PIPELINE_MINMAX_INV;
			}



	//consensusPipelineSetPrintFunction(consensusPipelineMinMaxPrintValues);

	/* Initialize the pipeline */
	consensusPipelineInit(CONSENSUS_PIPELINE_MINMAX_SIZE,
			consensusPipelineMinMaxInput,
			consensusPipelineMinMaxStoreTempData,
			consensusPipelineMinMaxOperation);
}

