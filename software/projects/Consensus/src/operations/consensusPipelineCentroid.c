///*
// * consensusPipelineCentroid.c
// *
// *  Created on: Nov 8, 2014
// *      Author: Zak
// */
//
//#include "../consensus/consensusPipeline.h"
//#include "../util/centroidData.h"
//#include <stdio.h>
//
//#define CONSENSUS_PIPELINE_CENTROID_INV		-1.0	// Invalid value
//#define CONSENSUS_PIPELINE_CENTROID_SIZE	20		// Size of the pipeline
//
//centroidData tempValue[CONSENSUS_PIPELINE_CENTROID_SIZE];	// Temporary data array
//centroidNbrData value[CONSENSUS_PIPELINE_CENTROID_SIZE];	// The pipeline itself
//
///**
// * Initialize centroid neighbor data
// */
//void pipelineCentroidCreateData(uint8 index) {
//	nbrDataCreateFloat(&value[index].x, "cpCx");
//	nbrDataCreateFloat(&value[index].y, "cpCy");
//}
//
///**
// * Get the XY value of a centroid coordinate from your data
// */
//void pipelineCentroidGetXY(float *x, float *y, uint8 index) {
//	*x = nbrDataGetFloat(&value[index].x);
//	*y = nbrDataGetFloat(&value[index].y);
//}
//
///**
// * Set the XY value of a centroid coordinate in your data
// */
//void pipelineCentroidSetXY(float x, float y, uint8 index) {
//	nbrDataSetFloat(&value[index].x, x);
//	nbrDataSetFloat(&value[index].y, y);
//}
//
///**
// * Get the XY value of a centroid coordinate from the temporary data
// */
//void pipelineCentroidGetTempXY(float *x, float *y, uint8 index) {
//	*x = tempValue[index].x;
//	*y = tempValue[index].y;
//}
//
///**
// * Set the XY value of a centroid coordinate in the temporary data
// */
//void pipelineCentroidSetTempXY(float x, float y, uint8 index) {
//	tempValue[index].x = x;
//	tempValue[index].y = y;
//}
//
///**
// * Get the XY value of a centroid coordinate from a neighbors data
// */
//void pipelineCentroidGetNbrXY(float *x, float *y, Nbr *nbrPtr, uint8 index) {
//	*x = nbrDataGetNbrFloat(&value[index].x, nbrPtr);
//	*y = nbrDataGetNbrFloat(&value[index].y, nbrPtr);
//}
//
///**
// * Transforms a neighbors XY into a local XY coordinate
// */
//void pipelineCentroidTransform(float *x, float *y, Nbr *nbrPtr) {
//	/* Get neighbor position information */
//	int32 orientation = nbrGetOrientation(nbrPtr);
//	int32 bearing = nbrGetBearing(nbrPtr);
//	int32 distance = externalPoseGetNbrRange(nbrPtr) * 10;
//
//	/* Do the rigid transform into our coordinate frame */
//	int16 xCoor = (int16) *x;
//	int16 yCoor = (int16) *y;
//
//	int16 xNbr = (int16) (distance * cosMilliRad(bearing) / MILLIRAD_TRIG_SCALER);
//	int16 yNbr = (int16) (distance * sinMilliRad(bearing) / MILLIRAD_TRIG_SCALER);
//
//	int32 transformationAngle = -normalizeAngleMilliRad2(MILLIRAD_PI - orientation + bearing);
//
//	int16 xTemp = (xCoor * cosMilliRad(transformationAngle)
//			- yCoor * sinMilliRad(transformationAngle)) / MILLIRAD_TRIG_SCALER
//			+ xNbr;
//	int16 yTemp = (xCoor * sinMilliRad(transformationAngle)
//			+ yCoor * cosMilliRad(transformationAngle)) / MILLIRAD_TRIG_SCALER
//			+ yNbr;
//
//	*x = ((float) xTemp) / 10.;
//	*y = ((float) yTemp) / 10.;
//}
//
///**
// * Prints out the contents of the pipeline from head to tail.
// */
//void pipelineCentroidPrintPipeline(void) {
//	char tempBuffer[30];
//
//	/* Print out the input value and the current value we have */
//	uint8 oldIndex = consensusPipelineGetOldestIndex();
//
//	float x, y;
//	pipelineCentroidGetXY(&x, &y, oldIndex);
//
//	sprintf(tempBuffer, "%d,%d,%.3f,%.3f\n", (int16) x, (int16) y, x, y);
//	rprintf(tempBuffer);
//	rprintfFlush();
//
//	cprintf("pt 3,%d,%d\n", (int16) x, (int16) y);
//}
//
///**
// * The input function for average consensus. Continually provides the same
// * input value. Stores data into requested index.
// *
// * @param index
// * 		The position in the array to store the input value into.
// */
//void pipelineCentroidInput(uint8 index) {
//	pipelineCentroidSetXY(0, 0, index);
//}
//
///**
// * Stores temporary data from a neighbor's pipeline into our temporary data
// * storage. Also transforms data into our local coordinate frame.
// *
// * @param nbrPtr
// * 		The neighbor to look up data from
// *
// * @param srcIndex
// * 		The index in the neighbors pipeline to save data from
// *
// * @param destIndex
// * 		The destination index in the temporary data storage buffer to save the
// * 		data from.
// */
//void pipelineCentroidStoreTempData(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex) {
//	float x, y;
//	/* Get data from neighbor */
//	pipelineCentroidGetNbrXY(&x, &y, nbrPtr, srcIndex);
//
//	/* Transform into our reference frame */
//	pipelineCentroidTransform(&x, &y, nbrPtr);
//
//	/* Set new locations */
//	pipelineCentroidSetTempXY(x, y, destIndex);
//}
//
//
//
///**
// * Averages two coordinate in our local coordinate frame together.
// *
// * @param index
// * 		The index from the pipeline arrays to use.
// */
//void pipelineCentroidOperation(uint8 index) {
//	/* Get our current value */
//	float x, y;
//	pipelineCentroidGetXY(&x, &y, index);
//
//	/* Get the stored value */
//	float tx, ty;
//	pipelineCentroidGetTempXY(&tx, &ty, index);
//
//	/* Stored value is invalid */
//	if (tx == CONSENSUS_PIPELINE_CENTROID_INV
//		|| ty == CONSENSUS_PIPELINE_CENTROID_INV) {
//		return;
//	}
//
//	/* Average our value and the temporary value together */
//	float nx = (x + tx) / 2.0;
//	float ny = (y + ty) / 2.0;
//
//	/* Set our new value */
//	pipelineCentroidSetXY(nx, ny, index);
//
//	/* Put invalid value back in place */
//	pipelineCentroidSetTempXY(CONSENSUS_PIPELINE_CENTROID_INV,
//							  CONSENSUS_PIPELINE_CENTROID_INV,
//							  index);
//}
//
//void pipelineCentroidInit(void) {
//	/* Initialize our data */
//	uint8 i;
//	for (i = 0; i < CONSENSUS_PIPELINE_CENTROID_SIZE; i++) {
//		/* Create neighbor data */
//		pipelineCentroidCreateData(i);
//
//		/* Initialize temp storage to nulls */
//		pipelineCentroidSetTempXY(CONSENSUS_PIPELINE_CENTROID_INV,
//								  CONSENSUS_PIPELINE_CENTROID_INV,
//								  i);
//	}
//
//	/* Initialize the external pose subsystem */
//	externalPoseInit();
//
//	consensusPipelineSetPrintFunction(pipelineCentroidPrintPipeline);
//
//	/* Initialize the pipeline */
//	consensusPipelineInit(CONSENSUS_PIPELINE_CENTROID_SIZE,
//			pipelineCentroidInput,
//			pipelineCentroidStoreTempData,
//			pipelineCentroidOperation);
//}
