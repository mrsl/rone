/*
 * centroidData.c
 *
 *  Created on: Nov 11, 2014
 *      Author: Golnaz
 */

#include "roneos.h"
#include "ronelib.h"
#include "centroidData.h"

void centroidNbrValueCreate(centroidNbrValue *value, char *name) {
	uint8 i;
	for (i = 0; i < CENTROID_VALUE_SIZE; i++) {
		nbrDataCreate(&value->data[i], name, 8, 0);
	}
}

void centroidNbrValueSet(centroidNbrValue *value, centroidValue newValue) {
	centroidValueConvert convert;
	convert.val = newValue;

	uint8 i;
	for (i = 0; i < CENTROID_VALUE_SIZE; i++) {
		nbrDataSet(&value->data[i], convert.bytes[i]);
	}
}

centroidValue centroidNbrValueGet(centroidNbrValue *value) {
	centroidValueConvert convert;

	uint8 i;
	for (i = 0; i < CENTROID_VALUE_SIZE; i++) {
		convert.bytes[i] = nbrDataGet(&value->data[i]);
	}

	return convert.val;
}

centroidValue centroidNbrValueGetNbr(centroidNbrValue *value, Nbr *nbrPtr) {
	centroidValueConvert convert;

	uint8 i;
	for (i = 0; i < CENTROID_VALUE_SIZE; i++) {
		convert.bytes[i] = nbrDataGetNbr(&value->data[i], nbrPtr);
	}

	return convert.val;
}

void centroidNbrDataCreate(centroidNbrData *data) {
	centroidNbrValueCreate(&data->x, "cdX");
	centroidNbrValueCreate(&data->y, "cdY");
}

void centroidNbrDataSet(centroidValue x, centroidValue y, centroidNbrData *data) {
	centroidNbrValueSet(&data->x, x);
	centroidNbrValueSet(&data->y, y);
}

void centroidNbrDataGet(centroidValue *x, centroidValue *y, centroidNbrData *data) {
	*x = centroidNbrValueGet(&data->x);
	*y = centroidNbrValueGet(&data->y);
}

void centroidNbrDataGetNbr(centroidValue *x, centroidValue *y, centroidNbrData *data, Nbr *nbrPtr) {
	*x = centroidNbrValueGetNbr(&data->x, nbrPtr);
	*y = centroidNbrValueGetNbr(&data->y, nbrPtr);
}

void centroidTransform(centroidValue *x, centroidValue *y, Nbr *nbrPtr) {
	int32 orientation = nbrGetOrientation(nbrPtr);
	int32 bearing = nbrGetBearing(nbrPtr);
	int32 distance = externalPoseGetNbrRange(nbrPtr);

	/* Do the rigid transform into our coordinate frame */
	int16 xCoor = (int16) *x;
	int16 yCoor = (int16) *y;

	int16 xNbr = (int16) (distance * cosMilliRad(bearing) / MILLIRAD_TRIG_SCALER);
	int16 yNbr = (int16) (distance * sinMilliRad(bearing) / MILLIRAD_TRIG_SCALER);

	int32 transformationAngle = normalizeAngleMilliRad2(MILLIRAD_PI - orientation + bearing);

	int16 xTemp = (xCoor * cosMilliRad(transformationAngle)
			- yCoor * sinMilliRad(transformationAngle)) / MILLIRAD_TRIG_SCALER
			+ xNbr;
	int16 yTemp = (xCoor * sinMilliRad(transformationAngle)
			+ yCoor * cosMilliRad(transformationAngle)) / MILLIRAD_TRIG_SCALER
			+ yNbr;

	*x = (centroidValue) xTemp;
	*y = (centroidValue) yTemp;
}

void centroidNbrDataCopy(centroidData *toCopy, centroidNbrData *toHere) {
	centroidNbrDataSet(toCopy->x, toCopy->y, toHere);
}

void centroidNbrDataPaste(centroidNbrData *toPaste, centroidData *toHere) {
	centroidValue x, y;
	centroidNbrDataGet(&x, &y, toPaste);
	centroidDataSet(x, y, toHere);
}

void centroidNbrDataPasteNbr(centroidNbrData *toPaste, centroidData *toHere, Nbr *nbrPtr) {
	centroidValue x, y;
	centroidNbrDataGetNbr(&x, &y, toPaste, nbrPtr);
	centroidDataSet(x, y, toHere);
}

void centroidDataSet(centroidValue x, centroidValue y, centroidData *data) {
	data->x = x;
	data->y = y;
}

void centroidDataGet(centroidValue *x, centroidValue *y, centroidData *data) {
	*x = data->x;
	*y = data->y;
}

void centroidDataCopy(centroidData *toCopy, centroidData *toHere) {
	centroidDataSet(toCopy->x, toCopy->y, toHere);
}

centroidValue centroidValueIIR(centroidValue new, centroidValue old, centroidValue alpha) {
	centroidValue hund = (centroidValue) 100.0;
	centroidValue temp = new * alpha + old * (hund - alpha);

	return temp / hund;
}

void centroidDataIIR(centroidData *new, centroidData *old, centroidValue alpha) {
	centroidValue newX, newY, oldX, oldY;
	centroidDataGet(&newX, &newY, new);
	centroidDataGet(&oldX, &oldY, old);

	newX = centroidValueIIR(newX, oldX, alpha);
	newY = centroidValueIIR(newY, oldY, alpha);

	centroidDataSet(newX, newY, new);
}
