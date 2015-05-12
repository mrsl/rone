/*
 * centroidLite.c
 *
 *  Created on: Jan 16, 2015
 *      Author: Zak
 */

#include "centroidLite.h"

centroidData centroidLite;
int32 centroidLiteBearing = MILLIRAD_HALF_PI;
boolean centroidLiteActive = FALSE;

void centroidLiteCalculateCentroid(NbrList *nbrListPtr) {
	centroidValue centroidX = 0.;
	centroidValue centroidY = 0.;
	uint8 pointCount = 1;

	uint8 i;
	for (i = 0; i < nbrListPtr->size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(nbrListPtr, i);

		if (nbrPtr == NULL) {
			continue;
		}

		if (!nbrDataGetNbr(&isActive, nbrPtr)) {
			continue;
		}

		centroidValue nbrX = 0.;
		centroidValue nbrY = 0.;

		centroidTransform(&nbrX, &nbrY, nbrPtr);

		centroidX += nbrX;
		centroidY += nbrY;

		pointCount++;
	}

	centroidX /= (centroidValue) pointCount;
	centroidY /= (centroidValue) pointCount;

	centroidDataSet(centroidX, centroidY, &centroidLite);

	centroidLiteActive = (pointCount > 1) ? TRUE : FALSE;
}

boolean centroidLiteIsActive() {
	return centroidLiteActive;
}

void centroidLiteCalculateCentroidBearing() {
	centroidValue centroidX, centroidY;
	centroidDataGet(&centroidX, &centroidY, &centroidLite);

	int16 iCentroidX = (int16) centroidX;
	int16 iCentroidY = (int16) centroidY;

	int32 tCentroidBearing = normalizeAngleMilliRad2(atan2MilliRad(iCentroidY, iCentroidX));

	centroidLiteBearing = filterIIR(centroidLiteBearing, tCentroidBearing, 30);
}

void centroidLiteCallback(NbrDatabase* ndPtr) {
	NbrList nbrList;
	nbrListCreate(&nbrList);

	centroidLiteCalculateCentroid(&nbrList);
	centroidLiteCalculateCentroidBearing();
}

centroidValue centroidLiteGetXCoordinate() {
	if (centroidLiteActive) {
		centroidValue centroidX, centroidY;
		centroidDataGet(&centroidX, &centroidY, &centroidLite);

		return centroidX;
	} else {
		return 0.;
	}
}

centroidValue centroidLiteGetYCoordinate() {
	if (centroidLiteActive) {
		centroidValue centroidX, centroidY;
		centroidDataGet(&centroidX, &centroidY, &centroidLite);

		return centroidY;
	} else {
		return 0.;
	}
}

int32 centroidLiteGetDistance() {
	centroidValue centroidX, centroidY;
	centroidDataGet(&centroidX, &centroidY, &centroidLite);

	int16 iCentroidX = (int16) centroidX;
	int16 iCentroidY = (int16) centroidY;

	return sqrtInt(iCentroidX * iCentroidX + iCentroidY * iCentroidY);
}

int32 centroidLiteGetBearing() {
	return centroidLiteBearing;
}

void centroidLiteInit() {
	centroidLite = (centroidData) {0., 0.};
	neighborsAddReceiveCallback(centroidLiteCallback);
}
