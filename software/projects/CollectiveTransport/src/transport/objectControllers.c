/*
 * objectControllers.c
 *
 *  Created on: Jan 17, 2015
 *      Author: Zak
 */

#include "objectControllers.h"

#define OBJECT_MAXIMUM_RV_GAIN	50
#define DIMENSIONAL_SCALE 3000000

int32 desiredBearing = 0;
int32 filteredTV = 0;

// in millirad/s, apriltag, millirad
void objectRotatePerpendicularLeft(int32 objectRV, Beh *behPtr) {

	int32 bearing = centroidLiteGetBearing();
	int32 distance = centroidLiteGetDistance();

	int32 bearingPerpForward = normalizeAngleMilliRad2(bearing - MILLIRAD_HALF_PI);
	int32 bearingPerpBackward = normalizeAngleMilliRad2(bearingPerpForward - MILLIRAD_PI);

	int32 desiredTV = (2 * MILLIRAD_PI * distance * objectRV) / DIMENSIONAL_SCALE;

	int32 bearingPerp;
	if (abs(bearingPerpForward) <= abs(bearingPerpBackward)) {
		bearingPerp = bearingPerpForward;
	} else {
		bearingPerp = bearingPerpBackward;
		desiredTV *= -1;
	}

	desiredBearing = filterIIRAngle(desiredBearing, bearingPerp, 20);

	if (abs(desiredBearing) >= (MILLIRAD_HALF_PI / 2)) {
		desiredTV /= 2;
	}

	filteredTV = filterIIR(filteredTV, desiredTV, 30);

	if (!leaderCanTV())  {
		behMoveForward(behPtr, 0);
	} else {
		behMoveForward(behPtr, filteredTV);
	}

	// Align to be facing perpendicular to vector as to rotate object.
	rvBearingController(behPtr, desiredBearing, OBJECT_MAXIMUM_RV_GAIN);
}

void objectRotatePerpendicularRight(int32 objectRV, Beh *behPtr) {

	int32 bearing = centroidLiteGetBearing();
	int32 distance = centroidLiteGetDistance();

	int32 bearingPerpForward = normalizeAngleMilliRad2(bearing + MILLIRAD_HALF_PI);
	int32 bearingPerpBackward = normalizeAngleMilliRad2(bearingPerpForward - MILLIRAD_PI);

	int32 desiredTV = (2 * MILLIRAD_PI * distance * objectRV) / DIMENSIONAL_SCALE;

	int32 bearingPerp;
	if (abs(bearingPerpForward) <= abs(bearingPerpBackward)) {
		bearingPerp = bearingPerpForward;
	} else {
		bearingPerp = bearingPerpBackward;
		desiredTV *= -1;
	}

	desiredBearing = filterIIRAngle(desiredBearing, bearingPerp, 20);

	if (abs(desiredBearing) >= (MILLIRAD_HALF_PI / 2)) {
		desiredTV /= 2;
	}

	filteredTV = filterIIR(filteredTV, desiredTV, 30);

	if (!leaderCanTV())  {
		behMoveForward(behPtr, 0);
	} else {
		behMoveForward(behPtr, filteredTV);
	}

	// Align to be facing perpendicular to vector as to rotate object.
	rvBearingController(behPtr, desiredBearing, OBJECT_MAXIMUM_RV_GAIN);
}

void objectTranslateGuide(int32 objectTV, Beh *behPtr) {
	int32 bearingForward = leaderGetGuideObjectBearing();
	int32 bearingBackward = normalizeAngleMilliRad2(bearingForward - MILLIRAD_PI);
	int32 desiredTV = objectTV;

	int32 bearing;
	if (abs(bearingForward) <= abs(bearingBackward)) {
		bearing = bearingForward;
	} else {
		bearing = bearingBackward;
		desiredTV *= -1;
	}
	desiredBearing = filterIIRAngle(desiredBearing, bearing, 20);

	if (abs(desiredBearing) >= (MILLIRAD_HALF_PI / 2)) {
		desiredTV /= 2;
	}

	filteredTV = filterIIR(filteredTV, desiredTV, 30);

	if (!leaderCanTV())  {
		behMoveForward(behPtr, 0);
	} else {
		behMoveForward(behPtr, filteredTV);
	}

	// Align to be facing perpendicular to vector as to rotate object.
	rvBearingController(behPtr, desiredBearing, OBJECT_MAXIMUM_RV_GAIN);
}
