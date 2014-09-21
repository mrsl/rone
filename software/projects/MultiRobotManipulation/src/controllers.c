/*
 * controllers.c
 *
 *  Created on: Sep 6, 2014
 *      Author: Zak
 */

#include "globalTreeCOM.h"

//#define MRM_RV_GAIN		40
#define MRM_RV_GAIN				1.2
#define MRM_RV_FLOCK_GAIN		15
#define ROTATION_DEADZONE		200
#define MRM_ROTATE_LEFT_BIAS	300

#define MRM_ALPHA1				50
#define MRM_ALPHA2				85

/**
 * Orbit the centroid, rotating the object about the centroid
 */
void mrmOrbitCentroid(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
	mrmPointOrbit(behPtr,
				 (int32) (navDataPtr->centroidX / 10),
				 (int32) (navDataPtr->centroidY / 10),
				 tvModifier);
}

/**
 * Orbit the pivot robot, rotating the object about the pivot
 */
void mrmOrbitPivot(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
	mrmPointOrbit(behPtr,
				 (int32) (navDataPtr->pivotX / 10),
				 (int32) (navDataPtr->pivotY / 10),
				 tvModifier);
}

/**
 * Custom flocking function to overlook guide robot
 */
int32 mrmFlockAngle(NbrList* nbrListPtr) {
	int32 i, x, y, alpha;
	Nbr* nbrPtr;

	x = 0;
	y = 0;
	for (i = 0; i < nbrListPtr->size; ++i) {
		nbrPtr = nbrListPtr->nbrs[i];
		// Skip if guide robot
		if (nbrGetID(nbrPtr) == getGuideRobot()) {
			continue;
		}
		if(nbrPtr->orientationValid) {
			alpha = normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation);
			x += cosMilliRad(alpha);
			y += sinMilliRad(alpha);
		}
	}

	if (nbrListPtr->size > 0) {
		alpha = normalizeAngleMilliRad2(atan2MilliRad(y, x));
	} else {
		alpha = 0;
	}

	return alpha;
}

/**
 * Orbit an arbitrary point in the robot's local reference frame.
 */
void mrmPointOrbit(Beh *behPtr, int32 x, int32 y, int32 tvModifier) {
	// Just stay put
	if (x == 0 && y == 0) {
		behSetTvRv(behPtr, 0, 0);
		return;
	}

	int32 tv = behGetTv(behPtr);
	int32 rv = behGetRv(behPtr);

	// Get bearing towards point
	int32 bearing = atan2MilliRad(y, x);

//	bearing -= MILLIRAD_HALF_PI;
//	tvModifier *= mrmChooseRotationDirection(&bearing);

	// Decided whether to drive forwards or backwards
	int32 bearingLeft = normalizeAngleMilliRad(bearing - MILLIRAD_PI / 2) - MILLIRAD_PI;
	int32 bearingRight = normalizeAngleMilliRad(bearing + MILLIRAD_PI / 2) - MILLIRAD_PI;

	if (abs(bearingLeft) < abs(bearingRight)) {
		bearing = bearingLeft;
	} else {
		bearing = bearingRight;
		tvModifier = -tvModifier;
	}

	// Proportional tv and rv control
	int32 distance = vectorMag(x, y) / 10; // In AprilTag units

//	int32 goalTv = boundAbs(tvModifier * distance, MRM_MAX_TV);
//	int32 goalRv = -smallestAngleDifference(0, bearing) * MRM_RV_GAIN / 100;

	int32 goalTv = boundAbs(tvModifier * distance, MRM_MAX_TV);
	int32 goalRv = 0;

	if (abs(bearing) > ROTATION_DEADZONE) {
		goalRv = MRM_RV_GAIN * bearing / 1.5;
		goalTv /= 1.5;
	}

	// Filter from previous state
	int32 finalTv = mrmIIR(goalTv, tv, MRM_ALPHA1);
	int32 finalRv = mrmIIR(goalRv, rv, MRM_ALPHA1);

	behSetTvRv(behPtr, finalTv, finalRv);
}

void mrmTranslateLeaderToGuide(navigationData *navDataPtr, NbrList *nbrListPtr,
									Beh *behPtr, int32 tvModifier) {

	int32 guideX = navDataPtr->guideX;
	int32 guideY = navDataPtr->guideY;

	int32 bearing;

	int32 goalTv, goalRv;

	int32 tv = behGetTv(behPtr);
	int32 rv = behGetRv(behPtr);

	if (roneID == getPivotRobot()) {
		// Get bearing towards guide
		bearing = atan2MilliRad(guideY, guideX);
	} else {
		bearing = mrmFlockAngle(nbrListPtr);
	}

	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;

	// Rotate and translate towards guide
	goalTv = tvModifier;
	goalRv = 0;

	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;
	if (abs(bearing) > ROTATION_DEADZONE) {
		goalRv = smallestAngleDifference(0, bearing) * MRM_RV_FLOCK_GAIN / 100;
		goalTv /= 1.5;
	}

//	goalTv = tvModifier;
//	goalRv = 0;
//
//	if (abs(bearing) > ROTATION_DEADZONE) {
//		goalRv = MRM_RV_GAIN * bearing / 1.5;
//		goalTv /= 1.5;
//	}


	// Filter from previous state
	int32 finalTv = mrmIIR(goalTv, tv, MRM_ALPHA1);
	int32 finalRv = mrmIIR(goalRv, rv, MRM_ALPHA1);

	behSetTvRv(behPtr, finalTv, finalRv);
}

void mrmTranslateLeaderToGuideVector(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {

	int32 guideX = navDataPtr->guideX;
	int32 guideY = navDataPtr->guideY;
	int32 centroidX = navDataPtr->centroidX;
	int32 centroidY = navDataPtr->centroidY;

	int32 bearing;

	int32 goalTv, goalRv;

	int32 tv = behGetTv(behPtr);
	int32 rv = behGetRv(behPtr);

	int32 x = guideX - centroidX;
	int32 y = guideY - centroidY;

	if (x == 0 && y == 0) {
		behSetTvRv(behPtr, 0, 0);
		return;
	}

	bearing = atan2MilliRad(y, x);

	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;

	// Rotate and translate towards guide
	goalTv = tvModifier;
	goalRv = 0;

	goalRv = smallestAngleDifference(0, bearing) * MRM_RV_FLOCK_GAIN / 100;
	if (abs(bearing) > ROTATION_DEADZONE) {
		goalTv /= 1.5;
	}

//	goalTv = tvModifier;
//	goalRv = 0;
//
//	if (abs(bearing) > ROTATION_DEADZONE) {
//		goalRv = MRM_RV_GAIN * bearing / 1.5;
//		goalTv /= 1.5;
//	}

	// Filter from previous state
//	int32 finalTv = mrmIIR(goalTv, tv, MRM_ALPHA2);
//	int32 finalRv = mrmIIR(goalRv, rv, MRM_ALPHA2);
//
//	behSetTvRv(behPtr, finalTv, finalRv);
	behSetTvRv(behPtr, goalTv, goalRv);
}

void mrmCycloidMotion(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
	int32 guideX = navDataPtr->guideX;
	int32 guideY = navDataPtr->guideY;
	int32 centroidX = navDataPtr->centroidX;
	int32 centroidY = navDataPtr->centroidY;

	// Find vector normal to the centroid vector (Clockwise)
	int32 rotationX = centroidX;
	int32 rotationY = centroidY;

	rotateXY32(&rotationX, &rotationY, -MILLIRAD_HALF_PI);

	// Find vector towards guide robot parallel to centroid-guide line (Look at translate)
	int32 translationX = guideX - centroidX;
	int32 translationY = guideY - centroidY;

	// Take the vector addition of these vectors
	int32 x = rotationX + translationX;
	int32 y = rotationY + translationY;

	if (x == 0 && y == 0) {
		behSetTvRv(behPtr, 0, 0);
		return;
	}

	// Find bearing
	int32 bearing = atan2MilliRad(y, x);
	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;

	int32 distance = vectorMag(x, y) / 10; // In AprilTag units
	// Rotate and translate towards guide
	int32 goalTv = boundAbs(tvModifier * distance, MRM_MAX_TV);
	int32 goalRv = 0;

	goalRv = smallestAngleDifference(0, bearing) * MRM_RV_FLOCK_GAIN / 100;
	if (abs(bearing) > ROTATION_DEADZONE) {
		goalTv /= 1.5;
	}

	int32 tv = behGetTv(behPtr);
	int32 rv = behGetRv(behPtr);

	// Filter from previous state
	//	int32 finalTv = mrmIIR(goalTv, tv, MRM_ALPHA2);
	//	int32 finalRv = mrmIIR(goalRv, rv, MRM_ALPHA2);
	//
	//	behSetTvRv(behPtr, finalTv, finalRv);
	behSetTvRv(behPtr, goalTv, goalRv);
}

//void mrmDisperse(navigationData *navData, Beh *beh, int32 tvModifier) {
//	int32 centroidX = (int32) (navData->centroidX / 10);
//	int32 centroidY = (int32) (navData->centroidY / 10);
//
//	if (centroidX == 0 && centroidY == 0) {
//		behSetTvRv(beh, 0, 0);
//		return;
//	}
//
//	int32 bearing = atan2MilliRad(centroidY, centroidX) - MILLIRAD_PI;
//
//	int32 distance = vectorMag(centroidX, centroidY);
//
//	int32 tv = bound(tvModifier * distance / 10, 20, 60);
//	int32 rv = 0;
//
//	if (abs(bearing) > ROTATION_DEADZONE) {
//		rv = ROTATION_RV_CONST * bearing / 1.5;
//		rv = boundAbs(rv, 800);
//		behSetTvRv(beh, tv / 2, rv);
//	} else {
//		behSetTvRv(beh, tv, 0);
//	}
//
//	cprintf("%d, %d, %d, %d\n", bearing, distance, tv, rv);
//}
