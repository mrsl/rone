/*
 * controllers.c
 *
 *  Created on: Sep 6, 2014
 *      Author: Zak
 */

#include "globalTreeCOM.h"

#define MRM_RV_GAIN				100
#define MRM_TV_GAIN				100
#define MRM_RV_FLOCK_GAIN		15
#define ROTATION_DEADZONE		200
#define MRM_ROTATE_LEFT_BIAS	300


#define MRM_MAX_CENTROID_DIST	150

#define MRM_MAX_CYCLOID_DIST	(MRM_MAX_CENTROID_DIST * 2)

#define MRM_ALPHA1				4
#define MRM_ALPHA2				20

int32 rvGain = MRM_RV_GAIN;
int32 tvGain = MRM_TV_GAIN;

int32 alpha = MRM_ALPHA1;

void setRVGain(int32 newRVGain) {
	rvGain = newRVGain;
}

int32 getRVGain() {
	return rvGain;
}

void setTVGain(int32 newTVGain) {
	tvGain = newTVGain;
}

int32 getTVGain() {
	return tvGain;
}

void setBehFilter(int32 newAlpha) {
	alpha = newAlpha;
}

int32 getBehFilter() {
	return alpha;
}


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
	if (roneID == getPivotRobot()) {
		//behSetTvRv(behPtr, 0, -200);
		behSetTvRv(behPtr, 0, -0);
	} else {
		mrmPointOrbit(behPtr,
					 (int32) (navDataPtr->pivotX / 10),
					 (int32) (navDataPtr->pivotY / 10),
					 tvModifier);
	}
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
//	int32 bearingRight = normalizeAngleMilliRad(bearing + MILLIRAD_PI / 2) - MILLIRAD_PI;
//
//	if (abs(bearingLeft) < abs(bearingRight)) {
//		bearing = bearingLeft;
//	} else {
//		bearing = bearingRight;
//		tvModifier = -tvModifier;
//	}
	bearing = bearingLeft;

	// Proportional tv and rv control
	int32 distance = vectorMag(x, y) / 10; // In AprilTag units

//	int32 goalTv = boundAbs(tvModifier * distance, MRM_MAX_TV);
//	int32 goalRv = -smallestAngleDifference(0, bearing) * MRM_RV_GAIN / 100;

	int32 goalTv = (getTVGain() / 100) * boundAbs(tvModifier * distance, MRM_MAX_TV);
	int32 goalRv = 0;

	if (abs(bearing) > ROTATION_DEADZONE) {
		goalRv = (getRVGain() / 100) * bearing / 10;
		goalTv = goalTv * 100 / 150;
	}

	// Filter from previous state
	int32 finalTv = mrmIIR(goalTv, tv, getBehFilter());
	int32 finalRv = mrmIIR(goalRv, rv, getBehFilter());

	behSetTvRv(behPtr, finalTv, finalRv);
}

//void mrmTranslateLeaderToGuide(navigationData *navDataPtr, NbrList *nbrListPtr,
//									Beh *behPtr, int32 tvModifier) {
//
//	int32 guideX = navDataPtr->guideX;
//	int32 guideY = navDataPtr->guideY;
//
//	int32 bearing;
//
//	int32 goalTv, goalRv;
//
//	int32 tv = behGetTv(behPtr);
//	int32 rv = behGetRv(behPtr);
//
//	if (roneID == getPivotRobot()) {
//		// Get bearing towards guide
//		bearing = atan2MilliRad(guideY, guideX);
//	} else {
//		bearing = mrmFlockAngle(nbrListPtr);
//	}
//
//	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;
//
//	// Rotate and translate towards guide
//	goalTv = tvModifier;
//	goalRv = 0;
//
//	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;
//	if (abs(bearing) > ROTATION_DEADZONE) {
//		goalRv = smallestAngleDifference(0, bearing) * MRM_RV_FLOCK_GAIN / 100;
//		goalTv /= 1.5;
//	}
//
////	goalTv = tvModifier;
////	goalRv = 0;
////
////	if (abs(bearing) > ROTATION_DEADZONE) {
////		goalRv = MRM_RV_GAIN * bearing / 1.5;
////		goalTv /= 1.5;
////	}
//
//
//	// Filter from previous state
//	int32 finalTv = mrmIIR(goalTv, tv, MRM_ALPHA1);
//	int32 finalRv = mrmIIR(goalRv, rv, MRM_ALPHA1);
//
//	behSetTvRv(behPtr, finalTv, finalRv);
//}

void mrmTranslateLeaderToGuideVector(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {

	int32 guideX = navDataPtr->guideX;
	int32 guideY = navDataPtr->guideY;
	int32 centroidX = navDataPtr->centroidX;
	int32 centroidY = navDataPtr->centroidY;

	int32 tv = behGetTv(behPtr);
	int32 rv = behGetRv(behPtr);

	int32 x = (guideX - centroidX) / 10;
	int32 y = (guideY - centroidY) / 10;

	if (x == 0 && y == 0) {
		behSetTvRv(behPtr, 0, 0);
		return;
	}

	//cprintf("pt 3,%d,%d\n", x, y);

	// Get bearing towards point
	int32 bearing = atan2MilliRad(y, x);

	// Decided whether to drive forwards or backwards
	int32 bearingLeft = normalizeAngleMilliRad(bearing - MILLIRAD_PI) - MILLIRAD_PI;
	bearing = bearingLeft;

	// Proportional tv and rv control
	int32 distance = vectorMag(x, y) / 10; // In AprilTag units

	int32 goalTv = getTVGain() * boundAbs(tvModifier * distance, MRM_MAX_TV) / 100;
	int32 goalRv = 0;

	if (abs(bearing) > ROTATION_DEADZONE) {
		goalRv = getRVGain() * bearing / 10 / 100;
		goalTv = goalTv * 100 / 150;
	}

	// Filter from previous state
	int32 finalTv = mrmIIR(goalTv, tv, getBehFilter());
	int32 finalRv = mrmIIR(goalRv, rv, getBehFilter());

	behSetTvRv(behPtr, finalTv, finalRv);

//	bearing = atan2MilliRad(y, x);
//
//	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;
//
//	// Rotate and translate towards guide
//	goalTv = tvModifier;
//	goalRv = 0;
//
//	goalRv = smallestAngleDifference(0, bearing) * MRM_RV_FLOCK_GAIN / 100;
//	if (abs(bearing) > ROTATION_DEADZONE) {
//		goalTv /= 1.5;
//	}

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
//	behSetTvRv(behPtr, goalTv, goalRv);
}

//void mrmCycloidMotion(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
//	if (roneID == getPivotRobot()) {
//		mrmTranslateLeaderToGuideVector(navDataPtr, behPtr, tvModifier);
//	} else {
//		mrmOrbitPivot(navDataPtr, behPtr, tvModifier / 10);
//	}
//}


void mrmCycloidMotion(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
	int32 guideX = navDataPtr->guideX;
	int32 guideY = navDataPtr->guideY;
	int32 centroidX = navDataPtr->centroidX;
	int32 centroidY = navDataPtr->centroidY;

	if ((guideX == 0 && guideY == 0) || (centroidX == 0 && centroidY == 0)) {
		behSetTvRv(behPtr, 0, 0);
		return;
	}

	// Get vector towards guide
	int32 transX = (guideX - centroidX) / 10;
	int32 transY = (guideY - centroidY) / 10;

	// Get bearing towards guide
	int32 Tbearing = atan2MilliRad(transY, transX);

	// Normalize bearing to translational vector
	Tbearing = normalizeAngleMilliRad(Tbearing - MILLIRAD_PI) - MILLIRAD_PI;

	// Find distance to centroid for velocity scaling
	//int32 Tmagnitude = vectorMag(transX, transY);

	// Get bearing towards centroid
	int32 Rbearing = atan2MilliRad(centroidY, centroidX);

	// Normalize bearing perpendicular to the centroid
	Rbearing = normalizeAngleMilliRad(Rbearing - MILLIRAD_PI / 2) - MILLIRAD_PI;

	// Find distance to centroid for velocity scaling
	int32 Rmagnitude = vectorMag(centroidX, centroidY);

	int32 rX = Rmagnitude * cosMilliRad(Rbearing) / MILLIRAD_TRIG_SCALER;
	int32 rY = Rmagnitude * sinMilliRad(Rbearing) / MILLIRAD_TRIG_SCALER;

	int32 tX = Rmagnitude * cosMilliRad(Tbearing) / MILLIRAD_TRIG_SCALER;
	int32 tY = Rmagnitude * sinMilliRad(Tbearing) / MILLIRAD_TRIG_SCALER;

	int32 Vmagnitude = vectorMag(rX + tX, rY + tY);

	// Get bearing towards cycloid path
	// TODO: Scale Translation to Rotation
	 int32 Cbearing = averageAngles(Tbearing, Rbearing);

//	int32 Cbearing;
//	int32 bearingLeft = averageAngles(Tbearing, Rbearing);
//	int32 bearingRight = normalizeAngleMilliRad(bearingLeft + MILLIRAD_PI) - MILLIRAD_PI;
//
//	if (abs(bearingLeft) < abs(bearingRight)) {
//		Cbearing = bearingLeft;
//	} else {
//		Cbearing = bearingRight;
//		tvModifier = -tvModifier;
//	}

	//cprintf("pt 0,%d,%d\n", Vmagnitude * cosMilliRad(Cbearing), Vmagnitude * sinMilliRad(Cbearing));

	// Calculate velocities
	int32 tv = behGetTv(behPtr);
	int32 rv = behGetRv(behPtr);

	int32 goalTv = tvModifier * boundAbs(Vmagnitude * MRM_MAX_TV  / MRM_MAX_CYCLOID_DIST * getTVGain() / 100, MRM_MAX_TV);
	int32 goalRv = 0;

	if (abs(Cbearing) > ROTATION_DEADZONE) {
		goalRv = getRVGain() * Cbearing / 10 / 100;
		goalTv = goalTv * 100 / 200;
	}

	//cprintf("%d, %d, %d\n", Vmagnitude, goalTv, goalRv);

	// Filter from previous state
	int32 finalTv = mrmIIR(goalTv, tv, getBehFilter());
	int32 finalRv = mrmIIR(goalRv, rv, getBehFilter());

	rprintf("%d,%d,%d,%d\n", goalTv, goalRv, finalTv, finalRv);
	rprintfFlush();

	behSetTvRv(behPtr, finalTv, finalRv);
}

//void mrmCycloidMotion(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
//	int32 guideX = navDataPtr->guideX;
//	int32 guideY = navDataPtr->guideY;
//	int32 centroidX = navDataPtr->centroidX;
//	int32 centroidY = navDataPtr->centroidY;
//
//	guideX = 500;
//	guideY = 0;
//
//	centroidX = 200;
//	centroidY = 0;
//
//	if ((guideX == 0 && guideY == 0) || (centroidX == 0 && centroidY == 0)) {
//		behSetTvRv(behPtr, 0, 0);
//		return;
//	}
//
//	// Get vector towards guide
//	int32 Tx = (guideX - centroidX) / 10;
//	int32 Ty = (guideY - centroidY) / 10;
//
//	// Get bearing towards guide
//	int32 Tbearing = atan2MilliRad(Ty, Tx);
//
//	// Normalize bearing to translational vector
//	Tbearing = normalizeAngleMilliRad(Tbearing - MILLIRAD_PI) - MILLIRAD_PI;
//
//	// Get bearing towards centroid
//	int32 Rbearing = atan2MilliRad(centroidY, centroidX);
//
//	// Normalize bearing perpendicular to the centroid
//	Rbearing = normalizeAngleMilliRad(Rbearing - MILLIRAD_PI / 2) - MILLIRAD_PI;
//
//
//	// Get bearing towards cycloid path
//	// TODO: Scale Translation to Rotation
//	int32 Cbearing = averageAngles(Tbearing, Rbearing);
//
//	// Find distance to centroid for velocity scaling
//	int32 Vmagnitude = vectorMag(centroidX, centroidY);
//
//	//cprintf("pt 0,%d,%d\n", Vmagnitude * cosMilliRad(Cbearing), Vmagnitude * sinMilliRad(Cbearing));
//
//	// Calculate velocities
//	int32 tv = behGetTv(behPtr);
//	int32 rv = behGetRv(behPtr);
//
//	int32 goalTv = boundAbs(Vmagnitude * MRM_MAX_TV  / MRM_MAX_CENTROID_DIST * getTVGain() / 100, MRM_MAX_TV);
//	int32 goalRv = 0;
//
//	if (abs(Cbearing) > ROTATION_DEADZONE) {
//		goalRv = getRVGain() * Cbearing / 10 / 100;
//		goalTv = goalTv * 100 / 150;
//	}
//
//	cprintf("%d, %d, %d\n", Vmagnitude, goalTv, goalRv);
//
//	// Filter from previous state
//	int32 finalTv = mrmIIR(goalTv, tv, getBehFilter());
//	int32 finalRv = mrmIIR(goalRv, rv, getBehFilter());
//
//	behSetTvRv(behPtr, finalTv, finalRv);
//}



//void mrmCycloidMotion(navigationData *navDataPtr, Beh *behPtr, int32 tvModifier) {
//	int32 guideX = navDataPtr->guideX;
//	int32 guideY = navDataPtr->guideY;
//	int32 centroidX = navDataPtr->centroidX;
//	int32 centroidY = navDataPtr->centroidY;
//
//	// Find vector normal to the centroid vector (Clockwise)
//	int32 rotationX = centroidX;
//	int32 rotationY = centroidY;
//
//	rotateXY32(&rotationX, &rotationY, MILLIRAD_HALF_PI);
//
//	int32 distanceR = vectorMag(rotationX, rotationY) / 10 + 1; // In AprilTag units
//
//	// Find vector towards guide robot parallel to centroid-guide line (Look at translate)
//	int32 translationX = guideX - centroidX;
//	int32 translationY = guideY - centroidY;
//
//	int32 distanceT = vectorMag(translationX, translationY) / 10 + 1; // In AprilTag units
//
//	// Take the vector addition of these vectors
//
//	rotationX *= distanceT;
//	rotationY *= distanceT;
//	translationX *= distanceR;
//	translationY *= distanceR;
//
//	int32 x = rotationX + translationX;
//	int32 y = rotationY + translationY;
//
//	if (x == 0 && y == 0) {
//		behSetTvRv(behPtr, 0, 0);
//		return;
//	}
//
//	// Find bearing
//	int32 bearing = atan2MilliRad(y, x);
//	bearing = normalizeAngleMilliRad(bearing) - MILLIRAD_PI;
//
//	int32 distance = vectorMag(x, y) / 10; // In AprilTag units
//
//	// Rotate and translate towards guide
//	int32 goalTv = boundAbs(tvModifier * distance, MRM_MAX_TV);
//	int32 goalRv = 0;
//
//	goalRv = smallestAngleDifference(0, bearing) * MRM_RV_FLOCK_GAIN / 100;
//	if (abs(bearing) > ROTATION_DEADZONE) {
//		goalTv /= 1.5;
//	}
//
//	int32 tv = behGetTv(behPtr);
//	int32 rv = behGetRv(behPtr);
//
//	// Filter from previous state
//	int32 finalTv = mrmIIR(tv, goalTv, MRM_ALPHA2);
//	int32 finalRv = mrmIIR(rv, goalRv, MRM_ALPHA2);
//
//	behSetTvRv(behPtr, finalTv, finalRv);
//	//behSetTvRv(behPtr, goalTv, goalRv);
//}

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
