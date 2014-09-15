/*
 * controllers.c
 *
 *  Created on: Sep 6, 2014
 *      Author: Zak
 */

#include "globalTreeCOM.h"

#define ROTATION_RV_CONST	1.2
#define ROTATION_DEADZONE	200

#define MRM_ALPHA			50

void mrmRotateCentroid(navigationData *navData, Beh *beh, int32 tvModifier) {
	mrmPointRotate(beh,
				  (int32) (navData->centroidX / 10),
				  (int32) (navData->centroidY / 10),
				  tvModifier);
}

void mrmRotatePivot(navigationData *navData, Beh *beh, int32 tvModifier) {
	mrmPointRotate(beh,
				  (int32) (navData->pivotX / 10),
				  (int32) (navData->pivotY / 10),
				  tvModifier);
}


void mrmPointRotate(Beh *beh, int32 x, int32 y, int32 tvModifier) {
	int32 tv = behGetTv(beh);
	int32 rv = behGetRv(beh);

	if (x == 0 && y == 0) {
		behSetTvRv(beh, 0, 0);
		return;
	}

	int32 bearing = atan2MilliRad(y, x);
	int32 bearingLeft = normalizeAngleMilliRad(bearing - MILLIRAD_PI / 2) - MILLIRAD_PI;
	int32 bearingRight = normalizeAngleMilliRad(bearing + MILLIRAD_PI / 2) - MILLIRAD_PI;

	if (abs(bearingLeft) < abs(bearingRight)) {
		bearing = bearingLeft;
	} else {
		bearing = bearingRight;
		tvModifier = -tvModifier;
	}

	int32 distance = vectorMag(x, y);

	int32 goalTv = boundAbs(tvModifier * distance / 10, 60);
	int32 goalRv = 0;

	if (abs(bearing) > ROTATION_DEADZONE) {
		goalRv = ROTATION_RV_CONST * bearing / 1.5;
		goalTv /= 1.5;
	}

	int32 finalTv = filterIIR(goalTv, tv, MRM_ALPHA);
	int32 finalRv = filterIIR(goalRv, rv, MRM_ALPHA);

	behSetTvRv(beh, finalTv, finalRv);
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
