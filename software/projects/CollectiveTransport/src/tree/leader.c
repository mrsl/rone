/*
 * leader.c
 *
 *  Created on: Jan 17, 2015
 *      Author: Zak
 */

#include "leader.h"
#include "guide.h"

NbrData isLeader;
NbrData active;
centroidNbrData guidePosition;
NbrData movementMode;
NbrData forward;

boolean manualDisable = FALSE;

int32 objectOrientation = 0;
int32 leaderOrientation = 0;
int32 angle2Rotate = 0;

int32 guideObjectBearing = 0;

centroidValue leaderGetGuideXCoordinate() {
	centroidValue centroidX, centroidY;
	centroidNbrDataGet(&centroidX, &centroidY, &guidePosition);

	return centroidX;
}

centroidValue leaderGetGuideYCoordinate() {
	centroidValue centroidX, centroidY;
	centroidNbrDataGet(&centroidX, &centroidY, &guidePosition);

	return centroidY;
}

boolean leaderIsLeader() {
	return nbrDataGet(&isLeader);
}

void leaderSetActive(boolean val) {
	nbrDataSet(&active, val);
}

boolean leaderNbrIsLeader(Nbr *nbrPtr) {
	return nbrDataGetNbr(&isLeader, nbrPtr);
}

Nbr *leaderGetLeader(NbrList *nbrListPtr) {
	uint8 i;

	// Initial pass to find leader
	for (i = 0; i < nbrListPtr->size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(nbrListPtr, i);
		if (leaderNbrIsLeader(nbrPtr)) {
			return nbrPtr;
		}
	}

	return NULL;
}

boolean leaderIsActive() {
	return nbrDataGet(&active);
}

boolean leaderNbrIsActive(Nbr *nbrPtr) {
	return nbrDataGetNbr(&active, nbrPtr);
}

void leaderSetLeader(boolean val) {
	nbrDataSet(&isLeader, val);
}

uint8 leaderGetMovementMode() {
	if (leaderIsActive()) {
		return nbrDataGet(&movementMode);
	} else {
		return MOVEMODE_STOP;
	}
}

extern centroidData centroidLite;

void leaderSetObjectOrientation() {
	centroidValue centroidX, centroidY;
	centroidDataGet(&centroidX, &centroidY, &centroidLite);

	int32 iVectorX = (int32) -centroidX;
	int32 iVectorY = (int32) -centroidY;

	int32 tObjectOrientation = normalizeAngleMilliRad2(atan2MilliRad(iVectorY, iVectorX));

	objectOrientation = filterIIRAngle(objectOrientation, tObjectOrientation, 30);
}

void leaderSetGuideObjectBearing() {
	centroidValue centroidX, centroidY;
	centroidDataGet(&centroidX, &centroidY, &centroidLite);

	centroidValue guideX, guideY;
	centroidNbrDataGet(&guideX, &guideY, &guidePosition);

	centroidValue vectorX = guideX - centroidX;
	centroidValue vectorY = guideY - centroidY;

	int32 iVectorX = (int32) vectorX;
	int32 iVectorY = (int32) vectorY;

	int32 tguideObjectBearing = normalizeAngleMilliRad2(atan2MilliRad(iVectorY, iVectorX));

	guideObjectBearing = filterIIRAngle(guideObjectBearing, tguideObjectBearing, 30);
}


int32 leaderGetGuideObjectBearing() {
	return guideObjectBearing;
}

int32 leaderGetObjectOrientation() {
	return objectOrientation;
}

// Need to search for guide robot
void leaderCallback(NbrDatabase* ndPtr) {
	NbrList nbrList;
	nbrListCreate(&nbrList);

	uint8 i;

	// Initial pass to find leader
	for (i = 0; i < nbrList.size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(&nbrList, i);
		if (leaderNbrIsLeader(nbrPtr)) {
			// They are less so they are leader
			if (nbrGetID(nbrPtr) < roneID) {
				nbrDataSet(&isLeader, FALSE);
			}
		}
	}
	// Find guide robot and get coordinates
	if (leaderIsLeader()) {
		leaderSetObjectOrientation();

		Nbr *guide = guideGetNextGuide(&nbrList);

		// Found a guide
		if (guide != NULL) {

			centroidValue guideX = 0.;
			centroidValue guideY = 0.;

			centroidTransform(&guideX, &guideY, guide);

			centroidNbrDataSet(guideX, guideY, &guidePosition);

			uint8 type = guideGetNbrType(guide);

			switch (type) {
			case (GUIDE_G): {
				nbrDataSet(&movementMode, MOVEMODE_STOP);
				break;
			}
			case (GUIDE_SM): {
				int32 guideOrientation = normalizeAngleMilliRad2(MILLIRAD_PI + nbrGetBearing(guide) - nbrGetOrientation(guide));
				int32 tAngle2Rotate = normalizeAngleMilliRad2(guideOrientation - objectOrientation);

				angle2Rotate = filterIIRAngle(angle2Rotate, tAngle2Rotate, 30);

				if (abs(angle2Rotate) < 350) {
					nbrDataSet(&movementMode, MOVEMODE_TRNS);
				} else {
					if (angle2Rotate < 0) {
						nbrDataSet(&movementMode, MOVEMODE_ROTR);
					} else {
						nbrDataSet(&movementMode, MOVEMODE_ROTL);
					}
				}
				break;
			}
			case (GUIDE_S): {
				nbrDataSet(&movementMode, MOVEMODE_TRNS);
				break;
			}
			}
		} else {
			nbrDataSet(&movementMode, MOVEMODE_STOP);
		}

	} else {
		Nbr *leader = leaderGetLeader(&nbrList);

		int32 tLeaderOrientation = normalizeAngleMilliRad2(MILLIRAD_PI + nbrGetBearing(leader) - nbrGetOrientation(leader));

		leaderOrientation = filterIIRAngle(leaderOrientation, tLeaderOrientation, 30);

		if (leader != NULL) {
			// Get the movement mode
			nbrDataSet(&movementMode, nbrDataGetNbr(&movementMode, leader));

			nbrDataSet(&active, nbrDataGetNbr(&active, leader));

			// Get the guide's position
			centroidValue x, y;
			centroidNbrDataGetNbr(&x, &y, &guidePosition, leader);
			centroidTransform(&x, &y, leader);
			centroidNbrDataSet(x, y, &guidePosition);
		} else {
			nbrDataSet(&movementMode, MOVEMODE_STOP);
		}
	}

	leaderSetGuideObjectBearing();
}

void leaderInit() {
	nbrDataCreate(&isLeader, "isLeader", 1, 0);
	nbrDataCreate(&active, "active", 1, 0);
	nbrDataCreate(&forward, "forward", 1, 0);
	nbrDataCreate(&movementMode, "moveMode", 2, 0);
	centroidNbrDataCreate(&guidePosition);
	centroidNbrDataSet(0, 0, &guidePosition);

	neighborsAddReceiveCallback(leaderCallback);
}
