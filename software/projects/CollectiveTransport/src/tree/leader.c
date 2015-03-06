/*
 * leader.c
 *
 *  Created on: Jan 17, 2015
 *      Author: Zak
 */

#include "../collectivetransport.h"

NbrData isLeader;
NbrData active;
centroidNbrData guidePosition;
NbrData movementMode;

uint8 deactiveTimeout = 0;
NbrData deactivate;

boolean manualDisable = FALSE;

int32 objectOrientation = 0;
int32 leaderOrientation = 0;
int32 angle2Rotate = 0;

int32 guideDist = 1000000;
int32 centroidDist = 1000000;
int32 centroid2GuideDist = 1000000;

boolean tvLimiter = 0;

uint8 prevGuide = 0;

int32 guideObjectBearing = 0;

extern state currentState;
extern int32 avoidBearing;
extern uint8 avoidActive;

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

boolean leaderCanTV() {
	return tvLimiter;
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
	centroidDist = vectorMag(centroidX, centroidY);

	centroidValue guideX, guideY;
	centroidNbrDataGet(&guideX, &guideY, &guidePosition);

	centroidValue vectorX = guideX - centroidX;
	centroidValue vectorY = guideY - centroidY;

	int32 iVectorX = (int32) vectorX;
	int32 iVectorY = (int32) vectorY;

	int32 tguideObjectBearing = normalizeAngleMilliRad2(atan2MilliRad(iVectorY, iVectorX));
	centroid2GuideDist = filterIIR(vectorMag(iVectorX, iVectorY), centroid2GuideDist, 20);

	guideObjectBearing = filterIIRAngle(guideObjectBearing, tguideObjectBearing, 30);
}


int32 leaderGetGuideObjectBearing() {
	return guideObjectBearing;
}

int32 leaderGetObjectOrientation() {
	return objectOrientation;
}

int32 leaderGetGuideDistance() {
	return guideDist;
}

void leaderDeactivateRobot(uint8 id) {
	nbrDataSet(&deactivate, id);
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

	tvLimiter = 1;
	if (centroid2GuideDist < 300) {
		tvLimiter = 0;
	}


	// Find guide robot and get coordinates
	if (leaderIsLeader()) {
		leaderSetObjectOrientation();

		Nbr *guide = guideGetNextGuide(&nbrList);

		// Found a guide
		if (guide != NULL) {

			uint8 tempGuide = nbrGetID(guide);

			if (tempGuide != prevGuide) {
				guideDist = 1000000;
				centroid2GuideDist = 1000000;
			}

			prevGuide = tempGuide;

			if (centroid2GuideDist < 300) {
				uint8 oldRobot = nbrDataGet(&deactivate);
				uint8 newRobot = nbrGetID(guide);

				if (oldRobot == newRobot) {
					deactiveTimeout++;
					if (deactiveTimeout > 3) {
						leaderDeactivateRobot(0);
					}
				} else {
					leaderDeactivateRobot(newRobot);
					deactiveTimeout = 0;
				}
			}

			centroidValue guideX = 0.;
			centroidValue guideY = 0.;

			centroidTransform(&guideX, &guideY, guide);

			centroidNbrDataSet(guideX, guideY, &guidePosition);

			guideDist = vectorMag(guideX, guideY);

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
			centroid2GuideDist = 1000000;
		}

	} else {
		Nbr *leader = leaderGetLeader(&nbrList);

		if (nbrDataGetNbr(&deactivate, leader) == roneID) {
			currentState = IDLE;
			guideSetGuide(FALSE);
		}

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

			if (currentState == IDLE) {
				avoidActive = 0;
				if (nbrGetRange(leader) < 400) {
					centroidValue nbrX = 0.;
					centroidValue nbrY = 0.;

					centroidTransform(&nbrX, &nbrY, leader);

					centroidValue avoidX = x - nbrX;
					centroidValue avoidY = y - nbrY;

					int32 guideBear = atan2MilliRad(nbrY, nbrX);

					int32 avoidBear = atan2MilliRad(avoidY, avoidX);
					int32 avoidBearP = normalizeAngleMilliRad2(avoidBear + MILLIRAD_HALF_PI);
					int32 avoidBearM = normalizeAngleMilliRad2(avoidBear - MILLIRAD_HALF_PI);

					if (abs(smallestAngleDifference(guideBear, avoidBearP)) > abs(smallestAngleDifference(guideBear, avoidBearM))) {
						avoidBear = avoidBearP;
					} else {
						avoidBear = avoidBearM;
					}

					avoidActive = 1;
					avoidBearing = avoidBear;
				}
			}


		} else {
			nbrDataSet(&movementMode, MOVEMODE_STOP);
		}
	}

	leaderSetGuideObjectBearing();
}

void leaderInit() {
	nbrDataCreate(&isLeader, "isLeader", 1, 0);
	nbrDataCreate(&active, "active", 1, 0);
	nbrDataCreate(&movementMode, "moveMode", 2, 0);
	nbrDataCreate(&deactivate, "deactivate", 8, 0);
	centroidNbrDataCreate(&guidePosition);
	centroidNbrDataSet(0, 0, &guidePosition);

	neighborsAddReceiveCallback(leaderCallback);
}
