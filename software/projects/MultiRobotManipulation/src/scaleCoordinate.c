/*
 * scaleCoordinate.c
 *
 * @brief Functions and structures to manipulate local and remote coordinates
 *
 *  Created on: Aug 27, 2014
 *      Author: Zak
 */

#include "roneos.h"
#include "globalTreeCOM.h"

/**
 * @brief Initializes a scale coordinate
 */
void createScaleCoordinate(scaleCoordinate *toCreate) {
	nbrDataCreate16(&toCreate->Xh, &toCreate->Xl, "scXhigh", "scXlow", 0);
	nbrDataCreate16(&toCreate->Yh, &toCreate->Yl, "scYhigh", "scYlow", 0);
	nbrDataCreate(&toCreate->childCount, "childC", 8, 0);
}

/**
 * @brief Updates the neighbor data of a scale coordinate
 */
void updateScaleCoordinate(scaleCoordinate *toUpdate, int16 newX, int16 newY, uint8 childCount) {
	nbrDataSet16(&toUpdate->Xh, &toUpdate->Xl, newX);
	nbrDataSet16(&toUpdate->Yh, &toUpdate->Yl, newY);
	nbrDataSet(&toUpdate->childCount, childCount);
}

/**
 * @brief Returns the current X value of the scale coordinate for a neighbor
 */
int16 getScaleCoordinateX(scaleCoordinate *toGet, Nbr *nbrPtr) {
	 return nbrDataGetNbr16(&toGet->Xh, &toGet->Xl, nbrPtr);
}

/**
 * @brief Returns the current Y value of the scale coordinate for a neighbor
 */
int16 getScaleCoordinateY(scaleCoordinate *toGet, Nbr *nbrPtr) {
	 return nbrDataGetNbr16(&toGet->Yh, &toGet->Yl, nbrPtr);
}

/**
 * @brief Returns the current Y value of the scale coordinate for a neighbor
 */
uint8 getScaleCoordinateChildCount(scaleCoordinate *toGet, Nbr *nbrPtr) {
	 return nbrDataGetNbr(&toGet->childCount, nbrPtr);
}


/**
 * @brief Sets given X and Y pointers to the value of the scale coordinate for a neighbor
 */
void getScaleCoordinate(scaleCoordinate *toGet, Nbr *nbrPtr, int16 *x, int16 *y) {
	*x = getScaleCoordinateX(toGet, nbrPtr);
	*y = getScaleCoordinateY(toGet, nbrPtr);
}

/**
 * @brief Rotates an X and Y coordinate by the angle
 */
void rotateXY(int16 *x, int16 *y, int32 angle) {
	int16 tempX, tempY;
	tempX = *x * cosMilliRad(angle) / MILLIRAD_TRIG_SCALER
			- *y * sinMilliRad(angle) / MILLIRAD_TRIG_SCALER;
	tempY = *x * sinMilliRad(angle) / MILLIRAD_TRIG_SCALER
			+ *y * cosMilliRad(angle) / MILLIRAD_TRIG_SCALER;

	*x = tempX;
	*y = tempY;
}

void rotateXY32(int32 *x, int32 *y, int32 angle) {
	int32 tempX, tempY;
	tempX = *x * cosMilliRad(angle) / MILLIRAD_TRIG_SCALER
			- *y * sinMilliRad(angle) / MILLIRAD_TRIG_SCALER;
	tempY = *x * sinMilliRad(angle) / MILLIRAD_TRIG_SCALER
			+ *y * cosMilliRad(angle) / MILLIRAD_TRIG_SCALER;

	*x = tempX;
	*y = tempY;
}

/**
 * @brief Transforms a scale coordinate of a neighbor into local reference frame, and sets given X and Y
 */
void scShiftNbrReferenceFrame(scaleCoordinate *toRotate, Nbr *nbrPtr, int16 *x, int16 *y) {
	int32 orientation = nbrGetOrientation(nbrPtr);
	int32 bearing = nbrGetBearing(nbrPtr);

	getScaleCoordinate(toRotate, nbrPtr, x, y);

	rotateXY(x, y, orientation);

	*x += (int16) externalPoseGetNbrRange(nbrPtr) * 10;

	rotateXY(x, y, bearing);
}

/**
 * @brief Transforms a scale coordinate to local reference frame from a tree
 *
 * Keeps in mind number of children as to properly average the coordinates.
 *
 * @param childCount Number of (grand)children in tree sans yourself
 */
void transformScaleCoordinate(scaleCoordinate *toTransform, Nbr *nbrPtr, int16 *x, int16 *y, uint8 *childCount) {
	// Get child count
	*childCount = getScaleCoordinateChildCount(toTransform, nbrPtr);

	// Get angular position of neighbor
	// uint8 nbrId = nbrGetID(nbrPtr);

	//int32 orientation = lookupGetOrientation(roneID, nbrId);
	//int32 bearing = lookupGetBearing(roneID, nbrId);

	NbrPose nbrPose;
	if (!externalPoseGetRelativePose(nbrPtr, &nbrPose)) {
		return;
	}

	int32 orientation = nbrGetOrientation(nbrPtr);
	int32 bearing = nbrGetBearing(nbrPtr);
	int16 distance = (int16) externalPoseGetNbrRange(nbrPtr) * 10;
//
//	int16 distance = nbrPose.distance * 10;
//	int32 bearing = (int32) nbrPose.bearing;
//	int32 orientation = (int32) nbrPose.orientation;

	//cprintf("ID:%d (%d,%d)(%d,%d,%d)(%d,%d)\n", nbrId, nbrPose.theta, nbrPose.theta2, distance, bearing, orientation, nbrGetBearing(nbrPtr), nbrGetOrientation(nbrPtr));


	// Get centroid guess from neighbor
	int16 xCoor, yCoor;
	getScaleCoordinate(toTransform, nbrPtr, &xCoor, &yCoor);

	// Get X and Y coordinate of neighbor

	//cprintf("TSC:%d,%d,%d,%d,%d,%d,%d,%u\n", xCoor, yCoor, xNbr, yNbr, bearing, orientation, transformationAngle, *childCount);

	//cprintf("ID:%u O:%d B:%d D:%d X:%d Y:%d C:%u\n", nbrId, orientation, bearing, distance, xCoor, yCoor, *childCount);
	applyTransformationMatrix(x, y, xCoor, yCoor, orientation, bearing, distance, *childCount);
}

void applyTransformationMatrix(int16 *x, int16 *y,
							   int16 xCoor, int16 yCoor,
							   int32 orientation, int32 bearing,
							   int16 distance,
							   uint8 childCount) {

	int16 xNbr, yNbr;
	xNbr = distance * cosMilliRad(bearing) / MILLIRAD_TRIG_SCALER;
	yNbr = distance * sinMilliRad(bearing) / MILLIRAD_TRIG_SCALER;

	int32 transformationAngle = -normalizeAngleMilliRad2(MILLIRAD_PI - orientation + bearing);
	int16 xTemp, yTemp;
	// Calculate X and Y of centroid guess from neighbor in local reference frame, considering children
	xTemp = (xCoor * cosMilliRad(transformationAngle) - yCoor * sinMilliRad(transformationAngle))/ MILLIRAD_TRIG_SCALER
			+ xNbr * (int16)childCount;
	yTemp = (xCoor * sinMilliRad(transformationAngle) + yCoor * cosMilliRad(transformationAngle)) / MILLIRAD_TRIG_SCALER
			+ yNbr * (int16)childCount;

	*x = xTemp;
	*y = yTemp;

	//cprintf(" %7d | %7d | %7d | %7d | %7d | %7d | %7d | %7d | %7u \n", *x, *y, xCoor, yCoor, xNbr, yNbr, orientation, bearing, childCount);
}




























