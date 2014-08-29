/*
 * scaleCoordinate.c
 *
 * @brief Functions and structures to manipulate local and remote coordinates
 *
 *  Created on: Aug 27, 2014
 *      Author: Zak K.
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
void rotateXYOld(int16 *x, int16 *y, int32 angle) {
	int16 tempX, tempY;
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
void transformScaleCoordinateOld(scaleCoordinate *toRotate, Nbr *nbrPtr, int16 *x, int16 *y) {
	int32 orientation = nbrGetOrientation(nbrPtr);
	int32 bearing = nbrGetBearing(nbrPtr);

	getScaleCoordinate(toRotate, nbrPtr, x, y);

	rotateXY(x, y, orientation);

	*x += ROBOT_RANGE;

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
	int32 orientation = nbrGetOrientation(nbrPtr);
	int32 bearing = nbrGetBearing(nbrPtr);

	// Get centroid guess from neighbor
	int16 xCoor, yCoor;
	getScaleCoordinate(toTransform, nbrPtr, &xCoor, &yCoor);

	// Get X and Y coordinate of neighbor

	//cprintf("TSC:%d,%d,%d,%d,%d,%d,%d,%u\n", xCoor, yCoor, xNbr, yNbr, bearing, orientation, transformationAngle, *childCount);

	applyTransformationMatrix(x, y, xCoor, yCoor, orientation, bearing, *childCount);
}

void applyTransformationMatrix(int16 *x, int16 *y,
							   int16 xCoor, int16 yCoor,
							   int32 orientation, int32 bearing,
							   uint8 childCount) {

	int16 xNbr, yNbr;
	xNbr = ROBOT_RANGE * cosMilliRad(bearing) / MILLIRAD_TRIG_SCALER;
	yNbr = ROBOT_RANGE * sinMilliRad(bearing) / MILLIRAD_TRIG_SCALER;

	int32 transformationAngle = -normalizeAngleMilliRad2(PI - orientation + bearing);
	int16 xTemp, yTemp;
	// Calculate X and Y of centroid guess from neighbor in local reference frame, considering children
	xTemp = xCoor * (cosMilliRad(transformationAngle) / MILLIRAD_TRIG_SCALER)
			- yCoor * (sinMilliRad(transformationAngle) / MILLIRAD_TRIG_SCALER)
			+ xNbr * (int16)childCount;
	yTemp = xCoor * (sinMilliRad(transformationAngle) / MILLIRAD_TRIG_SCALER)
			+ yCoor * (cosMilliRad(transformationAngle) / MILLIRAD_TRIG_SCALER)
			+ yNbr * (int16)childCount;

	*x = xTemp;
	*y = yTemp;

	//cprintf(" %7d | %7d | %7d | %7d | %7d | %7d | %7d | %7d | %7u \n", *x, *y, xCoor, yCoor, xNbr, yNbr, orientation, bearing, childCount);
}




























