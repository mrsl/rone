/*
 * scaleCoordinate.h
 *
 *  Created on: Aug 27, 2014
 *      Author: Zak K.
 */

#ifndef SCALECOORDINATE_H_
#define SCALECOORDINATE_H_

#define ROBOT_RANGE	4000	// Set distance between robots, temporary until range is figured out

struct {
	NbrData Xh;	// High bits of x coordinate
	NbrData Xl;	// Low bits of x coordinate
	NbrData Yh;	// High bits of y coordinate
	NbrData Yl;	// Low bits of y coordinate
	NbrData childCount;	// Number of children
} typedef scaleCoordinate;

/**
 * @brief Initializes a scale coordinate
 */
void createScaleCoordinate(scaleCoordinate *toCreate);

/**
 * @brief Updates the neighbor data of a scale coordinate
 */
void updateScaleCoordinate(scaleCoordinate *toUpdate, int16 newX, int16 newY, uint8 childCount);

/**
 * @brief Returns the current X value of the scale coordinate for a neighbor
 */
int16 getScaleCoordinateX(scaleCoordinate *toGet, Nbr *nbrPtr);

/**
 * @brief Returns the current Y value of the scale coordinate for a neighbor
 */
int16 getScaleCoordinateY(scaleCoordinate *toGet, Nbr *nbrPtr);

/**
 * @brief Sets given X and Y pointers to the value of the scale coordinate for a neighbor
 */
void getScaleCoordinate(scaleCoordinate *toGet, Nbr *nbrPtr, int16 *x, int16 *y);

/**
 * @brief Rotates an X and Y coordinate by the angle
 */
void rotateXY(int16 *x, int16 *y, int32 angle);

/**
 * @brief Transforms a scale coordinate of a neighbor into local reference frame
 */
void transformScaleCoordinate(scaleCoordinate *toTransform, Nbr *nbrPtr, int16 *x, int16 *y, uint8 *childCount);

void applyTransformationMatrix(int16 *x, int16 *y,
							   int16 xCoor, int16 yCoor,
							   int32 orientation, int32 bearing,
							   int16 distance,
							   uint8 childCount);

#endif /* SCALECOORDINATE_H_ */
