/*
 * sensorMag.c
 *
 *  Created on: Mar 24, 2014
 *      Author: jamesm
 */

#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"

#define MAG_IIR_TIME_CONSTANT	10

int32 magGetValueLeftRaw(uint8 axis) {
	return accelerometerGetValue(axis);
}

int32 magGetValueRightRaw(uint8 axis) {
	return gyroGetValue(axis);
}

static int32 magOffsetValuesLeft[MAG_AXES] = {0};
static int32 magOffsetValuesRight[MAG_AXES] = {0};
void magSetOffset(void) {
	uint8 i;
    for (i = 0; i < MAG_AXES; i++) {
    	magOffsetValuesLeft[i] = magGetValueLeftRaw(i);
    	magOffsetValuesRight[i] = magGetValueRightRaw(i);
	}
}

int32 magGetValueLeft(uint8 axis) {
	return magGetValueLeftRaw(axis) - magOffsetValuesLeft[axis];
}

int32 magGetValueRight(uint8 axis) {
	return magGetValueRightRaw(axis) - magOffsetValuesRight[axis];
}


void magInit(void) {
    // wait for a sec to get initial magnetometer values
    osTaskDelay(500);
    //magSetOffset();
}

#define MAG_LEFT_SCALER		120
#define MAG_RIGHT_SCALER	100

int32 magGetMagnitudeLeft(void) {
	int32 x, y, z;
	static int32 magMagLeft;

	x = magGetValueLeft(MAG_X_AXIS);
	y = magGetValueLeft(MAG_Y_AXIS);
	z = magGetValueLeft(MAG_Z_AXIS);

	int32 magMagNew = sqrtInt(x*x + y*y + z*z) * MAG_LEFT_SCALER / 100;
	magMagLeft = filterIIR(magMagLeft, magMagNew, MAG_IIR_TIME_CONSTANT);
	return magMagLeft;
}

int32 magGetMagnitudeRight(void) {
	int32 x, y, z;
	static int32 magMagRight;

	x = magGetValueRight(MAG_X_AXIS);
	y = magGetValueRight(MAG_Y_AXIS);
	z = magGetValueRight(MAG_Z_AXIS);

	int32 magMagNew = sqrtInt(x*x + y*y + z*z) * MAG_RIGHT_SCALER / 100;
	magMagNew = magMagNew / 16;
	magMagRight = filterIIR(magMagRight, magMagNew, MAG_IIR_TIME_CONSTANT);
	return magMagRight;
}
