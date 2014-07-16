/*
 * sensorMag.h
 *
 *  Created on: Mar 24, 2014
 *      Author: jamesm
 */

#ifndef SENSORMAG_H_
#define SENSORMAG_H_

#define MAG_X_AXIS				GYRO_X_AXIS
#define MAG_Y_AXIS				GYRO_Y_AXIS
#define MAG_Z_AXIS				GYRO_Z_AXIS
#define MAG_AXES				3

void magInit(void);
void magSetOffset(void);
int32 magGetValueLeft(uint8 axis);
int32 magGetValueRight(uint8 axis);
int32 magGetMagnitudeLeft(void);
int32 magGetMagnitudeRight(void);


#endif /* SENSORMAG_H_ */
