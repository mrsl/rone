/**
 * @file accelerometer.h
 *
 * @brief Interface functions for 3D accelerometer in the robot.
 *
 * @since July 22, 2010
 * @author James McLurkin
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

/******** Defines ********/

#define ACCELEROMETER_X             0
#define ACCELEROMETER_Y             1
#define ACCELEROMETER_Z             2

void accelerometer_init(void);

/******** Functions ********/

/**
 * @brief Gets the accelerometer value.
 * @param axis uint32 of the axis to get the value of.
 * @return int16 value of accelerometer.
 */
int16 accelerometerGetValue(uint32 axis); // external



#if defined(RONE_V6)
/**
 * @brief Updates the x, y, and z axes of the accelerometer.
 *
 * @returns void
 */
void accelerometerUpdate(void);
#endif

#if defined(RONE_V9)
/**
 * @brief Updates the x, y, and z axes of the accelerometer.
 *
 * @returns void
 */
void accelerometerUpdate(uint8 msgIn[]);


/**
 * @brief Get accelerometer value on axis.
 *
 * @param axis specifies ACCELOMETER_X, ACCELEROMETER_Y, or ACCELEROMETER_Z.
 * @returns if argument is valid axis, accelerometer value on specified axis; else, 0
 */
int16 accelerometerGetValue(uint32 axis);
#endif


#if defined(RONE_V12)
/**
 * @brief Updates the accelerometer.
 * @param msgIn Unsigned char to put encoder value.
 * @return void.
 */
void accelerometerUpdate(uint8 msgIn[]);


/**
 * @brief Get accelerometer value on axis.
 *
 * @param axis specifies ACCELOMETER_X, ACCELEROMETER_Y, or ACCELEROMETER_Z.
 * @returns if argument is valid axis, accelerometer value on specified axis; else, 0
 */
int16 accelerometerGetValue(uint32 axis);
#endif

#endif /* ACCELEROMETER_H_ */
