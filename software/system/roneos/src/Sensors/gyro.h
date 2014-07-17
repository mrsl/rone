/**
 * @file gyro.h
 *
 * @brief Interface to initialize and read 3D gyro data.
 * @since July 22, 2010
 * @author James McLurkin
 */

#ifndef GYRO_H_
#define GYRO_H_

/******** Defines ********/

#define GYRO_X_AXIS		0
#define GYRO_Y_AXIS		1
#define GYRO_Z_AXIS		2

/******** Functions ********/

/**
 * @brief Initialize gyro.
 *
 * (gyroX, gyroY, gyroZ) initialized to 0.
 * @returns void
 */
void gyro_init(void);


/**
 * @brief Get gyro value on axis.
 *
 * @param axis specifies GYRO_X_AXIS, GYRO_Y_AXIS, or GYRO_Z_AXIS.
 * @returns if argument is valid axis, gyro value on specified axis; else, 0
 */
int32 gyroGetValue(uint32 axis);


/**
 * @brief Update gyro values.
 *
 * gyroX, gyroY, and gyroZ are updated by values in msgIn that are concatenated in pairs according to axis.
 * Each pair of 9-bit integers converted into one 16-bit integer.
 * @param msgIn is array of length 6
 * @returns void
 */
void gyroUpdate(uint8 msgIn[]);


#endif /* GYRO_H_ */
