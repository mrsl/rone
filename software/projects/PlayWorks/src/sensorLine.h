/*
 * sensorLine.h
 *
 *  Created on: Jul 16, 2014
 *      Author: wgx1
 */

#ifndef SENSORLINE_H_
#define SENSORLINE_H_

// Use with reflectiveFindTurnDirection() to set proper RV
#define REFLECTIVE_TURING_SPEED					50

/*
 * @brief Find the number of active sensors detecting the line. If the number
 * if greater than 0, the direction to turn is stored in the location of
 * parameter direction, with positive number proportional to left turn and
 * negative right. Error occurs if there's a disconnect between active
 * sensors (think two lines). Direction is not calculated in this situation.
 *
 * @param direction location to store the direction.
 * @return number of reflective sensors reading the line; If there's a
 * disconnect between active sensors, return -1.
 */
int8 reflectiveGetNumActiveSensors(int8 *direction);


/*
 * @brief Find the direction and magnitude to turn and center the robot on line.
 * This function should be only called if totalActive is > 0.
 *
 * @param first index of the first sensor from the left to detect the line.
 * @param last index of the last sensor from the left to detect the line.
 * @return positive number of left turn; negative number for right turn; 0 for no
 * turn.
 */
int8 reflectiveFindTurnDirection(uint8 first, uint8 last);


#endif /* SENSORLINE_H_ */
