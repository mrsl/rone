/*
 * sensorLine.c
 *
 *  Created on: Jul 16, 2014
 *      Author: wgx1
 */

#include <string.h>
#include <stdio.h>
#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"

// [0, 255]
#define LINE_DETECTION_THRESHOLD		150

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
int8 reflectiveGetNumActiveSensors(int8 *direction) {
	int8 total = 0;
	volatile uint8 i, first, last;

	for (i = 0; i < NUM_REFLECTIVE_PORTS; i++) {
		if (reflectiveSensorsGetValue(i) > LINE_DETECTION_THRESHOLD) {
			// Save the first sensor from the left to detect the line
			if (total == 0) {
				first = i;
			} else if (i != last + 1){
				// Check if there's a disconnect
				total = -1;
				break;
			}
			total++;
			last = i;
		}
	}

	// If the robot detects a line, find out how much it should turn
	if (total > 0) {
		*(direction) = reflectiveFindTurnDirection(first, last);
	}

	return total;
}


/*
 * @brief Find the direction and magnitude to turn and center the robot on line.
 * This function should be only called if totalActive is > 0.
 *
 * @param first index of the first sensor from the left to detect the line.
 * @param last index of the last sensor from the left to detect the line.
 * @return positive number of left turn; negative number for right turn; 0 for no
 * turn.
 */
int8 reflectiveFindTurnDirection(uint8 first, uint8 last) {
	int8 fromFirst, fromLast;

	// Find the distance from either extreme
	fromFirst = first - 0;
	fromLast = NUM_REFLECTIVE_PORTS - 1 - last;

	// Use the difference to find the
	return fromLast - fromFirst;
}

