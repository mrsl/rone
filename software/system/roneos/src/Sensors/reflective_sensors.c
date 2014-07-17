/*
 * @file reflective_sensors.c
 * @brief interface functions for reflective sensors in tiletrack robots
 * @since July 15, 2014
 * @author William Xie
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "roneos.h"

uint8 reflectiveValues[NUM_REFLECTIVE_PORTS];


/*
 * @brief Updates reflective values received from tiletrack bottomboard.
 *
 * @returns void
 */
void reflectiveSensorsUpdate(uint8 msgIn[]) {
	int i;

	for (i = 0; i < NUM_REFLECTIVE_PORTS; i++) {
		reflectiveValues[i] = msgIn[i];
	}
}


/*
 * @brief Return the raw light sensor values normalized to uint8 range.
 * The higher the value, the less light received.
 *
 * @param index the nth reflective sensor
 * @return raw light sensor value
 */
uint8 reflectiveSensorsGetValue(uint8 index) {
	return reflectiveValues[index];
}

