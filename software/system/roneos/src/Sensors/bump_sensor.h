/**
 * @file bump_sensor.h
 *
 * @brief Reads bump sensor information and includes helper functions for processing this data.
 *
 * @since July 22, 2010
 * @author James McLurkin
 */

#ifndef BUMP_SENSOR_H_
#define BUMP_SENSOR_H_

/******** Defines ********/

/*	default numbering scheme starts at front-right, increases CCW  */
#define BUMP_FL_BIT				0x01
#define BUMP_LF_BIT				0x02
#define BUMP_LB_BIT				0x04
#define BUMP_BL_BIT				0x08
#define BUMP_BR_BIT				0x10
#define BUMP_RB_BIT				0x20
#define BUMP_RF_BIT				0x40
#define BUMP_FR_BIT 			0x80

/******** Functions ********/

/*
 * @brief Update bump sensors from the MSP communications message.
 *
 * bumpSensorBearing set to -1.
 * @param new_values the bump sensor value
 * @returns void
 */
void bumpSensorsUpdate(uint8 new_values);


/**
 * @brief Get bump sensor bits.
 *
 * @returns the bump sensor bits
 */
uint8 bumpSensorsGetBits();


/**
 * @brief Get bump sensor bearing.
 *
 * @returns the bump sensor bearing
 */
int16 bumpSensorsGetBearing();


/**
 * @brief Returns an integer identifying from which direction the robot was hit.
 *
 * 0 - Hit from front
 * 1 - Hit from left
 * 2 - Hit from right
 * 3 - Hit from behind
 *
 * If the robot is not being hit, returns -1.
 *
 * @returns An integer corresponding to the robot bump sensor that is active.
 */
uint8 bumpSensorsWhichHit();

//TODO: implement or delete?
void bumper_debug_print();

#endif /* BUMP_SENSOR_H_ */
