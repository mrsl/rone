/**
 * @file light_sensor.h
 *
 * @brief Interface to initialize and read light sensor ring on robot.
 *
 * @since July 22, 2010
 * @author James McLurkin
 */

#ifndef LIGHT_SENSOR_H_
#define LIGHT_SENSOR_H_

/******** Defines ********/

#define LIGHT_SENSOR_FRONT_RIGHT            0
#define LIGHT_SENSOR_FRONT_LEFT             1

// for v11 robots
#define LIGHT_SENSOR_REAR                   2

//for v12 robots
#define LIGHT_SENSOR_REAR_RIGHT				2
#define LIGHT_SENSOR_REAR_LEFT				3


/******** Functions ********/

/**
 * @brief Initializes light sensor.
 *
 * Sets sampling speed and configures and enables the light sensor.
 * @returns void
 */
void lightSensorInit(void);


/**
 *	@brief Gets the ADC value of the specified light sensor.
 *
 *	If the input light sensor is not recognized, returns 0
 *	@param light_sensor specifies which light sensor you want to check.
 *	Only {0, 1, 2} are allowed on v11 robots and {0, 1, 2, 3} allowed on
 *	v12 robots
 *	@returns the ADC value; 0 if the input parameter is not recognized.  Note that
 *	for calibrated robots, this value can be negative, i.e. less that the
 *	calibration conditions.
 */
int32 lightSensorGetValue(uint32 light_sensor); // python rone


#endif /* LIGHT_SENSOR_H_ */
