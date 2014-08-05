/*
 * @file reflective_sensors.h
 * @brief interface functions for reflective sensors in tiletrack robots
 * @since July 15, 2014
 * @author William Xie
 */
#ifndef REFLECTIVE_SENSORS_H_
#define REFLECTIVE_SENSORS_H_

#define NUM_REFLECTIVE_PORTS				5

// 8 degrees separation between reflective sensors (140 mrad)
#define REFLECTIVE_SENSOR_SEPARATION		140
/**
 * @brief Updates reflective values received from tiletrack bottomboard.
 *
 * @returns void
 */
void reflectiveSensorsUpdate(uint8 msgIn[]);


/**
 * @brief Return the raw light sensor values normalized to uint8 range.
 * The higher the value, the less light received.
 *
 * @param index the nth reflective sensor
 * @return raw light sensor value
 */
uint8 reflectiveSensorsGetValue(uint8 index);


#endif /* REFLECTIVE_SENSORS_H_ */
