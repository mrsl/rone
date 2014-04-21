/*
 * @file bump_sensor.c
 *
 * @brief Reads bump sensor information and includes helper functions for processing this data.
 *
 * @since July 22, 2010
 * @author James McLurkin
 */

#include "roneos.h"
#include "bump_sensor.h"

static uint8 bumpSensors = 0;
static int16 bumpSensorBearing = -1;


#if (defined(RONE_V9) || defined(RONE_V12))
/*
 * @brief Update bump sensors.
 *
 * bumpSensorBearing set to -1.
 * @param val the bump sensor value
 * @returns void
 */
void bumpSensorsUpdate(uint8 val) {
	bumpSensors = val;
	bumpSensorBearing = -1;
}
#endif

/*
 * @brief Get bump sensor bits.
 *
 * @returns the bump sensor bits
 */
uint8 bumpSensorsGetBits() {
	return bumpSensors;
}

/*
 * @brief Returns an integer identifying from which direction the robot was hit.
 *
 * 0 - Hit from front
 * 1 - Hit from left
 * 2 - Hit from right
 * 3 - Hit from behind
 * 4 - NOT being hit
 *
 * If the robot is not being hit, returns -1.
 *
 * @returns An integer corresponding to the robot bump sensor that is active.
 */
uint8 bumpSensorsWhichHit(){
	int16 bumpSensorAngle = bumpSensorsGetBearing();
	if(bumpSensorAngle == -1){
		return 4;
	}
	if ((bumpSensorAngle < 400) && (bumpSensorAngle > -400))
	{
		return 0;//front
	}
	else if ((bumpSensorAngle < 2000) && (bumpSensorAngle > 400))
	{
		return 1;//left
	}
	else if ((bumpSensorAngle < -2000) || (bumpSensorAngle > 2000))
	{
		return 2;//back

	} else if ((bumpSensorAngle < -400) && (bumpSensorAngle > -2000))
	{
		return 3;//right
	}
	else
	{
		return 4;
	}

}

/*
 * @brief Get bump sensor bearing.
 *
 * @returns the bump sensor bearing
 */
int16 bumpSensorsGetBearing() {
	if (bumpSensors == 0) {
		bumpSensorBearing = -1;
	} else {
		if (bumpSensorBearing == -1) {
			// compute the bump sensor bearing
			bumpSensorBearing = angleFromBitVectorOffset(bumpSensors);
		}
	}
	return bumpSensorBearing;
}





