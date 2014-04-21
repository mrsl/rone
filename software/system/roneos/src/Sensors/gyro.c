/*
 * @file gyro.c
 * @brief Interface to initialize and read 3D gyro data.
 * @since July 22, 2010
 * @author James McLurkin
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_qei.h"

#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "roneos.h"



#if defined(RONE_V6)

#define GYRO_ADC_CHANNEL 3

int8 gyroval;

//TODO There are two gyro_init - which one desired for system.c? Should one be deleted?
/*
 * @brief Initializes gyro.
 *
 * Sets sampling speed, configures and enables the gyro.
 * @returns void
 */
void gyro_init(void) {
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	//Set sampling speed
	MAP_SysCtlADCSpeedSet(SYSCTL_ADCSPEED_500KSPS);

	//Disable sample sequences for configuration
	MAP_ADCSequenceDisable(ADC_BASE, GYRO_ADC_CHANNEL);

	//Timer Trigger, priorities are same as number
	MAP_ADCSequenceConfigure(ADC_BASE, GYRO_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, GYRO_ADC_CHANNEL);

	MAP_ADCSequenceStepConfigure(ADC_BASE, GYRO_ADC_CHANNEL, 0, ADC_CTL_CH3 | ADC_CTL_END);

	//Config sequence steps
	MAP_ADCSequenceEnable(ADC_BASE, GYRO_ADC_CHANNEL);

}

/*
 * @brief Gets the ADC value of the gyro.
 *
 * @returns the value of the gyro
 */
int32 gyroGetValue(uint32 axis) {
	uint32 adc_val = 22;
	uint32 samples = 0;

	MAP_ADCProcessorTrigger(ADC_BASE, GYRO_ADC_CHANNEL);

	while (samples < 1) {
		samples = MAP_ADCSequenceDataGet(ADC_BASE, GYRO_ADC_CHANNEL, &adc_val);
	}
	return adc_val;
}
#endif


#if defined(RONE_V9)

#define GYRO_XMSB_IDX					0
#define GYRO_XLSB_IDX					1
#define GYRO_YMSB_IDX					2
#define GYRO_YLSB_IDX					3
#define GYRO_ZMSB_IDX					4
#define GYRO_ZLSB_IDX					5

int16 gyroX;
int16 gyroY;
int16 gyroZ;

/*
 * @brief Initialize gyro.
 *
 * (gyroX, gyroY, gyroZ) initialized to 0.
 * @returns void
 */
void gyro_init(void) {
	gyroX = 0;
	gyroY = 0;
	gyroZ = 0;
}

/*
 * @brief Get gyro value on axis.
 *
 * @param axis specifies GYRO_X_AXIS, GYRO_Y_AXIS, or GYRO_Z_AXIS.
 * @returns if argument is valid axis, gyro value on specified axis; else, 0
 */
int32 gyroGetValue(uint32 axis) {
	switch(axis) {
	case GYRO_X_AXIS:
		return gyroX;
	case GYRO_Y_AXIS:
		return gyroY;
	case GYRO_Z_AXIS:
		return gyroZ;
	default:
		return 0;
	}
}

/*
 * @brief Update gyro values.
 *
 * gyroX, gyroY, and gyroZ are updated by values in msgIn that are concatenated in pairs according to axis.
 * Each pair of 9-bit integers converted into one 16-bit integer.
 * @param msgIn is array of length 6
 * @returns void
 */
void gyroUpdate(uint8 msgIn[]) {
//convert 2 8-bit unsigned integers into one 16-bit integer
	gyroX = msgIn[GYRO_XMSB_IDX] << 8;
	gyroX ^= msgIn[GYRO_XLSB_IDX];

	gyroY = msgIn[GYRO_YMSB_IDX] << 8;
	gyroY ^= msgIn[GYRO_YLSB_IDX];

	gyroZ = msgIn[GYRO_ZMSB_IDX] << 8;
	gyroZ ^= msgIn[GYRO_ZLSB_IDX];
}
#endif

#if defined(RONE_V12)

#define GYRO_XMSB_IDX					0
#define GYRO_XLSB_IDX					1
#define GYRO_YMSB_IDX					2
#define GYRO_YLSB_IDX					3
#define GYRO_ZMSB_IDX					4
#define GYRO_ZLSB_IDX					5

int16 gyroX;
int16 gyroY;
int16 gyroZ;

/*
 * @brief Initialize gyro.
 *
 * (gyroX, gyroY, gyroZ) initialized to 0.
 * @returns void
 */
void gyro_init(void) {
	gyroX = 0;
	gyroY = 0;
	gyroZ = 0;
}

/*
 * @brief Get gyro value on axis.
 *
 * @param axis specifies GYRO_X_AXIS, GYRO_Y_AXIS, or GYRO_Z_AXIS.
 * @returns if argument is valid axis, gyro value on specified axis; else, 0
 */
int32 gyroGetValue(uint32 axis) {
	switch(axis) {
	case GYRO_X_AXIS:
		return gyroX;
	case GYRO_Y_AXIS:
		return gyroY;
	case GYRO_Z_AXIS:
		return gyroZ;
	default:
		return 0;
	}
}

/*
 * @brief Update gyro values.
 *
 * gyroX, gyroY, and gyroZ are updated by values in msgIn that are concatenated in pairs according to axis.
 * Each pair of 9-bit integers converted into one 16-bit integer.
 * @param msgIn is array of length 6
 * @returns void
 */
void gyroUpdate(uint8 msgIn[]) {
//convert 2 8-bit unsigned integers into one 16-bit integer
	gyroX = msgIn[GYRO_XMSB_IDX] << 8;
	gyroX ^= msgIn[GYRO_XLSB_IDX];

	gyroY = msgIn[GYRO_YMSB_IDX] << 8;
	gyroY ^= msgIn[GYRO_YLSB_IDX];

	gyroZ = msgIn[GYRO_ZMSB_IDX] << 8;
	gyroZ ^= msgIn[GYRO_ZLSB_IDX];
}
#endif
