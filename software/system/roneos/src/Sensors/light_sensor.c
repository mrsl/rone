/*
 * @file light_sensor.c
 *
 * @brief Interface to initialize and read light sensor ring on robot.
 *
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
#include "lightSensorCalibration.h"

#if defined(RONE_V9) || defined(RONE_V6)

#define LIGHT_SENSOR_FL_ADC_CHANNEL			0
#define LIGHT_SENSOR_FR_ADC_CHANNEL			1
#define LIGHT_SENSOR_R_ADC_CHANNEL			2

#define LIGHT_SENSOR_MAX_INDEX 				LIGHT_SENSOR_REAR

#elif defined(RONE_V12)

#define LIGHT_SENSOR_FR_ADC_CHANNEL			0
#define LIGHT_SENSOR_FL_ADC_CHANNEL			1
#define LIGHT_SENSOR_RL_ADC_CHANNEL			2
#define LIGHT_SENSOR_RR_ADC_CHANNEL			3

#define LIGHT_SENSOR_MAX_INDEX 				LIGHT_SENSOR_REAR_LEFT

#endif

/*
 * @brief Initializes light sensor.
 *
 * Sets sampling speed and configures and enables the light sensor.
 * @returns void
 */
void lightSensorInit(void) {
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	//Set sampling speed
	MAP_SysCtlADCSpeedSet(SYSCTL_ADCSPEED_500KSPS);

	//Disable sample sequences for configuration
#if defined(RONE_V9) || defined(RONE_V6)
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL);
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL);
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_R_ADC_CHANNEL);
#elif defined(RONE_V12)
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL);
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL);
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_RL_ADC_CHANNEL);
	MAP_ADCSequenceDisable(ADC_BASE, LIGHT_SENSOR_RR_ADC_CHANNEL);
#endif

	//Timer Trigger, priorities are same as number
#if defined(RONE_V9) || defined(RONE_V6)
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_FL_ADC_CHANNEL);
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_FR_ADC_CHANNEL);
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_R_ADC_CHANNEL,  ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_R_ADC_CHANNEL);
#elif defined(RONE_V12)
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_FR_ADC_CHANNEL);
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_FL_ADC_CHANNEL);
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_RL_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_RL_ADC_CHANNEL);
	MAP_ADCSequenceConfigure(ADC_BASE, LIGHT_SENSOR_RR_ADC_CHANNEL, ADC_TRIGGER_PROCESSOR, LIGHT_SENSOR_RR_ADC_CHANNEL);
#endif

#if defined(RONE_V9) || defined(RONE_V6)
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL, 0, ADC_CTL_CH0 | ADC_CTL_END);
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL, 0, ADC_CTL_CH1 | ADC_CTL_END);
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_R_ADC_CHANNEL,  0, ADC_CTL_CH2 | ADC_CTL_END);
#elif defined(RONE_V12)
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL, 0, ADC_CTL_CH0 | ADC_CTL_END);
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL, 0, ADC_CTL_CH1 | ADC_CTL_END);
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_RL_ADC_CHANNEL, 0, ADC_CTL_CH2 | ADC_CTL_END);
	MAP_ADCSequenceStepConfigure(ADC_BASE, LIGHT_SENSOR_RR_ADC_CHANNEL, 0, ADC_CTL_CH3 | ADC_CTL_END);
#endif

	//Config sequence steps
	// ADC_SSCTL3_R = (ADC_SSCTL3_TS0 | ADC_SSCTL3_END0);
#if defined(RONE_V9) || defined(RONE_V6)
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL);
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL);
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_R_ADC_CHANNEL);
#elif defined(RONE_V12)
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_FR_ADC_CHANNEL);
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_FL_ADC_CHANNEL);
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_RL_ADC_CHANNEL);
	MAP_ADCSequenceEnable(ADC_BASE, LIGHT_SENSOR_RR_ADC_CHANNEL);
#endif
}


//only values 0, 1, 2, (3) allowed (3 allowed on v12 robots). Error check in python.
/*
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
int32 lightSensorGetValue(uint32 light_sensor) {
	uint32 adc_val;
	uint32 samples = 0;
	uint8 robotID;
	uint8 i;

	// check bounds on input arguments
	if(light_sensor > LIGHT_SENSOR_MAX_INDEX) {
		return 0;
	}

	// read the ADC
	MAP_ADCProcessorTrigger(ADC_BASE, light_sensor);
	while (samples < 1) {
		samples = MAP_ADCSequenceDataGet(ADC_BASE, light_sensor, &adc_val);
	}

	// check to see if we have calibration data for this robot
	// if so, subtract off the calibration
	i = 0;
	while(TRUE) {
		if (lightSensorCalibrationData[i][0] == roneID) {
			// this robot has calibration data.  offset the measurements.
			adc_val -= lightSensorCalibrationData[i][light_sensor + 1];
			break;
		} else if (lightSensorCalibrationData[i][0] == ROBOT_ID_NULL) {
			// no more calibration data.  break
			break;
		} else {
			i++;
		}
	}

	return adc_val;
}
