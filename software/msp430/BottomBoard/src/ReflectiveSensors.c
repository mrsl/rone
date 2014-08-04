#include <stdio.h>
#include <msp430f2132.h>

#ifdef RONE_V12_TILETRACK

#include "typedefs.h"
#include "ReflectiveSensors.h"

uint16 reflectiveSensorVal[NUM_REFLECTIVE_PORTS];


void reflectiveSensorsInit() {
	// Setup power enable pin
	REFLECTIVE_POWER_EN_SEL &= ~REFLECTIVE_POWER_EN_BIT; // Configure pin to be GPIO
	REFLECTIVE_POWER_EN_DIR |= REFLECTIVE_POWER_EN_BIT; // Configure pin to be output

	// Setup sensor pins to ADC10
	REFLECTIVEA_PORT_REN &= ~(REFLECTIVEA_ADC_PORTS);
	REFLECTIVEB_PORT_REN &= ~(REFLECTIVEB_ADC_PORTS);
	ADC10AE0 |= (REFLECTIVEA_ADC_PORTS + REFLECTIVEB_ADC_PORTS);

	// Get initial values
	reflectiveSensorsUpdate();
}


void reflectiveSensorPowerEnable() {
	REFLECTIVE_POWER_EN_OUT |= REFLECTIVE_POWER_EN_BIT; // Configure pin to high
}


void reflectiveSensorPowerDisable() {
	REFLECTIVE_POWER_EN_OUT &= ~REFLECTIVE_POWER_EN_BIT; // Configure pin to low
}


uint16 reflectiveSampleADCPort(uint32 channel) {
	uint16 ADCValue;
   	ADC10CTL0 &= ~ENC;             						// Turn off ADC10 to switch channel
	ADC10CTL1 = channel | ADC10DIV_0 | CONSEQ_0;			// Input A0, single sequence
   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
   	ADCValue = ADC10MEM;
   	return ADCValue;
}

void reflectiveSensorsUpdate() {
	uint32 i;

	// Enable the reflective sensor LED output
	reflectiveSensorPowerEnable();

	// Wait a bit for the analog values to stablize
	//TODO CHANGE delay here
	for (i = 0; i < REFLECTIVE_SENSORS_ENABLE_DELAY; ++i) {}

	// Sample all 5 channels of reflective sensors (A0, A1, A2, A3, A6)
   	reflectiveSensorVal[0] = reflectiveSampleADCPort(REFLECTIVE_ADC_CHANNEL_0);
   	reflectiveSensorVal[1] = reflectiveSampleADCPort(REFLECTIVE_ADC_CHANNEL_1);
   	reflectiveSensorVal[2] = reflectiveSampleADCPort(REFLECTIVE_ADC_CHANNEL_2);
   	reflectiveSensorVal[3] = reflectiveSampleADCPort(REFLECTIVE_ADC_CHANNEL_3);
   	reflectiveSensorVal[4] = reflectiveSampleADCPort(REFLECTIVE_ADC_CHANNEL_4);


	// Disable the reflective sensor LED output
//   	reflectiveSensorPowerDisable();
}


uint8 reflectiveGetData(int i) {
	// Normalize to uint8 range
	return ((uint32)reflectiveSensorVal[i] * 255 / 1023);
}


#endif //#ifdef RONE_V12_TILETRACK
