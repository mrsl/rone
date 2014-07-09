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
	ADC10AE0 |= REFLECTIVE_ADC_PORTS;
}


void reflectiveSensorPowerEnable() {
	REFLECTIVE_POWER_EN_OUT |= REFLECTIVE_POWER_EN_BIT; // Configure pin to high
}


void reflectiveSensorPowerDisable() {
	REFLECTIVE_POWER_EN_OUT &= ~REFLECTIVE_POWER_EN_BIT; // Configure pin to low
}


void reflectiveSensorsUpdate() {
	uint32 i;

	// Enable the reflective sensor LED output
	reflectiveSensorPowerEnable();

	// Wait a bit for the analog values to stablize
	//TODO CHANGE delay here
	for (i = 0; i < REFLECTIVE_SENSORS_ENABLE_DELAY; ++i) {}

	// Sample all 5 channels of reflective sensors (A0, A1, A2, A3, A6)
   	ADC10CTL0 &= ~ENC;             						// Turn off ADC10 to switch channel
	ADC10CTL1 = INCH_0 | ADC10DIV_0 | CONSEQ_0;			// Input A0, single sequence
   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
   	reflectiveSensorVal[0] = ADC10MEM;

   	ADC10CTL0 &= ~ENC;             						// Turn off ADC10 to switch channel
	ADC10CTL1 = INCH_1 | ADC10DIV_0 | CONSEQ_0;			// Input A1, single sequence
   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
   	reflectiveSensorVal[1] = ADC10MEM;

   	ADC10CTL0 &= ~ENC;             						// Turn off ADC10 to switch channel
	ADC10CTL1 = INCH_2 | ADC10DIV_0 | CONSEQ_0;			// Input A2, single sequence
   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
   	reflectiveSensorVal[2] = ADC10MEM;

   	ADC10CTL0 &= ~ENC;             						// Turn off ADC10 to switch channel
	ADC10CTL1 = INCH_3 | ADC10DIV_0 | CONSEQ_0;			// Input A3, single sequence
   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
   	reflectiveSensorVal[3] = ADC10MEM;

   	ADC10CTL0 &= ~ENC;             						// Turn off ADC10 to switch channel
	ADC10CTL1 = INCH_6 | ADC10DIV_0 | CONSEQ_0;			// Input A6, single sequence
   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
   	reflectiveSensorVal[4] = ADC10MEM;

	// Disable the reflective sensor LED output
   	reflectiveSensorPowerDisable();
}


#endif //#ifdef RONE_V12_TILETRACK
