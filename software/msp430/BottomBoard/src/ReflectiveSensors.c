#include <stdio.h>
#include <msp430f2132.h>

#ifdef RONE_V12_TILETRACK

#include "typedefs.h"
#include "ReflectiveSensors.h"

uint16 reflectiveSensorVal[NUM_REFLECTIVE_PORTS];


void reflectiveSensorsInit() {
//	// Power enable pin
//	REFLECTIVE_POWER_EN_DIR |= REFLECTIVE_POWER_EN_BIT; // Configure pin to be output
//	REFLECTIVE_POWER_EN_SEL &= ~REFLECTIVE_POWER_EN_BIT; // Configure pin to be GPIO
//	reflectiveSensorPowerEnable();
//
//	// Individual sensor input pins
//	REFLECTIVEA_PORT_DIR &= ~REFLECTIVEA_BITS;		// Configure pins to be inputs
//	REFLECTIVEA_PORT_SEL &= ~REFLECTIVEA_BITS;		// Configure pins to be GPIO
//	REFLECTIVEB_PORT_DIR &= ~REFLECTIVEB_BITS;		// Configure pins to be inputs
//	REFLECTIVEB_PORT_SEL &= ~REFLECTIVEB_BITS;		// Configure pins to be GPIO
//
//	reflectiveSensorsUpdate();
}


void reflectiveSensorPowerEnable() {
//	REFLECTIVE_POWER_EN_OUT |= REFLECTIVE_POWER_EN_BIT; // Configure pin to high
}


void reflectiveSensorPowerDisble() {
//	REFLECTIVE_POWER_EN_OUT &= ~REFLECTIVE_POWER_EN_BIT; // Configure pin to low
}


void reflectiveSensorsUpdate() {
//	// enable the reflective sensor LED output
//	reflectiveSensorPowerEnable();
//
//	// wait a bit for the analog values to stablize
//	//TODO add delay here
//
//	// Sample all 5 channels of reflective sensors (A0, A1, A2, A3, A6)
//	ADC10CTL1 = INCH_0 | ADC10DIV_0 | CONSEQ_0;			// Input A0, single sequence
//   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
//   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
//   	reflectiveSensorVal[0] = ADC10MEM;
//
//	ADC10CTL1 = INCH_1 | ADC10DIV_0 | CONSEQ_0;			// Input A1, single sequence
//   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
//   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
//   	reflectiveSensorVal[1] = ADC10MEM;
//
//	ADC10CTL1 = INCH_2 | ADC10DIV_0 | CONSEQ_0;			// Input A2, single sequence
//   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
//   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
//   	reflectiveSensorVal[2] = ADC10MEM;
//
//	ADC10CTL1 = INCH_3 | ADC10DIV_0 | CONSEQ_0;			// Input A3, single sequence
//   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
//   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
//   	reflectiveSensorVal[3] = ADC10MEM;
//
//	ADC10CTL1 = INCH_6 | ADC10DIV_0 | CONSEQ_0;			// Input A6, single sequence
//   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
//   	while (ADC10CTL1 & BUSY); 							// Wait until sample ends
//   	reflectiveSensorVal[4] = ADC10MEM;
//
//	// disable the reflective sensor LED output
//	reflectiveSensorPowerDisble();
}


#endif //#ifdef RONE_V12_TILETRACK
