/*
 * currentSense.c
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */


#include "typeDefs.h"
#include "currentSense.h"
#include "SPI.h"
#include "IO.h"
#include "servo.h"

//unsigned int currentRunAvg[CURRENT_RUN_AVG_LEN];
//unsigned int currentRunAvgCount = 0;

current currentValues = { 0 };
uint8 force = 0;

/* Finds the average of the currents */
void currentAverage(void) {
	int i;

	currentValues.sum = 0;
	for (i = 0; i < CURRENT_RUN_AVG_LEN; i++) {
		currentValues.sum += currentValues.runValues[i];
	}
	currentValues.average = (currentValues.sum >> CURRENT_RUN_AVG_LEN_EXPO); //average = sum / CURRENT_RUN_AVERAGE_LENGTH

}

/* Updates our average and packs it into the SPI buffer*/
void currentUpdate(void) {
	currentAverage();
	messagePackCurrentAverage(currentValues);
}

/* calculates if our force is too high */
uint8 forceGet(void) {
	if (force && (currentValues.average < THRESHOLD_LOW)) {
		force =  0;			// if we had high current and have reached our low threshold, go low
	} else if (!force && (currentValues.average > THRESHOLD_HIGH)) {
		force =  1;			// if we had low current and have reached our high threshold, go high
	}
	return force;
}


void configTimerA0(void) {
	TA0CTL = TASSEL_2 + MC_1 + ID_3;	// SMCLK/8, up mode
	TA0CCTL2 =
	TA0CCR0 = PULSE_PERIOD; 	// Servo signal period
	TA0CCR2 = ADC_SAMPLE;		// ADC sample time
}

/* configures ADC to sample with */
void configADC10(void) {
	ADC10CTL1 = (INCH_0 | ADC10DIV_7 | CONSEQ_2 | SHS_3);	// sample A0, div clock by 8, single channel conversion, trigger on TA0.2
	ADC10CTL0 = (ADC10SHT_0 | ADC10ON | ADC10IE | ENC);		// hold=64*ADC10CLKs, ADC on,  interrupt enable, start conversion
}


#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
	currentValues.runValues[currentValues.runValuesCount++] = ADC10MEM;
	if (currentValues.runValuesCount == CURRENT_RUN_AVG_LEN) {
		currentValues.runValuesCount = 0;
	}
}
