/*
 * IO.c
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */

#include "typeDefs.h"
#include "currentSense.h"
#include "SPI.h"
#include "IO.h"

/*Position Variables*/
unsigned int hallEffect0;
unsigned int hallEffect1;
unsigned int forceBit;


void positionGet(void) {
	hallEffect0 = (HALL_EFFECT0_IN & HALL_EFFECT0) >> HALL0_OFFSET;
	hallEffect1 = (HALL_EFFECT1_IN & HALL_EFFECT1) >> HALL1_OFFSET;
}

void GPIOUpdate(void) {
	positionGet();
	forceBit = forceGet();
	//messagePackIO((1 << SOFT_STOP0_OFFSET | 0 << SOFT_STOP1_OFFSET | 1 << FORCE_BIT_OFFSET));
	messagePackIO((hallEffect0 << SOFT_STOP0_OFFSET | hallEffect1 << SOFT_STOP1_OFFSET | forceBit << FORCE_BIT_OFFSET));
}

void configPins(void)
{

	/* White LED setup */
	P3DIR |= LED; 			// Make P3.6 an output
	P3OUT |= LED;			// P3.6 is off
	P3REN &= ~LED;			// Pull up resistor off

	/* Current sense setup */
	P1DIR &= ~(CURRENT);			// Assign P1.0 as current sense input
	P1SEL &= ~(CURRENT);			// I/O mode
	P1SEL2 &= ~(CURRENT);
	P1REN &= ~(CURRENT);			// No resistors enabled for current sense


	/* Servo pins setup */
	// PWM mode
	P2DIR |= (SERVO);			// 1
	P2SEL |= (SERVO);			// 1
	P2SEL2 &= ~(SERVO);			// 0
	P2REN &= ~(SERVO);			// Pullup resistors off

	/* Hall effect pin setup */
	P1DIR &= ~(HALL_EFFECT0);		// Inputs
	P1REN &= ~(HALL_EFFECT0);		// pullup off
	P1SEL &= ~(HALL_EFFECT0);
	P1SEL2 &= ~(HALL_EFFECT0);

	P2DIR &= ~(HALL_EFFECT1);		// Inputs
	P2REN &= ~(HALL_EFFECT1);		// pullup off
	P2SEL &= ~(HALL_EFFECT1);
	P2SEL2 &= ~(HALL_EFFECT1);


}
