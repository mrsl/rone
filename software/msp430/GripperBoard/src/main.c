/*
 * One servo controlled by '62 via SPI
 *
 * one servo's positions is controlled by byte values sent from the '62 top board chip.
 * The current values for each servo are sent back to the '62. The Servo signal duration is
 * implemented using timers.
 *
 *	TODO: put into LPM sometimes
 *
 * Updated on: Mar 3, 2014
 * Author: William Xie, Taylor Vaughn
 */

#include "typeDefs.h"
#include "currentSense.h"
#include "SPI.h"
#include "IO.h"
#include "servo.h"

/* Maximum number of TA1 (Timer A1) cycles required to run the CCR1 ISR (First compare/capture comparison interrupt vector) */
#define MIN_GAP	22

/* Servo variables */
volatile uint8 servoValue =  15;
volatile uint16 servoPWMValue = 3000;		// Stores the values for servo PWM on time
//volatile uint16 servoPWMValueOld = 3000;

volatile uint8	countUp = 0;
volatile uint16 pulseGoal = 3000;
volatile uint16 pulseGoalOld = 3000;
//volatile uint8  debugAscending = 1;
//volatile uint8 debugGoal = 0;

/* Run if we break */
void faultRoutine(void) {
	int i;
	P3OUT |= LED;				// Turn LED off
	for (i = 0; i < 80; i++)
		_delay_cycles(100000);
	P3OUT &= ~LED;				// Turn LED on
	for (i = 0; i < 80; i++)
		_delay_cycles(100000);
}

/* configure clocks */
void configClocks(void) {
	if (CALBC1_16MHZ == 0xFF || CALDCO_16MHZ == 0xFF)
		faultRoutine(); 	// If calibration data is erased run FaultRoutine()
	BCSCTL1 = CALBC1_16MHZ; 					// Set range
	DCOCTL = CALDCO_16MHZ; 					// Set DCO step + modulation

	BCSCTL2 |= SELM_0 + DIVM_0; 			// MCLK = SMCLK = DCO
	BCSCTL2 &= ~SELS;
	BCSCTL2 |= DIVS_0;

	BCSCTL3 |= LFXT1S_2; 					// LFXT1 = VLO

	IFG1 &= ~OFIFG; 						// Clear OSCFault flag
}


/* Used to configure servo timing */
void configTimerA1(void) {
	TA1CTL = TASSEL_2 + MC_1 + ID_3;	// SMCLK/8, up mode
	TA1CCR0 = PULSE_PERIOD; 			// Servo signal period
	TA1CCR1 = PULSE_OFFSET; 			// Servo min pulse length
	TA1CCR2 = PULSE_UPDATE;				// Servo update time

	TA1CCTL1 |= OUTMOD_7;				// reset/set
	TA1CCTL2 |= CCIE;					// Enable TA1 interrupt on CCR2
}


int main(void) {
   	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog time
	configClocks();
	configPins();
	configTimerA0();
	configTimerA1();
	configADC10();
	initSPI();
	initMessage();

	_enable_interrupts();		// enable global interrupts

	while (1) {
		//pack our new message
		GPIOUpdate();
		readCurrentADC();
		readPotentiometerADC();
		currentUpdate();
		messagePackServoValues(servoValue);
	}
}


#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1(void) {
	if (!(TA1IV & TA1IV_TACCR2)) {
		return;
	}
	pulseGoalOld = pulseGoal;
//	if(debugAscending && debugGoal < 255) {
//		debugGoal++;
//	}
//	else if (debugGoal == 255) {
//		debugAscending= 0;
//		debugGoal--;
//	}
//	else if(!debugAscending && debugGoal > 0) {
//		debugGoal--;
//	}
//	else if(debugGoal == 0) {
//		debugAscending = 1;
//		debugGoal++;
//	}
	pulseGoal = (messageGetServoValue() * SERVO_SCALE_VALUE) + PULSE_OFFSET; // if we are not gripping anything, approach our goal pos.
//	pulseGoal = (debugGoal * SERVO_SCALE_VALUE) + PULSE_OFFSET;
	if (pulseGoalOld < pulseGoal ){		// If we are moving up
		countUp = 1;
	} else if ( pulseGoalOld > pulseGoal) {
		countUp = 0;
	}


	// If our current is too high (ie, we are gripping something) we need to back off until we are "safe"
	if (forceGet()) {
				if (countUp && (pulseGoalOld < servoPWMValue) && (pulseGoal >= servoPWMValue)){ //if we are counting up and have not counted past servoPWMValue
					servoPWMValue = pulseGoal + PULSE_ADJUST;
				} else if (!countUp && (pulseGoalOld > servoPWMValue) && (pulseGoal <= servoPWMValue)){ //if we are couting down have not counted past servoPWMValue
					servoPWMValue = pulseGoal - PULSE_ADJUST;
				}// else {
				//	servoPWMValue = servoPWMValue;
				//}
	} else {
	// if we are moving the "wrong" direction, or if we have low force, increment towards goal
	if (servoPWMValue > (pulseGoal + (PULSE_ADJUST << 1) )) {	// If our value is enough greater than the goal value, go down by ADJUST
				servoPWMValue -= (PULSE_ADJUST << 1);
			} else if (servoPWMValue < (pulseGoal - (PULSE_ADJUST << 1))) {	// If our value is enough less than the goal value, go up by adjust
				servoPWMValue += (PULSE_ADJUST << 1);
			} else {
				servoPWMValue = pulseGoal;	// If we are within range, use the exact value
			}
			//break;

	}


	TA1CCR1 = servoPWMValue;

	servoValue =  (servoPWMValue - PULSE_OFFSET)/SERVO_SCALE_VALUE;	// convert back so that the robot knows what our current position actually is.
}
