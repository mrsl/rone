/*
 * servo.h
 *
 *  Created on: Mar 21, 2014
 *      Author: Taylor
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "typeDefs.h"

/* Settings TA1 counter values for TA = 2MHz = SMCLK/8 */
#define SMCLK_FREQ		16000000LL
#define TA1_CLK_FREQ	(SMCLK_FREQ/8)
#define PULSE_MIN_US	500		// Min. pulse length = 500us, Max. pulse length = 2500us
#define PULSE_MAX_US	2500
#define PULSE_ADJUST_US	5		// If current is too high, move left/right 50us
#define PULSE_PERIOD_US	25000	// Pulse period = 25ms
#define POS_HALF_US		((PULSE_MIN_US - PULSE_MAX_US) >> 1) + PULSE_MAX_US

/* Convert from micro seconds to counts of TA1 */
#define US_TO_TA1_COUNT(us)		((uint16)((us * TA1_CLK_FREQ) / 1000000LL))
#define PULSE_OFFSET	US_TO_TA1_COUNT(PULSE_MIN_US)
#define PULSE_MIN		US_TO_TA1_COUNT(PULSE_MIN_US)
#define PULSE_MAX		US_TO_TA1_COUNT(PULSE_MAX_US)
#define PULSE_ADJUST	US_TO_TA1_COUNT(PULSE_ADJUST_US)
#define PULSE_PERIOD 	US_TO_TA1_COUNT(PULSE_PERIOD_US)
#define POS_HALF	 	US_TO_TA1_COUNT(POS_HALF_US)
#define PULSE_UPDATE 	30000-1 // Sampling point in the period


/* TODO: Make this scale value on the 8962 side probs */
#define SERVO_SCALE_VALUE			22


#endif /* SERVO_H_ */
