/*
 * currentSense.h
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */


#ifndef CURRENTSENSE_H_
#define CURRENTSENSE_H_

#include "typeDefs.h"

/* Current Variables */
#define CURRENT  	BIT0
#define CURRENT_OUT	P1OUT
#define THRESHOLD_HIGH 	300
#define THRESHOLD_LOW	150


/* Settings counter values for TA = 2MHz = SMCLK/8 */
#define ADC_SAMPLE		1050

void currentUpdate(void);
void configADC10(void);
uint8 forceGet(void);
void configTimerA0(void);

#endif /* CURRENTSENSE_H_ */
