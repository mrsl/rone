/*
 * IO.h
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */

#ifndef IO_H_
#define IO_H_

#include "typeDefs.h"


//HALL_EFFECT0 = P1.3, HALL_EFFECT1 = P2.7
#define HALL_EFFECT0		BIT3
#define HALL_EFFECT1		BIT7
//#define HALL_EFFECT_PINS	(HALL_EFFECT0 + HALL_EFFECT1)
#define HALL_EFFECT_DIR		2DIR
#define HALL_EFFECT0_IN		P1IN
#define HALL_EFFECT1_IN		P2IN

#define SERVO  		BIT2
#define SERVO_OUT	P2OUT

#define HALL0_OFFSET		3
#define HALL1_OFFSET		7

#define LED 	BIT6

void GPIOUpdate(void);
void configPins(void);


#endif /* IO_H_ */
