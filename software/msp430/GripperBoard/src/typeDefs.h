/*
 * typeDefs.h
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <msp430.h>
#include <stdio.h>

#define TRUE	1
#define FALSE	0

#define CURRENT_RUN_AVG_LEN_EXPO	2
#define CURRENT_RUN_AVG_LEN			1 << CURRENT_RUN_AVG_LEN_EXPO

typedef unsigned char uint8;
typedef unsigned short int uint16;
typedef unsigned long int uint32;

typedef struct {
	unsigned int runValues[CURRENT_RUN_AVG_LEN];	//keeps track of current values
	unsigned int runValuesCount;			//keeps track of most recently updated value
	unsigned long sum;		//keeps track of temporary sum
	unsigned long average;		//final sum
} current;

#endif /* TYPEDEFS_H_ */
