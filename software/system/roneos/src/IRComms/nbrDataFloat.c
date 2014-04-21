/*
 * Author: Golnaz
 * Date: Sept. 2013
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "roneos.h"
#include "neighborsInternal.h"




// private leave in c file
typedef union FloatConvert {
	float val;
	uint8 bytes[4];
} FloatConvert;


void nbrDataCreateFloat(NbrDataFloat* msgPtr, const char* name) {
	nbrDataCreate(&msgPtr->msgFloat0, name, 8, 0);
	nbrDataCreate(&msgPtr->msgFloat1, name, 8, 0);
	nbrDataCreate(&msgPtr->msgFloat2, name, 8, 0);
	nbrDataCreate(&msgPtr->msgFloat3, name, 8, 0);
}


// pack a 32-bit float into 4 8-bit messages
void nbrDataSetFloat(NbrDataFloat* msgPtr, float val) {
	FloatConvert fc;
	fc.val = val;
	nbrDataSet(&msgPtr->msgFloat0, fc.bytes[0]);
	nbrDataSet(&msgPtr->msgFloat1, fc.bytes[1]);
	nbrDataSet(&msgPtr->msgFloat2, fc.bytes[2]);
	nbrDataSet(&msgPtr->msgFloat3, fc.bytes[3]);
}

// pack a 32-bit float into 4 8-bit messages
float nbrDataGetNbrFloat(NbrDataFloat* msgPtr, Nbr* nbrPtr) {
	FloatConvert fc;
	fc.bytes[0] = nbrDataGetNbr(&msgPtr->msgFloat0, nbrPtr);
	fc.bytes[1] = nbrDataGetNbr(&msgPtr->msgFloat1, nbrPtr);
	fc.bytes[2] = nbrDataGetNbr(&msgPtr->msgFloat2, nbrPtr);
	fc.bytes[3] = nbrDataGetNbr(&msgPtr->msgFloat3, nbrPtr);
	return fc.val;
}

float nbrDataGetFloat(NbrDataFloat* Float) {
	FloatConvert fc;
	fc.bytes[0] = nbrDataGet(&(Float->msgFloat0));
	fc.bytes[1] = nbrDataGet(&(Float->msgFloat1));
	fc.bytes[2] = nbrDataGet(&(Float->msgFloat2));
	fc.bytes[3] = nbrDataGet(&(Float->msgFloat3));
	return fc.val;
}
