/*
 * BroadcastMinMax.c
 *
 *  Created on: Apr 10, 2013
 *      Author: jamesm
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

//TODO: doesn't actually use all the input parameters? Why? (Used in TriDemo so didn't want to break)
/**
 * @brief creates a min/max broadcast message in msgPtr
 *
 * @param msgPtr
 * @param name
 * @param val
 * @returns void
 */
void broadcastMinMaxCreate(BroadcastMinMaxMessage* msgPtr, char* name, uint8 val) {
	nbrDataCreate(&msgPtr->val, "val", 8, val);
	nbrDataCreate(&msgPtr->valLocal, "vallocal", 8, val);
	msgPtr->hasBeenSet = FALSE;
}


/**
 * @brief creates a min/max broadcast message in msgPtr
 *
 * @param msgPtr
 * @param hopsMax the maximum number of hops that this message can propogate
 * @returns void
 */
uint8 minMaxFindMaxNbrs(BroadcastMinMaxMessage* msgPtr, NbrList* nbrListPtr) {
	Nbr* nbrPtr;
	uint8 i, val;
	uint8 valmax = nbrDataGet(&msgPtr->valLocal);

	for (i = 0; i < nbrListGetSize(nbrListPtr); i++){
		nbrPtr = nbrListGetNbr(nbrListPtr, i);
		val = nbrDataGetNbr(&msgPtr->val, nbrPtr);
		if (val > valmax){
			valmax = val;
		}
	}
	return valmax;
}


/**
 * @brief creates a min/max broadcast message in msgPtr
 *
 * @param msgPtr
 * @param nbrListPtr list of neibors
 * @returns void
 */
void broadcastMinMaxSetMaxPlusOne(BroadcastMinMaxMessage* msgPtr, NbrList* nbrListPtr) {
	uint8 valLocal;
	if(msgPtr->hasBeenSet == FALSE) {
		valLocal = minMaxFindMaxNbrs(msgPtr, nbrListPtr) + 1;
		nbrDataSet(&msgPtr->valLocal, valLocal);
		broadcastMinMaxSet(msgPtr, valLocal);
	}
}


/**
 * @brief creates a min/max broadcast message in msgPtr
 *
 * @param msgPtr
 * @param val the maximum number of hops that this message can propagate
 * @returns void
 */
void broadcastMinMaxSet(BroadcastMinMaxMessage* msgPtr, uint8 val) {
	if(msgPtr->hasBeenSet == FALSE) {
		nbrDataSet(&msgPtr->valLocal, val);
		msgPtr->hasBeenSet = TRUE;
	}
}


/**
 * @brief propagates a min.max message
 *
 * @param msgPtr
 * @param nbrListPtr list of neibors
 * @returns void
 */
uint8 broadcastMinMaxUpdate(BroadcastMinMaxMessage* msgPtr, NbrList* nbrListPtr) {
	uint8 val;

	val = minMaxFindMaxNbrs(msgPtr, nbrListPtr);
	val = max(val, nbrDataGet(&msgPtr->valLocal));
	nbrDataSet(&msgPtr->val, val);
	return val;
}


/**
 * @brief gets a min/max broadcast message local value in msgPtr
 *
 * @param msgPtr
 * @returns void
 */
uint8 broadcastMinMaxGetValLocal(BroadcastMinMaxMessage* msgPtr) {
	return nbrDataGet(&msgPtr->valLocal);
}


/**
 * @brief gets a min/max broadcast message value in msgPtr
 *
 * @param msgPtr
 * @returns void
 */
uint8 broadcastMinMaxGetVal(BroadcastMinMaxMessage* msgPtr) {
	return nbrDataGet(&msgPtr->val);
}

