/*
 * CoMData.h
 *
 *  Created on: Dec 23, 2013
 *      Author: Golnaz Habibi
 */

#ifndef COMDATA_H_
#define COMDATA_H_




/**
 * @brief Information of CoM stored on a network neighbor.
 */

typedef struct NbrDataCoM {
	NbrDataFloat xpos;
	NbrDataFloat ypos;
	NbrData req_ID;
	NbrData ack_ID;
	NbrData nonce;
} NbrDataCoM;



/**
 * @brief Information of Ack CoM stored on a network neighbor.
 */




void nbrDataCoMCreate(NbrDataCoM* nbrCoMDataPtr, const char * name);
void nbrDataCoMSetPos(NbrDataCoM* nbrCoMDataPtr, float xvalue, float yvalue);
void nbrDataCoMSetReqID(NbrDataCoM* nbrCoMDataPtr, uint8 reqvalue);
void nbrDataCoMSetAckID(NbrDataCoM* nbrCoMDataPtr, uint8 ackvalue);
void nbrDataCoMSetNonce(NbrDataCoM* nbrCoMDataPtr, uint8 noncevalue);

float nbrDataCoMGetx(NbrDataCoM* nbrCoMDataPtr,Nbr* nbrPtr);
float nbrDataCoMGety(NbrDataCoM* nbrCoMDataPtr,Nbr* nbrPtr);
uint8 nbrDataCoMGetReqID(NbrDataCoM* nbrCoMDataPtr,Nbr* nbrPtr);
uint8 nbrDataCoMGetAckID(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr);
uint8 nbrDataCoMGetNonce(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr);


/**
 * Lookup table for the range data of robots in the tear drop obstacle
 * robots
 *
 *
 */

int16 getnbrRange(int32 id1, int32 id2);

#endif /* COMDATA_H_ */
