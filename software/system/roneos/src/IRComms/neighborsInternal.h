/*
 * neighborsInternal.h
 *
 *  Created on: Nov 23, 2012
 *      Author: jamesm
 */

#ifndef NEIGHBORSINTERNAL_H_
#define NEIGHBORSINTERNAL_H_

extern NbrData nbrMsgID;

/******** Internal Neighbor System Functions ********/

uint8 neighborMessageUnpackID(uint8* message);
void nbrDataIRMessagePack(IRCommsMessage *irMessage);
void nbrDataIRMessageUnpack(IRCommsMessage *irMessage, Nbr* nbrPtr);


void nbrMsgRadioAddNbr(uint8 ID);
void nbrMsgRadioRemoveNbr(uint8 ID);
void nbrMsgRadioXmit(void);

void irCommsBeaconHighPower(boolean highPower);

#endif /* NEIGHBORSINTERNAL_H_ */
