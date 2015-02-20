/*
 * nbrMsg.c
 *
 *  Created on: Nov 23, 2012
 *      Author: jamesm
 */

#include <string.h>
#include "roneos.h"
#include "neighborsInternal.h"

/******** Defines ********/


/******** Variables ********/
static NbrData* nbrDataStartPtr = NULL;

static uint8 nbrDataIRIdx = 0;
static uint8 nbrDataIRMessageBitCount = 0;

static uint8 nbrDataRFIdx = 0;
//static uint16 nbrDataRFMessageBitCount = 0;

NbrData nbrMsgID;
static NbrMsgRadio nbrRadioMsgNbrData[NBR_DATA_RF_PACKETS_MAX];


/******** Functions ********/

void nbrDataAdd(NbrData* nbrDataPtr, const char* name, uint8 size, uint8 value, uint8 idx, uint8 type){
	NbrData* nbrMsgTempPtr;

	nbrDataPtr->name = name;
	nbrDataPtr->size = size;
	nbrDataPtr->value = value;
	nbrDataPtr->idx = idx;
	nbrDataPtr->type = type;
	nbrDataPtr->nextPtr = NULL;

	if (nbrDataStartPtr == NULL) {
		// if list is empty, store the message at the top of the list
		nbrDataStartPtr = nbrDataPtr;
	} else {
		nbrMsgTempPtr = nbrDataStartPtr;
		while (nbrMsgTempPtr->nextPtr != NULL) {
			nbrMsgTempPtr = nbrMsgTempPtr->nextPtr;
		}
		nbrMsgTempPtr->nextPtr = nbrDataPtr;
	}
}


void nbrDataCreateIR(NbrData* nbrDataPtr, const char* name, uint8 size, uint8 value){
	if (nbrDataIRIdx >= NBR_DATA_IR_IDX_MAX) {
		// too many IR nbrData
		return;
	}
	if ((nbrDataIRMessageBitCount + size) >= NBR_DATA_IR_BITS_MAX) {
		// too many total bits in the resulting IR message
		return;
	}

	// There is enough space.  Create and add the message.
	// The index is used for efficient access for the nbrDataGetNbr() function
//	nbrDataPtr->name = name;
//	nbrDataPtr->size = size;
//	nbrDataPtr->value = value;
//	nbrDataPtr->idx = nbrDataIRCount++;
//	nbrDataPtr->type = NBR_DATA_TYPE_IR;
//	nbrDataPtr->nextPtr = NULL;

	nbrDataAdd(nbrDataPtr, name, size, value, nbrDataIRIdx++, NBR_DATA_TYPE_IR);

	nbrDataIRMessageBitCount+= size;
	irCommsSetSize(nbrDataIRMessageBitCount);
}

void nbrDataRFPacketIndex(uint16 nbrDataRFIdx, uint8* packetPtr, uint8* idxPtr) {
	*idxPtr = nbrDataRFIdx % NBR_DATA_RF_IDX_PER_PACKET;
	*packetPtr = nbrDataRFIdx / NBR_DATA_RF_IDX_PER_PACKET;
}

void nbrDataCreate(NbrData* nbrDataPtr, const char* name, uint8 size, uint8 value){
	uint8 packet, idx;

	if (nbrDataRFIdx >= NBR_DATA_RF_IDX_MAX) {
		// too many nbrMsgs
		error("too many nbr data radio bytes");
		return;
	}
	//TODO use this line when we bit-pack data
	//if ((nbrDataRFMessageBitCount + size) >= NBR_DATA_RF_BITS_MAX) {
//		// too many total bits in the resulting RF message.  exit
//		error("too many nbr data radio bits");
//		return;
//	}

	// There is enough space.  Create and add the message.
	nbrDataAdd(nbrDataPtr, name, size, value, nbrDataRFIdx, NBR_DATA_TYPE_RF);

	//TODO enable this when we are bit-packing RF messages.  for now, just add 8 bits to the total number of bits
	//nbrDataRFMessageBitCount+= size;
	nbrDataRFPacketIndex(nbrDataRFIdx, &packet, &idx);
	if (idx == 0) {
		// we need to init a new nbr radio message.  we do this dynamically, so that it will
		// not be sent if it does not need to be sent
		nbrMsgRadioCreate(&nbrRadioMsgNbrData[packet], "nbrData");
	}
	nbrDataRFIdx++;
}

void nbrDataCreate16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL, const char* nameH, const char* nameL, uint16 value){
	nbrDataCreate(nbrDataPtrH, nameH, 8, (uint8)(value>>8));
	nbrDataCreate(nbrDataPtrL, nameL, 8, (uint8)(value));
}

void nbrDataCreate32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL,
	const char* nameHH, const char* nameHL, const char* nameLH, const char* nameLL, uint32 value) {
	nbrDataCreate16(nbrDataPtrHH, nbrDataPtrHL, nameHH, nameHL, (uint16)(value>>16));
	nbrDataCreate16(nbrDataPtrLH, nbrDataPtrLL, nameLH, nameLL, (uint16)(value));
}


void nbrDataPrintNbr(Nbr* nbrPtr){
	uint8 val;
	if(nbrPtr) {
		NbrData* nbrDataPtr = nbrDataStartPtr;
		while (nbrDataPtr != NULL) {
			val = nbrDataGetNbr(nbrDataPtr, nbrPtr);
			cprintf("%d", val);
			if (nbrDataPtr->nextPtr) {
				cprintf(",");
			}
			nbrDataPtr = nbrDataPtr->nextPtr;
		}
	}
}


void nbrDataPrintNbrVerbose(Nbr* nbrPtr){
	if(nbrPtr) {
		NbrData* nbrDataPtr = nbrDataStartPtr;
		while (nbrDataPtr != NULL) {
			cprintf("  %d:%s=%d\n", nbrDataPtr->idx, nbrDataGetName(nbrDataPtr), nbrDataGetNbr(nbrDataPtr, nbrPtr));
			nbrDataPtr = nbrDataPtr->nextPtr;
		}
	}
}


void nbrDataPrintHeaders(void){
	NbrData* nbrMsgPtr = nbrDataStartPtr;
	while (nbrMsgPtr != NULL) {
		cprintf("%s,%d", nbrMsgPtr->name, nbrMsgPtr->size);
		if (nbrMsgPtr->nextPtr) {
			cprintf(" ");
		}
		nbrMsgPtr = nbrMsgPtr->nextPtr;
	}
}


uint8 nbrDataCount(void){
	return nbrDataIRIdx + nbrDataRFIdx;
}


void nbrDataSet(NbrData* nbrDataPtr, uint8 value){
	if (nbrDataPtr) {
		uint8 mask = 0xFF >> (8 - nbrDataPtr->size);
		nbrDataPtr->value = (value & mask);
	}
}

void nbrDataSet16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL, uint16 value){
	nbrDataSet(nbrDataPtrH, (uint8)(value>>8));
	nbrDataSet(nbrDataPtrL, (uint8)(value));
}

void nbrDataSet32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL,
	uint32 value){
	nbrDataSet16(nbrDataPtrHH, nbrDataPtrHL, (uint16)(value>>16));
	nbrDataSet16(nbrDataPtrLH, nbrDataPtrLL, (uint16)(value));
}


uint8 nbrDataGet(NbrData* nbrDataPtr){
	if (nbrDataPtr) {
		return(nbrDataPtr->value);
	} else {
		return 0;
	}
}

uint16 nbrDataGet16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL){
	return ((uint16)(nbrDataGet(nbrDataPtrH))<<8)+nbrDataGet(nbrDataPtrL);
}

uint32 nbrDataGet32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL){
	return ((uint32)(nbrDataGet16(nbrDataPtrHH, nbrDataPtrHL))<<16)+nbrDataGet16(nbrDataPtrLH, nbrDataPtrLL);
}

/*
 * @brief Get value of the local nbrData
 *
 * @param nbrDataPtr pointer to neighbor message
 * @param nbrPtr neighbor pointer
 * @returns if nbrPtr and nbrMsgPtr are valid, value; else, 0
 */
uint8 nbrDataGetNbr(NbrData* nbrDataPtr, Nbr* nbrPtr){
	uint8 val = 0;
	if (nbrPtr && nbrDataPtr) {
		if (nbrDataPtr->type == NBR_DATA_TYPE_IR) {
			val = nbrPtr->messages[nbrDataPtr->idx];
		}
		else if (nbrDataPtr->type == NBR_DATA_TYPE_RF) {
			NbrMsgRadioNbrData* nbrMsgPtr;
			uint8 packet, idx;
			nbrDataRFPacketIndex(nbrDataPtr->idx, &packet, &idx);
			nbrMsgPtr = nbrMsgRadioGetNbr(&nbrRadioMsgNbrData[packet], nbrPtr);
			val = nbrMsgPtr->data[idx];
		}
	} else {
		val = 0;
	}
	return val;
}

uint16 nbrDataGetNbr16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL, Nbr* nbrPtr){
	return ((uint16)(nbrDataGetNbr(nbrDataPtrH, nbrPtr))<<8)+nbrDataGetNbr(nbrDataPtrL, nbrPtr);
}

uint32 nbrDataGetNbr32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL, Nbr* nbrPtr){
	return ((uint32)(nbrDataGetNbr16(nbrDataPtrHH, nbrDataPtrHL, nbrPtr))<<16)+nbrDataGetNbr16(nbrDataPtrLH, nbrDataPtrLL, nbrPtr);
}

const char* nbrDataGetName(NbrData* nbrMsgPtr){
	if (nbrMsgPtr) {
		return(nbrMsgPtr->name);
	} else {
		return NULL;
	}
}


uint8 nbrDataGetSize(NbrData* nbrMsgPtr){
	if (nbrMsgPtr) {
		return(nbrMsgPtr->size);
	} else {
		return 0;
	}
}


// special case the unpacking of the ID
uint8 neighborMessageUnpackID(uint8* message){
	uint8 val;
	if (nbrMsgID.idx != 0) {
		//something very bad has happened.  the ID should always be the first nbr message
		//error();
		return 0;
	}
	// get the first byte of the message. mask out the number of bits in the ID
	uint8 mask = 0xFF >> (8 - nbrMsgID.size);
	val = message[0] & mask;
	return val;
}


void nbrDataIRMessagePack(IRCommsMessage *irMessage){
	uint8 i, byte0, byte1, shifts, size, mask;
	uint8 bitIdx = 0;
	uint16 msgWord;
	// add an additional byte to the message data to let the 16-bit word overflow safely
	uint8 msgTempIR[IR_COMMS_MESSAGE_LENGTH_MAX + 1];
	NbrData* nbrDataPtr = nbrDataStartPtr;
	//char msgTempRadio[NBR_RADIO_MESSAGE_DATA_LENGTH];

	// setup the IR nbr Data in the IR message
	// clear the packed IR message so we can or in new bits
	for (i = 0; i < IR_COMMS_MESSAGE_LENGTH_MAX + 1; i++) {
		msgTempIR[i] = 0;
	}

	while (nbrDataPtr != NULL) {
		if (nbrDataPtr->type == NBR_DATA_TYPE_IR) {
			// store the IR data in the IR message
			size = nbrDataPtr->size;

			// mask unneeded bits
			mask = 0xFF >> (8 - size);
			msgWord = (uint16)(nbrDataPtr->value & mask);
			// shift left to over to merge.  Tricky.  Don't forget about endianness...
			//shifts = (8 - size) + (8 - (bitIdx & 0x07));
			//msgWord <<= shifts;
			//byte0 = (uint8)(msgWord >> 8);
			//byte1 = (uint8)(msgWord & 0xFF);
			//msgTempIR[bitIdx >> 3] |= byte0;
			//msgTempIR[(bitIdx >> 3) + 1] |= byte1;
			shifts = (bitIdx & 0x07);
			msgWord <<= shifts;
			byte0 = (uint8)(msgWord & 0xFF);
			byte1 = (uint8)(msgWord >> 8);
			msgTempIR[bitIdx >> 3] |= byte0;
			msgTempIR[(bitIdx >> 3) + 1] |= byte1;
			bitIdx += size;
		} else if (nbrDataPtr->type == NBR_DATA_TYPE_RF) {
			// store the RF nbr Data in the RF message
			//msgTempRadio[nbrDataPtr->idx] = nbrDataPtr->value;
			uint8 packet, idx;
			nbrDataRFPacketIndex(nbrDataPtr->idx, &packet, &idx);
			nbrRadioMsgNbrData[packet].data[idx] = nbrDataPtr->value;
			// set the length to be the index + 1. we assume the packets were added in order,
			// so the indices will always increase, and each packet will be at max except for the last one
			nbrRadioMsgNbrData[packet].length = idx + 1;
		}
		nbrDataPtr = nbrDataPtr->nextPtr;
	}
	//memcpy(message, messageTemp, NEIGHBOR_MESSAGE_LENGTH_WITH_ID); /* Should be the same as MAX_IR_MSG_LEN. */


	// Set the data (packed above), the size (computed above) of the new IR message.
	memcpy(irMessage->data, msgTempIR, IR_COMMS_MESSAGE_LENGTH_MAX);
	//nbrMsgRadioSet(&nbrRadioMsgNbrData, msgTempRadio, nbrDataRFIdx);
}


void nbrDataIRMessageUnpack(IRCommsMessage *irMessage, Nbr* nbrPtr){
	uint8 byte0, byte1, shifts, size, mask;
	uint8 bitIdx = 0;
	uint16 msgWord;
	// add an additional byte to the message data to let the 16-bit word overflow safely
	uint8 messageTemp[IR_COMMS_MESSAGE_LENGTH_MAX + 1];
	NbrData* nbrDataPtr = nbrDataStartPtr;

	memcpy(messageTemp, irMessage->data, IR_COMMS_MESSAGE_LENGTH_MAX);
	messageTemp[IR_COMMS_MESSAGE_LENGTH_MAX] = 0;

	while (nbrDataPtr != NULL) {
		if (nbrDataPtr->type == NBR_DATA_TYPE_IR) {
//			size = nbrDataPtr->size;
//			shifts = (8 - size) + (8 - (bitIdx & 0x07));
//			msgWord = ((uint16)(messageTemp[bitIdx >> 3]) << 8) | (uint16)(messageTemp[(bitIdx >> 3) + 1]);
//			msgWord >>= shifts;
//			mask = 0xFF >> (8 - size);
//			nbrPtr->messages[nbrDataPtr->idx] = ((uint8)msgWord) & mask;
			size = nbrDataPtr->size;
			shifts = (bitIdx & 0x07);
			msgWord = ((uint16)(messageTemp[bitIdx >> 3])) | ((uint16)(messageTemp[(bitIdx >> 3) + 1])  << 8);
			msgWord >>= shifts;
			mask = 0xFF >> (8 - size);
			nbrPtr->messages[nbrDataPtr->idx] = ((uint8)msgWord) & mask;
			bitIdx += size;
		} else if (nbrDataPtr->type == NBR_DATA_TYPE_RF) {
			// the RF data can stay in the RF message.  we pull it out when the user calls the getter
		}
		nbrDataPtr = nbrDataPtr->nextPtr;
	}
}
