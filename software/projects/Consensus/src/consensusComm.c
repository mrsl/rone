/*
 * consensusComm.c
 *
 *  Created on: Oct 11, 2014
 *      Author: zkk
 */

#include "consensusComm.h"

uint32 consensusPeriod = CONSENSUS_DEFAULT_PERIOD;

uint16 consensusAckChance = CONSENSUS_DEFAULT_ACK_CHANCE;

uint32 consensusWakeTime;
boolean consensusSuccess;

RadioCmd consensusRadioCmdAck;
RadioCmd consensusRadioCmdData;

RadioMessage consensusRadioMessageData;

uint8 consensusNumMessages;

char *consensusData;
uint8 consensusDataSize;

char consensusReadData[CONSENSUS_MAX_DATA_SIZE];

void (*consensusPreOp)(void *) = NULL;
void (*consensusPostOp)(void *) = NULL;
void (*consensusOp)(void *, void *) = NULL;

void consensusPrintBytes(void *p, uint8 l) {
	uint8 i;
	cprintf("(");
	for (i = 0; i < l; ++i) {
		cprintf("%02X", ((unsigned char *)p)[i]);
	}
	cprintf(")");
}

void consensusNoOp(void *a, void *b) {
	// No-Op
}

void consensusInitData(void *data, uint8 size) {
	if (size > CONSENSUS_MAX_DATA_SIZE) {
		error("Consensus data too large!");
	}

	consensusData = data;
	consensusDataSize = size;

	consensusNumMessages = (size + RADIO_COMMAND_MESSAGE_DATA_LENGTH
			- (size % RADIO_COMMAND_MESSAGE_DATA_LENGTH))
			/ RADIO_COMMAND_MESSAGE_DATA_LENGTH;

	radioCommandAddQueue(&consensusRadioCmdAck, "consensusAck", 1);
	radioCommandAddQueue(&consensusRadioCmdData, "consensusData", consensusNumMessages);

	cprintf("Consensus Initialized! Data Size: %d Num Messages: %d\n", consensusDataSize, consensusNumMessages);
}

void consensusInitPreConsensusOperation(void (*func)(void *)) {
	consensusPreOp = func;
}

void consensusInitPostConsensusOperation(void (*func)(void *)) {
	consensusPostOp = func;
}

void consensusInitConsensusOperation(void (*func)(void *, void *)) {
	consensusOp = func;
}

void consensusSetPeriod(uint32 newPeriod) {
	consensusPeriod = newPeriod;
}

void consensusSetAckChance(uint16 newAckChance) {
	consensusAckChance = newAckChance;
}


void consensusClearReadData() {
	uint8 i;

	for (i = 0; i < consensusDataSize; i++) {
		consensusReadData[i] = 0;
	}
}

uint8 consensusGetSizeData() {
	return consensusDataSize;
}

/**
 * Transmits whatever is at the consensusData pointer to a destination ID
 */
void consensusDataTransmit(uint8 destID) {
//	char *bufferPtr = consensusRadioMessageData.command.data;
//	char *tempBufPtr;

	uint8 dataLeft = consensusDataSize;

	//rprintf("	");

	//consensusPrintBytes(consensusData, consensusDataSize);
	//cprintf("	%s\n", consensusData);

	cprintf("	TRANSMITTING %d to %d: ", dataLeft, destID);

	while (dataLeft > 0) {
		char *bufferPtr = consensusRadioMessageData.command.data;
		uint8 i = 0;
		while (i < RADIO_COMMAND_MESSAGE_DATA_LENGTH && i < dataLeft) {
			*bufferPtr++ = consensusData[(consensusDataSize - dataLeft) + i++];
		}

		cprintf("%d ", i);
		radioCommandXmit(&consensusRadioCmdData, destID, &consensusRadioMessageData);

		//consensusPrintBytes(consensusRadioMessageData.command.data, 8);

		dataLeft -= i;
		osTaskDelay(CONSENSUS_INTER_MESSAGE_DELAY);
	}

	cprintf("\n");
}

/**
 * Receives incoming data from other robot, returns 1 if success, 0 if failure
 * Stores data in the consensusReadData buffer
 */
uint8 consensusDataReceive() {
	uint8 dataLeft = consensusDataSize;
	char *dataPtr = consensusReadData;

	//rprintf("	");
	cprintf("	RECEIVE: ");

	while (radioCommandReceive(&consensusRadioCmdData, &consensusRadioMessageData, CONSENSUS_RESPONSE_TIMEOUT)
		   && dataLeft > 0) {
		char *bufferPtr = consensusRadioMessageData.command.data;

		cprintf("C: ");
		uint8 i = 0;
		for (i = 0; i < RADIO_COMMAND_MESSAGE_DATA_LENGTH && i < dataLeft; i++) {
			cprintf("%c", ((char *)bufferPtr)[i]);
		}
		cprintf(" - ");


		i = 0;
		while (i < RADIO_COMMAND_MESSAGE_DATA_LENGTH && i < dataLeft) {
			*dataPtr++ = bufferPtr[i++];
		}
		cprintf("%d ", i);

		//consensusPrintBytes(consensusRadioMessageData.command.data, 8);
		dataLeft -= i;
	}

	//consensusPrintBytes(consensusReadData, consensusDataSize);

	//cprintf("	%s\n", consensusReadData);
	//rprintf("\n");
	cprintf("\n");

	if (dataLeft > 0) {
		return (0);
	}

	return (1);
}

void consensusSendAck(uint8 destID) {
	consensusAckMessage *cAck = (consensusAckMessage *) &consensusRadioMessageData.command.data;
	cAck->ackCheck = CONSENSUS_ACK;
	cAck->id = roneID;

	cprintf("	SENDING ACK\n");

	radioCommandXmit(&consensusRadioCmdAck, destID, &consensusRadioMessageData);
	osTaskDelay(CONSENSUS_INTER_MESSAGE_DELAY);
}

uint8 consensusReceiveAck() {
	// Wait for ack for w/e time
	if (!radioCommandReceive(&consensusRadioCmdAck, &consensusRadioMessageData, consensusPeriod - CONSENSUS_ACK_TIMEOUT_DELTA)) {
		return 0;
	} else {
		// This is not an ack message
		if (((consensusAckMessage *) &consensusRadioMessageData.command.data)->ackCheck != CONSENSUS_ACK) {
			return 0;
		}
	}
	cprintf("	ACK RECEIVED\n");

	// Get ID of partner robot from message
	return ((consensusAckMessage*) &consensusRadioMessageData.command.data)->id;
}


void consensusAckMode() {
	cprintf("	Ack Mode\n");
	// Random wait to offset to prevent synchronous transmissions
	osTaskDelay(rand() % (consensusPeriod / 2));

	// Choose a random neighbor
	NbrList nbrList;
	neighborsGetMutex();
	nbrListCreate(&nbrList);
	neighborsPutMutex();

	if (nbrList.size == 0) {
		cprintf("	No one to ack with!\n");
		return;
	}

	Nbr *nbrPtr = nbrListGetNbr(&nbrList, (((uint8) rand()) % nbrList.size));
	if (nbrPtr == NULL) {
		cprintf("	No one to ack with!\n");
		return;
	}

	uint8 destID = nbrGetID(nbrPtr);

	cprintf("	Acking with robot %d\n", destID);

	// Send ack to neighbor
	consensusSendAck(destID);

	cprintf("	Ack sent.\n");

	if (!consensusDataReceive()) {
		cprintf("	No data received :(\n");
		return;
	}

	cprintf("	Data received! Transmitting...\n");

	consensusDataTransmit(destID);

	cprintf("	Transmitted! Consensus time.\n");

	(*consensusOp)(consensusData, consensusReadData);

}

void consensusIdleMode() {
	cprintf("	Idle Mode\n");

	uint8 fromID;
	// Receive an ack from a neighbor
	if ((fromID = consensusReceiveAck()) == 0) {
		// No ack received
		cprintf("	No Ack received :(\n");
		return;
	}

	cprintf("	Ack from robot %d! Transmitting...\n", fromID);

	consensusDataTransmit(fromID);

	cprintf("	Transmitted! Receiving data...\n");

	if (!consensusDataReceive()) {
		cprintf("	No data received :(\n");
		return;
	}

	cprintf("	Received! Consensus time.\n");

	(*consensusOp)(consensusData, consensusReadData);
}

void consensusTask() {
	for (;;) {
		consensusWakeTime = osTaskGetTickCount();

		consensusSuccess = FALSE;

		cprintf("Consensus Round Begin\n");

		(*consensusPreOp)(consensusData);

		// Decide whether to attempt to connect or wait for a connection
		if ((rand() % CONSENSUS_RAND_MOD) < consensusAckChance) {
			consensusAckMode();
		} else {
			consensusIdleMode();
		}

		(*consensusPostOp)(consensusData);

		consensusClearReadData();

		cprintf("Consensus Round End\n");

		osTaskDelayUntil(&consensusWakeTime, consensusPeriod);
	}
}

void consensusInit(void *data, uint16 size) {
	cprintf("Initializing Consensus\n");

	consensusInitData(data, size);

	// Init to no ops
	if (consensusPreOp == NULL)
		consensusPreOp = (void (*)(void *))&consensusNoOp;
	if (consensusPostOp == NULL)
		consensusPostOp =(void (*)(void *)) &consensusNoOp;
	if (consensusOp == NULL)
		consensusOp = (void (*)(void *, void *))&consensusNoOp;

	osTaskCreate(consensusTask, "consensus", 1536, NULL, CONSENSUS_TASK_PRIORITY);
	cprintf("Consensus Initialized\n");
}
