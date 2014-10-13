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

RadioCmd consensusRadioCmdAtoB;
RadioCmd consensusRadioCmdBtoA;

RadioMessage consensusRadioMessageData[CONSENSUS_MAX_MESSAGES];

uint8 consensusNumMessages;

void *consensusData;
uint16 consensusDataSize;

char consensusReadData[CONSENSUS_MAX_DATA_SIZE];

void (*consensusPreOp)(void *);
void (*consensusPostOp)(void *);
void (*consensusOp)(void *, void *);

void consensusNoOp(void *a, void *b) {
	// No-Op
}

void consensusInitData(void *data, uint16 size) {
	if (size <= CONSENSUS_MAX_DATA_SIZE) {
		error("Consensus data too large!");
	}

	consensusData = data;
	consensusDataSize = size;

	consensusNumMessages = (size + RADIO_COMMAND_MESSAGE_DATA_LENGTH
			- (size % RADIO_COMMAND_MESSAGE_DATA_LENGTH))
			/ RADIO_COMMAND_MESSAGE_DATA_LENGTH;

	radioCommandAddQueue(&consensusRadioCmdAtoB, "consensusAtoB", CONSENSUS_MAX_MESSAGES);
	radioCommandAddQueue(&consensusRadioCmdBtoA, "consensusBtoA", CONSENSUS_MAX_MESSAGES);
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

void consensusBuildMessage() {
	uint8 i, j;

	for (i = 0; i < consensusNumMessages; i++) {
		uint16 dataIndex;
		for (j = 0; j < RADIO_COMMAND_MESSAGE_DATA_LENGTH && (dataIndex = (RADIO_COMMAND_MESSAGE_DATA_LENGTH * i) + j) < consensusDataSize; j++) {
			consensusRadioMessageData[i].command.data[j] = consensusData + dataIndex;
		}
	}
}

void consensusParseMessage() {
	uint8 i, j;

	for (i = 0; i < consensusNumMessages; i++) {
		uint16 dataIndex;
		for (j = 0; j < RADIO_COMMAND_MESSAGE_DATA_LENGTH && (dataIndex = (RADIO_COMMAND_MESSAGE_DATA_LENGTH * i) + j) < consensusDataSize; j++) {
			consensusReadData + dataIndex = consensusRadioMessageData[i].command.data[j];
		}
	}
}

void consensusSendAck(uint8 destID) {
	consensusAckMessage *cAck = &consensusRadioMessageData[0].command.data;
	cAck->ackCheck = CONSENSUS_ACK;

	radioCommandXmit(&consensusRadioCmdAtoB, destID, &consensusRadioMessageData[0]);
}


void consensusAckMode() {
	// Random wait to offset to prevent synchronous transmissions
	osTaskDelay(rand() % (consensusPeriod / 2));

	// Choose a random neighbor
	NbrList nbrList;
	neighborsGetMutex();
	nbrListCreate(&nbrList);
	neighborsPutMutex();

	uint8 destID = nbrListGetNbr(&nbrList, rand() % nbrList.size)->ID;

	// Send ack to neighbor
	consensusSendAck(destID);

	// Wait for data from neighbor
	uint8 i = 0;
	while (radioCommandReceive(&consensusRadioCmdBtoA, &consensusRadioMessageData[i], CONSENSUS_RESPONSE_TIMEOUT)) {
		if (i == consensusNumMessages) {
			break;
		}
		i++;
	}
	consensusParseMessage();

	// Send your data
	consensusBuildMessage();
	for (i = 0; i < consensusNumMessages; i++) {
		radioCommandXmit(&consensusRadioCmdAtoB, destID, &consensusRadioMessageData[i]);
		osTaskDelay(CONSENSUS_INTER_MESSAGE_DELAY);
	}

	// Do consensus if success
	consensusOp(consensusData, consensusReadData);
}

void consensusIdleMode() {
	// Wait for ack for w/e time

	// If ack, then transmit your data

	// Then wait for data back, then consensus
}

void consensusTask() {
	consensusWakeTime = osTaskGetTickCount();
	for (;;) {
		consensusPreOp(consensusData);

		// Decide whether to attempt to connect or wait for a connection
		if ((rand() % CONSENSUS_RAND_MOD) < consensusAckChance) {
			consensusAckMode();
		} else {
			consensusIdleMode();
		}

		consensusPostOp(consensusData);

		osTaskDelayUntil(&consensusWakeTime, consensusPeriod);
	}
}

void consensusInit(void *data, uint16 size) {
	consensusInitData(data, size);

	consensusPreOp = (void (*)(void *))&consensusNoOp;
	consensusPostOp =(void (*)(void *)) &consensusNoOp;
	consensusOp = (void (*)(void *, void *))&consensusNoOp;

	osTaskCreate(consensusTask, "consensus", 1024, NULL, CONSENSUS_TASK_PRIORITY);
}
