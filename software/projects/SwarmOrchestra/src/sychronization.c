/*
 * sychronization.c
 *
 *  Created on: Nov 6, 2014
 *      Author: zkk
 */

#include "roneos.h"
#include "roneLib.h"

#define HH_MASK	0xFF000000
#define HL_MASK	0x00FF0000
#define LH_MASK	0x0000FF00
#define LL_MASK	0x000000FF

#define HH_SHIFT	24
#define HL_SHIFT	16
#define LH_SHIFT	8
#define LL_SHIFT	0

BroadcastMessage synchBroadcastMessage;

/* Timestamp broadcast message data */
BroadcastMsgData synchTimeMessageHH;	// High order bits
BroadcastMsgData synchTimeMessageHL;
BroadcastMsgData synchTimeMessageLH;
BroadcastMsgData synchTimeMessageLL;	// Low order bits

/**
 * Callback that happens each neighbor round. Updates the synchronization clock.
 */
static void synchCallback(NbrDatabase* ndPtr) {
	NbrList nbrList;

	nbrListCreate(&nbrList);

	broadcastMsgUpdate(&synchBroadcastMessage, &nbrList);

	cprintf("synch!\n");

	//TODO: do something with the updated broadcast message
	uint8 tsHH, tsHL, tsLH, tsLL;
	if (broadcastMsgIsSource(&synchBroadcastMessage)) {
		/* Get the ticks and split them */
		uint32 transmitTime = osTaskGetTickCount();
		tsHH = (uint8) ((transmitTime & HH_MASK) >> HH_SHIFT);
		tsHL = (uint8) ((transmitTime & HL_MASK) >> HL_SHIFT);
		tsLH = (uint8) ((transmitTime & LH_MASK) >> LH_SHIFT);
		tsLL = (uint8) ((transmitTime & LL_MASK) >> LL_SHIFT);

		broadcastMsgDataSet(&synchTimeMessageHH, tsHH);
		broadcastMsgDataSet(&synchTimeMessageHL, tsHL);
		broadcastMsgDataSet(&synchTimeMessageLH, tsLH);
		broadcastMsgDataSet(&synchTimeMessageLL, tsLL);

		cprintf("%u\n", transmitTime);
	} else {
		tsHH = broadcastMsgDataGet(&synchTimeMessageHH);
		tsHL = broadcastMsgDataGet(&synchTimeMessageHL);
		tsLH = broadcastMsgDataGet(&synchTimeMessageLH);
		tsLL = broadcastMsgDataGet(&synchTimeMessageLL);

		uint32 receiveTime = tsHH << HH_SHIFT
							| tsHL << HL_SHIFT
							| tsLH << LH_SHIFT
							| tsLL << LL_SHIFT;

		cprintf("%u\n", receiveTime);
	}
}

/**
 * Initialize the synchronization subsystem.
 * TODO:
 * Should automatically elect a conductor robot that is the clock synchronizer.
 * This clock robot then propogates its time through a tree
 */
void synchInit(void) {
	neighborsAddReceiveCallback(synchCallback);
}