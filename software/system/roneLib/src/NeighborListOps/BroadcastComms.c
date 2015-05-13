/*
 * @file BroadcastComms.c
 *
 *  @brief Multi-hop broadcast message communications library.
 *  Multi-hop broadcast messages are used to communicate over long distances in the system.
 *  After creation, the message is INACTIVE, and no data is transmitted.  Once at least one robot
 *  becomes the source of a message by using the broadcastMsgSetSource() function.  This robot
 *  will have a message hops of 0, and will source the data that will be transmitted throughout the
 *  network.
 *
 *  @since Apr 6, 2011
 *
 *  @author Golnaz Habibi
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

/*
 * @brief Creates a multi-hop broadcast message.
 * The msgPtr argument is the pointer to the data structure that stores the broadcast message data.
 * this is not a user-editable structure, use the getters and setters to access the broadcast
 * message data.
 *
 * @param msgPtr pointer for broadcast message.
 * @param maxHops the maximum number of hops that this message can propagate
 * @returns void
 */
void broadcastMsgCreate(BroadcastMessage* msgPtr, uint8 maxHops) {
	nbrDataCreate(&msgPtr->msgSourceID, "sourceID", ROBOT_ID_NUM_BITS, ROBOT_ID_NULL);
	nbrDataCreate(&msgPtr->msgHops, "hops", 5, 0);
	nbrDataCreate(&msgPtr->msgTimestamp, "timestamp", 5, 0);
	msgPtr->inactiveTimeOut = 0;
	msgPtr->hopsMax = maxHops;
	msgPtr->timeToUpdateRound = 0;
	broadcastMsgClear(msgPtr);
}


/*
 * @brief Updates neighbor data with data from the sender of the input message.
 *
 * @param msgPtr pointer for the broadcast message
 * @param nbrDataPtr pointer for the neighbor data to be updated
 * @returns void
 */
void broadcastMsgUpdateNbrData(BroadcastMessage* msgPtr, NbrData* nbrDataPtr) {
	Nbr* nbrPtr;
	if (msgPtr && nbrDataPtr) {
		if (!broadcastMsgIsSource(msgPtr)) {
			nbrPtr = nbrsGetWithID(broadcastMsgGetSenderID(msgPtr));
			if(nbrPtr) {
				nbrDataSet(nbrDataPtr, nbrDataGetNbr(nbrDataPtr, nbrPtr));
			}
		}
	}
}

/*
 * @brief Resets the input broadcast message.
 *
 * @param msgPtr pointer to broadcast message to clear
 * @returns void
 */
void broadcastMsgClear(BroadcastMessage* msgPtr) {
	msgPtr->senderID = ROBOT_ID_NULL;
	nbrDataSet(&msgPtr->msgSourceID, ROBOT_ID_NULL);
	nbrDataSet(&msgPtr->msgHops, BROADCAST_MSG_HOPS_MAX);
	nbrDataSet(&msgPtr->msgTimestamp, 0);
	msgPtr->active = FALSE;
	msgPtr->isSource = FALSE;
}


/*
 * @brief Sets the isSource property of the input message to val.
 *
 * @param msgPtr pointer to broadcast message
 * @param val boolean to set source to.
 * @returns void
 */
void broadcastMsgSetSource(BroadcastMessage* msgPtr, boolean val) {
	if(val) {
		msgPtr->isSource = TRUE;
	} else {
		msgPtr->isSource = FALSE;
	}
}


/*
 * @brief Sets the isSource property of the input message to val.
 *
 * @param msgPtr pointer to broadcast source data
 * @param val boolean to set source to.
 * @returns void
 */



/*
 * @brief Gets the number of message hops for the given broadcast message.
 *
 * @param msgPtr pointer for broadcast message
 * @returns the number of message hops
 */
uint8 broadcastMsgGetHops(BroadcastMessage* msgPtr) {
	return nbrDataGet(&msgPtr->msgHops);
}

/*
 * @brief Gets the number of message hops for the given neighbor.
 *
 * @param msgPtr pointer for broadcast message
 * @param nbrPtr neighbor
 * @returns the number of message hops for the given neighbor
 */
uint8 broadcastMsgGetHopsNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr) {
	return nbrDataGetNbr(&msgPtr->msgHops, nbrPtr);
}


/*
 *  @brief Gets the timestamp of the input message
 *
 *  @param msgPtr message
 *  @returns the timestamp of the message
 */
uint8 broadcastMsgGetTimestamp(BroadcastMessage* msgPtr) {
	return nbrDataGet(&msgPtr->msgTimestamp);
}


/*
 *  @brief Gets the current timestamp of the neighbor.
 *
 *  @param msgPtr pointer for the message
 *  @param nbrPtr pointer for the neighbor
 *  @returns the timestamp of the neighbor
 */
uint8 broadcastMsgGetTimestampNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr) {
	return nbrDataGetNbr(&msgPtr->msgTimestamp, nbrPtr);
}


/*
 *  @brief Gets the sender ID for the input message.
 *
 *  @param msgPtr pointer for the message
 *  @returns sender ID for the input message
 */
uint8 broadcastMsgGetSenderID(BroadcastMessage* msgPtr) {
	return msgPtr->senderID;
}
/*
 *  @brief
 *
 *  @param msgPtr
 *  @param nbrPtr
 *  @returns ID of sender nbr.
 */
uint8 broadcastMsgGetSenderIDNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr) {
	return (nbrPtr->ID);
}

/*
 * @brief Gets the source ID of the input broadcast message.
 *
 * @param msgPtr pointer for the broadcast message
 * @returns source ID
 */
uint8 broadcastMsgGetSourceID(BroadcastMessage* msgPtr) {
	//return msgPtr->sourceID;
	return nbrDataGet(&msgPtr->msgSourceID);
}



/*
 *  @brief Gets the source ID of a neighbor.
 *
 *  @param msgPtr pointer for the message
 *  @param nbrPtr pointer for the neighbor
 *  @returns The source ID of a neighbor
 */
uint8 broadcastMsgGetSourceIDNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr) {
	//return (nbrPtr->message[BROADCAST_MSG_SOURCEID_IDX]);
	return nbrDataGetNbr(&msgPtr->msgSourceID, nbrPtr);
}



boolean broadcastMsgIsSource(BroadcastMessage* msgPtr) {
	if (broadcastMsgIsActive(msgPtr)) {
		return (nbrDataGet(&msgPtr->msgSourceID) == roneID ? TRUE : FALSE);
	} else {
		return FALSE;
	}
}


boolean broadcastMsgIsActive(BroadcastMessage* msgPtr) {
	return (msgPtr->active);
}


//#define BROADCAST_MSG_HOPS_MASK				0x0F
//#define BROADCAST_MSG_HOPS_SHIFT			0
//#define BROADCAST_MSG_CHILDRENCOUNT_MASK	0x70
//#define BROADCAST_MSG_CHILDRENCOUNT_SHIFT	4
//#define BROADCAST_MSG_STATIONARY_MASK		0x80
//#define BROADCAST_MSG_STATIONARY_SHIFT		7
//
//
//void broadcastMsgSetHops(uint8 hops) {
//	uint8 data = neighborsGetMessageData(BROADCAST_MSG_HOPS_IDX);
//	data = (data & ~BROADCAST_MSG_HOPS_MASK) | ((hops & BROADCAST_MSG_HOPS_MASK)<< BROADCAST_MSG_HOPS_SHIFT);
//	neighborsSetMessageData(data, BROADCAST_MSG_HOPS_IDX);
//}
//
//void broadcastMsgSetChildCount(uint8 childrenCount) {
//	uint8 data = neighborsGetMessageData(BROADCAST_MSG_HOPS_IDX);
//	data = (data & ~BROADCAST_MSG_CHILDRENCOUNT_MASK) | ((childrenCount & BROADCAST_MSG_CHILDRENCOUNT_MASK)<< BROADCAST_MSG_CHILDRENCOUNT_SHIFT);
//	neighborsSetMessageData(data, BROADCAST_MSG_HOPS_IDX);
//}
//
//uint8 broadcastMsgGetChildCountNbr(Nbr* nbrPtr) {
//	//return (nbrPtr->message[BROADCAST_MSG_HOPS_IDX] >> 4);
//	return (nbrPtr->message[BROADCAST_MSG_HOPS_IDX] & BROADCAST_MSG_CHILDRENCOUNT_MASK) >> BROADCAST_MSG_CHILDRENCOUNT_SHIFT;
//}
//
//void broadcastMsgSetStationary(uint8 stationary) {
//	if(stationary) { stationary = 1; }
//	uint8 data = neighborsGetMessageData(BROADCAST_MSG_HOPS_IDX);
//	data = (data & ~BROADCAST_MSG_STATIONARY_MASK) | ((stationary & BROADCAST_MSG_STATIONARY_MASK)<< BROADCAST_MSG_STATIONARY_SHIFT);
//	neighborsSetMessageData(data, BROADCAST_MSG_HOPS_IDX);
//}
//
//uint8 broadcastMsgGetStationaryNbr(Nbr* nbrPtr) {
//	return (nbrPtr->message[BROADCAST_MSG_HOPS_IDX] & BROADCAST_MSG_STATIONARY_MASK) >> BROADCAST_MSG_STATIONARY_SHIFT;
//}
//
//void broadcastMsgSetParentID(uint8 parentID) {
//	neighborsSetMessageData(parentID, BROADCAST_PARENTID_IDX);
//}
//
//uint8 broadcastMsgGetParentIDNbr(Nbr* nbrPtr) {
//	return (nbrPtr->message[BROADCAST_PARENTID_IDX]);
//}
//
//
//void broadcastMsgSetSourceID(uint8 sourceID) {
//	neighborsSetMessageData(sourceID, BROADCAST_MSG_SOURCEID_IDX);
//}

//void broadcastMsgSetData(uint8 sourceID, uint8 hops, uint8 childrenCount, uint8 stationary, uint8 parentID) {
//	char nbrMessage[NEIGHBOR_MESSAGE_LENGTH];
//
//	nbrMessage[BROADCAST_MSG_SOURCEID_IDX] = sourceID;
//	//nbrMessage[BROADCAST_MSG_HOPS_IDX] = (hops & 0x0F) | (childrenCount << 4);
//	//nbrMessage[BROADCAST_MSG_TIME_IDX] = msgPtr->timeStamp;
//	broadcastMsgSetHops(hops);
//	broadcastMsgSetChildCount(childrenCount);
//	broadcastMsgSetStationary(stationary);
//	nbrMessage[BROADCAST_PARENTID_IDX] = parentID;
//	neighborsSetMessage(nbrMessage);
//}

/*
 *  @brief Updates the broadcast messages of all neighbors in the neighbor list.
 *
 *  \internal
 *  TODO: make brief more specific
 *  If the input message is the source, the neighbors' broadcast message will be updated to match the input.
 *  Otherwise ?
 *  \endinternal
 *  @param msgPtr broadcast message of the robot
 *  @param nbrListPtr list of neighbors to update
 *  @returns void
 */
void broadcastMsgUpdate(BroadcastMessage* msgPtr, NbrList* nbrListPtr) {
	uint8 i, hopsNbr, sourceIDNbr, timestampNbr;
	Nbr* nbrPtr;

	boolean updateNow = neighborsNewRoundCheck(&msgPtr->timeToUpdateRound);

	if (!updateNow) {
		return;
	}

	if (msgPtr->isSource) {
		// you are the source of this message.  transmit sourcey stuff
		msgPtr->senderID = roneID;
		nbrDataSet(&msgPtr->msgSourceID, roneID);
		nbrDataSet(&msgPtr->msgHops, 0);
		nbrDataSet(&msgPtr->msgTimestamp, nbrDataGet(&msgPtr->msgTimestamp) + 1);
		msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
		msgPtr->active = TRUE;
		//cprintf("timestamp=%d\n", nbrDataGet(&msgPtr->msgTimestamp));
	} else {
		// Update the timeout at each round
		if (msgPtr->inactiveTimeOut > 0){
			msgPtr->inactiveTimeOut--;
			if (msgPtr->inactiveTimeOut == 0) {
				msgPtr->active = FALSE;
			}
		}

		//TODO this is incorrect - we need to carry some of this state forward to deal with timestamps
		//broadcastMsgClear(msgPtr);

		for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
			nbrPtr = nbrListGetNbr(nbrListPtr, i);
			hopsNbr = broadcastMsgGetHopsNbr(msgPtr, nbrPtr);
			sourceIDNbr = broadcastMsgGetSourceIDNbr(msgPtr, nbrPtr);
			timestampNbr = broadcastMsgGetTimestampNbr(msgPtr, nbrPtr);
			if(sourceIDNbr == ROBOT_ID_NULL) {
				continue;
			}

			if (hopsNbr < broadcastMsgGetHops(msgPtr)) {

				// TODO: deal with the source change
				// if the source is different, then keep the message for now
				if(timestampNbr != broadcastMsgGetTimestamp(msgPtr)) {
					//cprintf("lalalallal lalala\n");
					msgPtr->senderID = broadcastMsgGetSenderIDNbr(msgPtr, nbrPtr);
					nbrDataSet(&msgPtr->msgSourceID, sourceIDNbr);
					nbrDataSet(&msgPtr->msgTimestamp, timestampNbr);
					nbrDataSet(&msgPtr->msgHops, hopsNbr + 1);
					msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
					msgPtr->active = TRUE;
					msgPtr->messageUpdateRound = neighborsGetRound();
				} else {
					hopsNbr = 45;
				}
			}
		}
		//if(nbrDataGet(&msgPtr->msgSourceID) == ROBOT_ID_NULL){
		//   msgPtr->active = FALSE;
		//}
	}
	if (msgPtr->active == TRUE) {
		// we directly set these above now
		//nbrDataSet(&msgPtr->msgSourceID, msgPtr->sourceID);
		//nbrDataSet(&msgPtr->msgHops, msgPtr->hops);
	} else {
		msgPtr->inactiveTimeOut = 0;
		nbrDataSet(&msgPtr->msgSourceID, ROBOT_ID_NULL);
		nbrDataSet(&msgPtr->msgHops,  msgPtr->hopsMax);
	}
}



void broadcastMsgDataCreate(BroadcastMsgData* msgDataPtr, BroadcastMessage* msgPtr, const char * name, uint8 val) {
	msgDataPtr->msgPtr = msgPtr;
	nbrDataCreate(&msgDataPtr->data, name, 8, val);
}


void broadcastMsgDataSet(BroadcastMsgData* msgDataPtr, uint8 val) {
	// if this robot is the source, set the val.  otherwise do nothing
	if(broadcastMsgIsSource(msgDataPtr->msgPtr)) {
		nbrDataSet(&msgDataPtr->data, val);
	}
}


uint8 broadcastMsgDataGet(BroadcastMsgData* msgDataPtr) {
	// return the current best value for the broadcast msg
	// this is the value from the parent in the broadcast tree
	uint8 val = 0;

	if(broadcastMsgIsActive(msgDataPtr->msgPtr)) {
		if(broadcastMsgIsSource(msgDataPtr->msgPtr)) {
			// you are the source.  return your current value
			val = nbrDataGet(&msgDataPtr->data);
		} else {
			// look up the nbdID of the sender of this message
			uint8 senderID = broadcastMsgGetSenderID(msgDataPtr->msgPtr);
			Nbr* nbrPtr;
			//cprintf("sender: %d",senderID);
			// get the nbr pointer for this neighbor
			// use the internal function to avoid craeting a nbrList
			nbrPtr = nbrsGetWithID(senderID);
			if (nbrPtr) {
				// get the data from the sender nbr
				val = nbrDataGetNbr(&msgDataPtr->data, nbrPtr);
				nbrDataSet(&msgDataPtr->data,val);
			}
		}
	}
	return val;
}


///*
// *  @brief Updates the broadcast data of the source
// *
// *
// */
//void broadcastMsgDataUpdate(BroadcastMsgData* msgDataPtr) {
//	uint8 i, hopsNbr, sourceIDNbr, timestampNbr, sourceBearingPrn, sourceDistancePrn;
//	int16 xSource, ySource;
//	Nbr* nbrPtr;
//	NbrList* nbrListParents=nbrListPtr ;
//	//NbrList ListOfParents;
//
//
//	if (msgDataPtr->isSource)
//	{
//		// you are the source of this message.  transmit sourcey stuff
//		nbrDataSet(&msgDataPtr->msgSourceDistance , 0); // set the distance to 0.
//		nbrDataSet(&msgDataPtr->msgSourceBearing , 0); // set the bearing to 0.
//		nbrDataSet(&msgDataPtr->msgSourceMode, sourceMode); // set source mode
//		nbrDataSet(&msgDataPtr->msgSourceID, roneID);
//		msgDataPtr->active = TRUE;
//
//	}
//	else {
//
//
//		nbrListParents = nbrListGetParents(nbrListParents, nbrListPtr, msgPtr);
//		if(nbrListParents == NULL) {
//			msgDataPtr->active = FALSE;
//		}
//		else{
//			nbrPtr = nbrListGetNbr(nbrListParents, 1);
//
//			sourceBearingPrn = broadcastMsgGetSourceBearing(msgDataPtr);
//			sourceDistancePrn = broadcastMsgGetSourceDistance(msgDataPtr);
//			xSource = ((int32)nbrPtr->range) * cosMilliRad((int32)nbrPtr->bearing)  + sourceDistancePrn * sinMilliRad(sourceBearingPrn);
//			ySource = ((int32)nbrPtr->range) * sinMilliRad((int32)nbrPtr->bearing)  + sourceDistancePrn * cosMilliRad(sourceBearingPrn);
//			nbrDataSet(&msgDataPtr->msgSourceID, broadcastMsgGetSourceID(msgPtr));
//			nbrDataSet(&msgDataPtr->msgSourceDistance, xSource + ySource);
//			nbrDataSet(&msgDataPtr->msgSourceBearing, normalizeAngleMilliRad2(atan2MilliRad(ySource,xSource)));
//			nbrDataSet(&msgDataPtr->msgSourceMode, broadcastMsgGetSourceMode(msgDataPtr));
//
//			msgPtr->active = TRUE;
//
//		}
//		if(nbrDataGet(&msgDataPtr->msgSourceID) == ROBOT_ID_NULL){
//			msgDataPtr->active = FALSE;
//		}
//		//
//		else {
//			msgDataPtr->active = FALSE;
//		}
//
//	}
//}

/*
 *  Currently always returns TRUE
 *  TODO Implement or delete?
 */
static boolean timeStampCheck(BroadcastMessage* msgPtr, uint8 timestampMine, uint8 timestampNbr) {

	return TRUE;
}





	/*void broadcastLeaderMsgUpdate(BroadcastMessage* msgPtr, NbrList* nbrListPtr, int32 mode) {
			uint8 i, hopsNbrs, sourceIDNbr, timestampNbr;
			Nbr* nbrPtr;

			boolean updateNow = neighborsNewRoundCheck(&msgPtr->timeToUpdateRound);
			if (!updateNow) {
				return;
			}

			if (msgPtr->isSource) {
				// you are the source of this message.  transmit sourcey stuff
				msgPtr->senderID = roneID;
				nbrDataSet(&msgPtr->msgSourceID, roneID);
				nbrDataSet(&msgPtr->msgHops, hopsNbrs);
				nbrDataSet(&msgPtr->msgTimestamp, nbrDataGet(&msgPtr->msgTimestamp) + 1);
				msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
				msgPtr->active = TRUE;
				//cprintf("timestamp=%d\n", nbrDataGet(&msgPtr->msgTimestamp));
			} else {
				// Update the timeout at each round
				if (msgPtr->inactiveTimeOut > 0){
					msgPtr->inactiveTimeOut--;
					if (msgPtr->inactiveTimeOut == 0) {
						msgPtr->active = FALSE;
					}
				}

				//TODO this is incorrect - we need to carry some of this state forward to deal with timestamps
				//broadcastMsgClear(msgPtr);

				for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
					nbrPtr = nbrListGetNbr(nbrListPtr, i);
					hopsNbrs = broadcastMsgGetHopsNbr(msgPtr, nbrPtr);
					sourceIDNbr = broadcastMsgGetSourceIDNbr(msgPtr, nbrPtr);
					timestampNbr = broadcastMsgGetTimestampNbr(msgPtr, nbrPtr);
					if(sourceIDNbr == ROBOT_ID_NULL) {
						continue;
					}
					if (hopsNbrs < broadcastMsgGetHops(msgPtr)) {
						// TODO: deal with the source change
						// if the source is different, then keep the message for now
						if(timestampNbr != broadcastMsgGetTimestamp(msgPtr)) {
							msgPtr->senderID = broadcastMsgGetSenderIDNbr(msgPtr, nbrPtr);
							nbrDataSet(&msgPtr->msgSourceID, sourceIDNbr);
							nbrDataSet(&msgPtr->msgTimestamp, timestampNbr);
							nbrDataSet(&msgPtr->msgHops, mode);
							msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
							msgPtr->active = TRUE;
							msgPtr->messageUpdateRound = neighborsGetRound();
						} else {
							hopsNbrs = 45;
						}
					}
				}
				//if(nbrDataGet(&msgPtr->msgSourceID) == ROBOT_ID_NULL){
				//   msgPtr->active = FALSE;
				//}
			}
			if (msgPtr->active == TRUE) {
				// we directly set these above now
				//nbrDataSet(&msgPtr->msgSourceID, msgPtr->sourceID);
				//nbrDataSet(&msgPtr->msgHops, msgPtr->hops);
			} else {
				msgPtr->inactiveTimeOut = 0;
				nbrDataSet(&msgPtr->msgSourceID, ROBOT_ID_NULL);
				nbrDataSet(&msgPtr->msgHops,  msgPtr->hopsMax);
			}
		}






	 */






	/*
	 *  @brief
	 *
	 *  @param msgPtr
	 *  @param nbrListPtr
	 *  @returns void
	 */
	void broadcastMsgUpdateLeaderElection(BroadcastMessage* msgPtr, NbrList* nbrListPtr) {
		uint8 i, hopsNbr, sourceIDNbr, timestampNbr;
		Nbr* nbrPtr;

		msgPtr->senderID = roneID;
		nbrDataSet(&msgPtr->msgSourceID, roneID);
		nbrDataSet(&msgPtr->msgHops, 0);
		nbrDataSet(&msgPtr->msgTimestamp, nbrDataGet(&msgPtr->msgTimestamp) + 1);
		msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
		msgPtr->active = TRUE;
		msgPtr->isSource = TRUE;

		for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
			nbrPtr = nbrListGetNbr(nbrListPtr, i);
			hopsNbr = broadcastMsgGetHopsNbr(msgPtr, nbrPtr);
			timestampNbr = broadcastMsgGetTimestampNbr(msgPtr, nbrPtr);
			sourceIDNbr = broadcastMsgGetSourceIDNbr(msgPtr, nbrPtr);
			if (sourceIDNbr == 0) {
				continue;
			}
			if (hopsNbr >= (msgPtr->hopsMax)) {
				continue;
			}
			if (sourceIDNbr > broadcastMsgGetSourceID(msgPtr)) {
				continue;
			}
			if (sourceIDNbr < broadcastMsgGetSourceID(msgPtr)){
				msgPtr->senderID = broadcastMsgGetSenderIDNbr(msgPtr, nbrPtr);
				nbrDataSet(&msgPtr->msgSourceID, sourceIDNbr);
				nbrDataSet(&msgPtr->msgTimestamp, timestampNbr);
				nbrDataSet(&msgPtr->msgHops, hopsNbr + 1);
				msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
				msgPtr->active = TRUE;
				msgPtr->messageUpdateRound = neighborsGetRound();
			} else if(hopsNbr <= broadcastMsgGetHops(msgPtr)) {
				// same source ID, fewer hops.  Check the time stamp
				if (broadcastMsgGetTimestamp(msgPtr) != timestampNbr) {
					msgPtr->senderID = broadcastMsgGetSenderIDNbr(msgPtr, nbrPtr);
					nbrDataSet(&msgPtr->msgSourceID, sourceIDNbr);
					nbrDataSet(&msgPtr->msgTimestamp, timestampNbr);
					nbrDataSet(&msgPtr->msgHops, hopsNbr + 1);
					msgPtr->inactiveTimeOut = BROADCAST_MSG_MAX_TIMEOUT;
					msgPtr->active = TRUE;
					msgPtr->messageUpdateRound = neighborsGetRound();
				} else {
					// If we've stablized into a system in which we see neighbors at lower hops,
					// but with a static time stamp, go ahead and keep it active for a little
					// while, but then eventually force a leader election
					if (msgPtr->inactiveTimeOut > 0) {
						msgPtr->inactiveTimeOut--;
					}
				}
			}
		}
		//	broadcastMsgSetSourceID(msgPtr->sourceID);
		//	broadcastMsgSetHops(msgPtr->hops);
	}

	/*
	 * @brief Prints NbrID and the number of hops taken
	 * @param nbrListPtr -  list of neighbors to updatet
	 * @param broadcastMessagePtr
	 * @param name of node
	 * @returns void
	 */
	void nbrListPrintHops(NbrList* nbrListPtr, BroadcastMessage* broadcastMessagePtr, char* name) {
		uint8 i;

		cprintf("%s={", name);
		for (i = 0; i < nbrListPtr->size; i++) {
			if((nbrListPtr->size > 1) && (i > 0)) {
				cprintf(",");
			}
			cprintf("%02d:%d", nbrListPtr->nbrs[i]->ID, broadcastMsgGetHopsNbr(broadcastMessagePtr, nbrListPtr->nbrs[i]));
		}
		cprintf("}");
	}




	/*
	 * "nbrListParents" is the existing parents list, and "nbrListIn" is the list of neighbors
	 * from which you want to identify the parents.
	 * Put the parents in "nbrListIn" into "nbrListParents".
	 */

	/*
	 *	@brief Puts the parents in nbrListIn into nbrListParents
	 *
	 *	@param	nbrListParents the existing parents list
	 *	@param nbrListIn the list of neighbors
	 *	@param msgPtr the broadcast message
	 *	@returns void
	 */
	NbrList* nbrListGetParents(NbrList* nbrListParentsPtr, NbrList* nbrListInPtr, BroadcastMessage* msgPtr) {
		uint8 i;
		Nbr* nbrPtr;

		nbrListClear(nbrListParentsPtr);
		if (broadcastMsgGetSourceID(msgPtr) != 0) {
			for (i = 0; i < nbrListInPtr->size; i++) {
				nbrPtr = nbrListInPtr->nbrs[i];
				if (broadcastMsgGetSourceIDNbr(msgPtr, nbrPtr) == 0) {
					continue;
				}
				if (broadcastMsgGetHopsNbr(msgPtr, nbrPtr) < broadcastMsgGetHops(msgPtr)) {//&& (broadcastMsgHopsNbr(nbrPtr) != BROADCAST_MSG_HOPS_MAX)) {
					nbrListAddNbr(nbrListParentsPtr, nbrPtr);
				}
			}
		}
		return nbrListParentsPtr;
	}


	/*
	 *  @brief
	 *
	 *  @param nbrListSiblings
	 *  @param nbrListIN
	 *  @param msgPtr
	 *  @returns void
	 */
	NbrList* nbrListGetSiblings(NbrList* nbrListSiblings, NbrList* nbrListIn, BroadcastMessage* msgPtr) {
		uint8 i;
		Nbr* nbrPtr;

		nbrListClear(nbrListSiblings);
		if (broadcastMsgGetSourceID(msgPtr) != 0 ) {
			for (i = 0; i < nbrListIn->size; i++) {
				nbrPtr = nbrListIn->nbrs[i];
				if (broadcastMsgGetSourceIDNbr(msgPtr, nbrPtr) == 0) {
					continue;
				}
				if (broadcastMsgGetHopsNbr(msgPtr, nbrPtr) == broadcastMsgGetHops(msgPtr)) {
					nbrListAddNbr(nbrListSiblings, nbrPtr);
				}
			}
		}
		return nbrListSiblings;
	}

	/*
	 *  @brief
	 *
	 *  @param nbrListChildren
	 *  @param nbrListIn
	 *  @param msgPtr
	 *  @returns void
	 */
	NbrList* nbrListGetChildren(NbrList* nbrListChildren, NbrList* nbrListIn, BroadcastMessage* msgPtr) {
		uint8 i;
		Nbr* nbrPtr;

		nbrListClear(nbrListChildren);
		if (broadcastMsgGetSourceID(msgPtr) != 0 ) {
			for (i = 0; i < nbrListIn->size; i++) {
				nbrPtr = nbrListIn->nbrs[i];
				if (broadcastMsgGetSourceIDNbr(msgPtr, nbrPtr) == 0) {
					continue;
				}
				if (broadcastMsgGetHopsNbr(msgPtr, nbrPtr) > broadcastMsgGetHops(msgPtr)) {
					nbrListAddNbr(nbrListChildren, nbrPtr);
				}
			}
		}
		return nbrListChildren;
	}

	/*
	 * @brief Finds source of Message
	 *	@param nbrListPtr the list of neighbors
	 *	@param msgPtr message to be sent
	 *
	 */
	Nbr* nbrListFindSource(NbrList* nbrListPtr, BroadcastMessage* msgPtr) {
		Nbr* nbrPtr = NULL;
		uint8 i;

		for (i = 0; i < nbrListPtr->size; i++) {
			if (broadcastMsgGetSourceID(msgPtr) == nbrListPtr->nbrs[i]->ID) {
				nbrPtr = nbrListPtr->nbrs[i];
				break;
			}
		}
		return nbrPtr;
	}


	//
	//		if(Parents.size+Siblings.size>=2)
	//		if (Parents.size>=2){
	//	ParentListOut->nbrs={Parents[0],Parents[1]};
	//	}
	//elseif (Parents.size==1 && Siblings>=1 ){
	//
	//	ParentListOut->nbrs={Parents[0],Siblings[0]};
	//}
	//elseif (Siblings.size>1){
	//ParentListOut->nbrs={Siblings[0],Siblings[1]};}
	//else{
	//	ParentListOut->nbrs=Parents;
	//}
	//
	//		elseif (Siblings.size==1)
	//
	//		ParentListOut->nbrs=Siblings;


	//void nbrListSortParents(nbrList* ParentListOut, nbrList* ParentListIn,BroadcastMessage* msgPtr) {
	//	uint8 i;
	//	nbr* parentPtr;
	//	uint8 minhopPtr=PARENT_MSG_HOPS_MAX;
	//	for (i = 0; i < ParentListIn->size; i++) {
	//		parentPtr = ParentListIn->nbrs[i];
	//
	//		if(broadcastMsgHopsNbr(parentPtr) <= minhopPtr ) {
	//			minhopPtr=broadcastMsgHopsNbr(parentPtr);
	//			ParentListOut->nbrs[0]=parentPtr;
	//		}
	//	}
	//		for (i = 0; i < ParentListIn->size; i++) {
	//				parentPtr = ParentListIn->nbrs[i];
	//				if(broadcastMsgHopsNbr(parentPtr) <= minhopPtr && ) {
	//					minhopPtr=broadcastMsgHopsNbr(parentPtr);
	//					ParentListOut->nbrs[1]=parentPtr;
	//		}
	//
	//	}
	//
	//		}


