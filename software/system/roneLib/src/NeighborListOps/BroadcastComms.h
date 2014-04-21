/**
 * @file BroadcastComms.h
 *
 * @brief Multi-hop broadcast message communications library.
 *
 * Multi-hop broadcast messages are used to communicate over long distances in the system.
 * After creation, the message is INACTIVE, and no data is transmitted.  Once at least one robot
 * becomes the source of a message by using the broadcastMsgSetSource() function.  This robot
 * will have a message hops of 0, and will source the data that will be transmitted throughout the
 * network.
 *
 * Call broadcastMsgUpdate() or broadcastMsgUpdateLeaderElection() each neighbor round to update
 * the status of the broadcast message.  In a normal broadcast message,broadcastMsgUpdate() will
 * select the message from the neighbor in nbrList with the fewest number of hops, with a secondary
 * sorting based on the [check this] robotID of the sending robot.
 *
 * broadcastMsgUpdateLeaderElection() selects messages from neighbors first from the robot with
 * the lowest source ID, then secondarily based on the hops from the source.  This means that a
 * message from a source with a lower ID will override a message from a source with a higher ID,
 * ultimately reaching the entire network.  In at most O(diam(G)) time, each robot will learn the
 * ID of the source robot.
 *
 * @since April 6, 2011
 *
 * @author Golnaz Habibi
 *
 *
 */

#include "roneos.h"
#ifndef BROADCASTCOMMS_H_
#define BROADCASTCOMMS_H_

// the maximum number of hops for any broadcast message
#define BROADCAST_MSG_HOPS_MAX			15

// message time to live.  Defined in rounds
#define BROADCAST_MSG_MAX_TIMEOUT		4 

/**
 * @brief Message that is broadcast
 */
typedef struct BroadcastMessage {
	boolean active;				/**<Message active then True, False if message cleared or inactive*/
	uint8 senderID;				/**<Source of message*/
	uint32 messageUpdateRound;	/**<Contains the neighbor round the message was updated*/
	boolean isSource;			/**<True if you are the source otherwise False*/
	uint8 hopsMax;				/**<Maximum number of hops this message can go*/
	uint8 inactiveTimeOut;		/**<Stops sending message after certain of sends*/
	uint32 timeToUpdateRound;	/**<Contains the neighbor round this function was last called*/

	NbrData msgSourceID;		/**<Soucre ID*/
	NbrData msgHops;			/**<Number of hops for message */
	NbrData msgTimestamp;		/**<When message was sent*/
} BroadcastMessage;


/**
 * @brief Data of Source 9mode and bearing and distance to source) that is broadcast
 */

typedef struct BroadcastMsgData {
	BroadcastMessage* msgPtr;	/**<Broadcast message*/
	NbrData data;				/**<Data*/
} BroadcastMsgData;









/**
 * @brief Creates a broadcast message
 *
 * @param msgPtr pointer for broadcast message
 * @param maxHops the maximum number of hops that this message can propagate
 * @returns void
 */
void broadcastMsgCreate(BroadcastMessage* msgPtr, uint8 maxHops);

void broadcastMsgUpdateNav(BroadcastMessage* msgPtr, NbrList* nbrListPtr);
//REMOVE void broadcastSrcDataUpdateNav(BroadcastMsgData* msgPtr, NbrList* nbrListPtr);


/**
 * @brief Clears the broadcast message state
 *
 * @param msgPtr pointer for broadcast message to clear
 * @returns void
 */
void broadcastMsgClear(BroadcastMessage* msgPtr);
//REMOVE void broadcastSrcDataClear(BroadcastMsgData* msgPtr);


/**
 * @brief Sets the isSource property of the input message to val.
 *
 * @param msgPtr pointer to broadcast message
 * @param val source value
 * @returns void
 */
void broadcastMsgSetSource(BroadcastMessage* msgPtr, boolean val);
//REMOVE void broadcastSrcDataSetSource(BroadcastMsgData* msgPtr, boolean val);


boolean broadcastMsgIsSource(BroadcastMessage* msgPtr);
boolean broadcastMsgIsActive(BroadcastMessage* msgPtr);


/**
 * @brief Updates the broadcast messages of this robot.
 * Does not elect a leader, will tessellate the network if there are multiple sources.
 *
 * @param msgPtr broadcast message data structure
 * @param nbrListPtr the list of neighbors whose messages to consider
 * @returns void
 */
void broadcastMsgUpdate(BroadcastMessage* msgPtr, NbrList* nbrListPtr);


/**
 * @brief Updates the broadcast messages of this robot.
 * Does not elect a leader, will not tessellate the network if there are multiple sources.
 *
 * @param msgPtr broadcast message data structure
 * @param nbrListPtr the list of neighbors whose messages to consider
 * @returns void
 */
void broadcastMsgUpdateLeaderElection(BroadcastMessage* msgPtr, NbrList* nbrListPtr);


/**
 * @brief Updates the neighbor data with the neighbor data from the sender of the broadcast message if the message is not the source.
 *
 * @param msgPtr pointer for the broadcast message
 * @param nbrDataPtr pointer for the neighbor data to be updated
 * @returns void
 */
void broadcastMsgUpdateNbrData(BroadcastMessage* msgPtr, NbrData* nbrDataPtr);


/**
 * @brief Gets the number of message hops for the given broadcast message.
 *
 * @param msgPtr pointer for broadcast message
 * @returns the number of message hops
 */
uint8 broadcastMsgGetHops(BroadcastMessage* msgPtr);

/**
 * @brief Gets the number of message hops for the given broadcast message of neighbor.
 *
 * @param msgPtr pointer for broadcast message in neighbor
 * @returns the number of message hops
 */
uint8 broadcastMsgGetHopsNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr);


/**
 * @brief Gets the source ID of the input broadcast message.
 *
 * @param msgPtr pointer for the broadcast message
 * @returns source ID
 */
uint8 broadcastMsgGetSourceID(BroadcastMessage* msgPtr);


/**
 *  @brief Gets the source ID of a neighbor.
 *
 *  @param msgPtr pointer for the message
 *  @param nbrPtr pointer for the neighbor
 *  @returns The source ID of a neighbor
 */
uint8 broadcastMsgGetSourceIDNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr);


/**
 *  @brief Gets the sender ID for the input message.
 *
 *  @param msgPtr pointer for the message
 *  @returns sender ID for the input message
 */
uint8 broadcastMsgGetSenderID(BroadcastMessage* msgPtr);


/**
 *  @brief
 *
 *  @param msgPtr
 *  @param nbrPtr
 *  @returns
 */
uint8 broadcastMsgGetSenderIDNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr);


/**
 *  @brief Gets the current timestamp of the neighbor.
 *
 *  @param msgPtr pointer for the message
 *  @param nbrPtr pointer for the neighbor
 *  @returns the timestamp of the neighbor
 */
uint8 broadcastMsgGetTimestampNbr(BroadcastMessage* msgPtr, Nbr* nbrPtr);


/**
 *  @brief Gets the timestamp of the input message
 *
 *  @param msgPtr message
 *  @returns the timestamp of the message
 */
uint8 broadcastMsgGetTimestamp(BroadcastMessage* msgPtr);



/**
 * @brief Prints NbrID and the number of hops taken
 * @param nbrListPtr -  list of neighbors to updatet
 * @param broadcastMessagePtr
 * @param name of node
 * @returns void
 */
void nbrListPrintHops(NbrList* nbrListPtr, BroadcastMessage* broadcastMessagePtr, char* name);


/**
 *	@brief Puts the parents in nbrListIn into nbrListParents
 *
 *	@param	nbrListParents the existing parents list
 *	@param nbrListIn the list of neighbors
 *	@param msgPtr the broadcast message
 *	@returns void
 */
NbrList* nbrListGetParents(NbrList* nbrListParents, NbrList* nbrListIn, BroadcastMessage* msgPtr);


/**
 *  @brief Gets sibling
 *
 *  @param nbrListSiblings
 *  @param nbrListIn
 *  @param msgPtr
 *  @returns void
 */
NbrList* nbrListGetSiblings(NbrList* nbrListSiblings, NbrList* nbrListIn, BroadcastMessage* msgPtr);


/**
 *  @brief
 *
 *  @param nbrListChildren
 *  @param nbrListIn
 *  @param msgPtr
 *  @returns void
 */
NbrList* nbrListGetChildren(NbrList* nbrListChildren, NbrList* nbrListIn, BroadcastMessage* msgPtr);


/**
 * @brief Finds source of Message
 *	@param nbrListPtr the list of neighbors
 *	@param msgPtr message to be sent
 *  @returns void
 */
Nbr* nbrListFindSource(NbrList* nbrListPtr, BroadcastMessage* msgPtr);



//TODO: Not Connected ton anything
void nbrListPickParents(NbrList* ParentListOut1, NbrList* Parents, NbrList* Siblings,BroadcastMessage* msgPtr);


//TODO: implement or remove?
//void nbrListSortParents(nbrList* ParentListOut, nbrList* ParentListIn,BroadcastMessage* msgPtr);
//void nbrListPrint(NbrList* nbrListPtr, char* name);
//void nbrListPrintDebug(NbrList* nbrListPtr, char* name);

//void broadcastMsgSetHops(uint8 hops);
//uint8 broadcastMsgGetChildCountNbr(Nbr* nbrPtr);
//uint8 broadcastMsgGetParentIDNbr(Nbr* nbrPtr);
//void broadcastMsgSetStationary(uint8 stationary);
//void broadcastMsgSetChildCount(uint8 childrenCount);
//void broadcastMsgSetParentID(uint8 parentID);
//void broadcastMsgSetData(uint8 sourceID, uint8 hops, uint8 childrenCount, uint8 stationary, uint8 parentID);


void broadcastMsgDataCreate(BroadcastMsgData* msgDataPtr, BroadcastMessage* msgPtr, const char * name, uint8 val);
void broadcastMsgDataSet(BroadcastMsgData* msgDataPtr, uint8 val);
uint8 broadcastMsgDataGet(BroadcastMsgData* msgDataPtr);




#endif /* BROADCASTCOMMS_H_ */
