/**
 * @file BroadcastMinMax.h
 * @brief Min-Max spanning tree between robots
 * @details
 * @since April 10, 2013
 * @author James McLurkin
 */

#ifndef BROADCASTMINMAX_H_
#define BROADCASTMINMAX_H_

/**
 * @brief Broadcast Min-Max Message
 * @details Broadcast message that goes through tree
 */
typedef struct BroadcastMinMaxMessage {
	boolean hasBeenSet;					/**< True if message is sent, else False*/
	NbrData val;						/**< Data on Neighbor*/
	NbrData valLocal;					/**< Data on Self*/
} BroadcastMinMaxMessage;


/**
 * @brief creates a min/max broadcast message in msgPtr
 *
 * @param msgPtr
 * @param name
 * @param val - value to be created
 * @returns void
 */
void broadcastMinMaxCreate(BroadcastMinMaxMessage* msgPtr, char* name, uint8 val);


/**
 * @brief
 *
 * @param msgPtr
 * @param val - value to be set
 * @returns void
 */
void broadcastMinMaxSet(BroadcastMinMaxMessage* msgPtr, uint8 val);


/**
 * @brief
 *
 * @param msgPtr
 * @param nbrListPtr
 * @returns void
 */
void broadcastMinMaxSetMaxPlusOne(BroadcastMinMaxMessage* msgPtr, NbrList* nbrListPtr);


/**
 * @brief
 *
 * @param msgPtr
 * @param nbrListPtr
 * @returns void
 */
uint8 broadcastMinMaxUpdate(BroadcastMinMaxMessage* msgPtr, NbrList* nbrListPtr);


/**
 * @brief
 *
 * @param msgPtr
 * @returns void
 */
uint8 broadcastMinMaxGetValLocal(BroadcastMinMaxMessage* msgPtr);


/**
 * @brief
 *
 * @param msgPtr
 * @returns void
 */
uint8 broadcastMinMaxGetVal(BroadcastMinMaxMessage* msgPtr);

#endif /* BROADCASTMINMAX_H_ */
