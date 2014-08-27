/*
 * list.h
 *
 *  Created on: Apr 14, 2014
 *      Author: mrsl
 */

#ifndef ROBOTLIST_H_
#define ROBOTLIST_H_

#define GLOBAL_ROBOTLIST_MAX_SIZE 				10
#define GLOBAL_ROBOTLIST_ROUND_WAIT_TIME		10



// each robot is an element of n trees (n = size of network) , ID is the ID of the tree ??? (the root may be???), each robot has a hop in each tree

typedef struct GlobalRobotListElement {
	NbrData ID;
	NbrData nonce;
	NbrData Hops;			//0 if can not see, 1 if source, else increase form 1;
	NbrData	ParentID;
	uint32 updateRound;
} GlobalRobotListElement;

 // The list of trees
typedef struct GlobalRobotList {
	uint8 size;		/**< the size of the global robot list */
	uint8 nonce;	/**< this robot's nonce */
	GlobalRobotListElement list[GLOBAL_ROBOTLIST_MAX_SIZE];
} GlobalRobotList;


void globalRobotListCreate(GlobalRobotList* globalRobotListPtr);

GlobalRobotListElement* globalRobotListGetElt(GlobalRobotList* globalRobotListPtr, uint8 idx);

void grlUpdateElt(GlobalRobotList* globalRobotListPtr, uint8 robotID, uint8 nonce);

void globalRobotListUpdate(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr);

void globalRobotListPrintAllTree(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr);

void globalRobotListPrintNbrTree(GlobalRobotList* globalRobotListPtrm, Nbr* nbrptr);

void globalRobotListPrintSelfTree(GlobalRobotList* globalRobotListPtr);

void globalRobotListDelete(GlobalRobotList* globalRobotListPtr, uint8 idx);

void globalRobotUpdateTree(GlobalRobotList* globalRobotListPtr, NbrList nbrListPtr, uint8 idx);

void globalRobotListMove(GlobalRobotList* globalRobotListPtr, uint8 strtIdx, uint8 finIdx);

uint8 globalRobotListGetSize(GlobalRobotList* globalRobotListPtr);

int8 globalRobotListGetIndex(GlobalRobotList* globalRobotListPtr, uint8 robotID);

uint8 grlEltGetID(GlobalRobotListElement* grlEltPtr);

uint8 grlEltGetNonce(GlobalRobotListElement* grlEltPtr);

uint8 grlEltGetHops(GlobalRobotListElement* grlEltPtr);

uint8 grlEltGetParentID(GlobalRobotListElement* grlEltPtr);

uint32 grlEltGetTimeStamp(GlobalRobotListElement* grlEltPtr);

#endif /* ROBOTLIST_H_ */
