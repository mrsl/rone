/*
 * list.c
 *
 *  Created on: Apr 14, 2014
 *      Author: mrsl
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"


void globalRobotListCreate(GlobalRobotList* globalRobotListPtr) {
	uint8 i;

	for (i = 0; i < GLOBAL_ROBOTLIST_MAX_SIZE; ++i) {
		nbrDataCreate(&(globalRobotListPtr->list[i].ID), "ID", 8, ROBOT_ID_NULL);
		nbrDataCreate(&(globalRobotListPtr->list[i].nonce), "nonce", 8, 0);
		nbrDataCreate(&(globalRobotListPtr->list[i].Hops), "Hops", 8, 0);
		nbrDataCreate(&(globalRobotListPtr->list[i].ParentID), "ParentID", 8, 0);
		globalRobotListPtr->list[i].updateRound = 0;
	}

	globalRobotListPtr->size = 0;
	globalRobotListPtr->nonce = 1;
}


GlobalRobotListElement* globalRobotListGetElt(GlobalRobotList* globalRobotListPtr, uint8 idx) {
	if (idx < globalRobotListPtr->size) {
		return &(globalRobotListPtr->list[idx]);
	} else {
		return NULL;
	}
}


uint8 globalRobotListGetSize(GlobalRobotList* globalRobotListPtr) {
	return globalRobotListPtr->size;
}


int8 globalRobotListGetIndex(GlobalRobotList* globalRobotListPtr, uint8 robotID) {
	int j;
	for (j = 0; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++) {
		uint8 nbrTotalRobotListRobotID = nbrDataGet(&(globalRobotListPtr->list[j].ID));
		if (nbrTotalRobotListRobotID == ROBOT_ID_NULL) {
			// no more robots on the list of this neighbor.  break and select the next nbr.
			return -1;
		}else if (nbrTotalRobotListRobotID == robotID) {
			return j;
		}
	}
	return -1;
}



uint8 grlEltGetID(GlobalRobotListElement* grlEltPtr) {
	if (grlEltPtr) {
		return nbrDataGet(&grlEltPtr->ID);
	} else {
		return ROBOT_ID_NULL;
	}
}


uint8 grlEltGetNonce(GlobalRobotListElement* grlEltPtr) {
	if (grlEltPtr) {
		return nbrDataGet(&grlEltPtr->nonce);
	} else {
		return 0;
	}
}

uint8 grlEltGetHops(GlobalRobotListElement* grlEltPtr) {
	if (grlEltPtr) {
		return nbrDataGet(&grlEltPtr->Hops);
	} else {
		return 0;
	}
}

uint8 grlEltGetParentID(GlobalRobotListElement* grlEltPtr) {
	if (grlEltPtr) {
		return nbrDataGet(&grlEltPtr->ParentID);
	} else {
		return 0;
	}
}


uint32 grlEltGetTimeStamp(GlobalRobotListElement* grlEltPtr) {
	if (grlEltPtr) {
		return grlEltPtr->updateRound;
	} else {
		return 0;
	}
}


void grlUpdateElt(GlobalRobotList* globalRobotListPtr, uint8 robotID, uint8 nonce) {
	uint8 i;
	GlobalRobotListElement* grlEltPtr;
	GlobalRobotListElement* grlEltPtr_Next;
	boolean foundRobot = FALSE;
	// rip over the list.  find the robot with ID robotID, if there, update the nonce and timestamp
	for (i = 0; i < globalRobotListGetSize(globalRobotListPtr); i++) {
		grlEltPtr = globalRobotListGetElt(globalRobotListPtr, i);
		if (grlEltPtr) {
			if (grlEltGetID(grlEltPtr) == robotID) {
				// found the robot.
				foundRobot = TRUE;
				grlEltPtr->updateRound = neighborsGetRound();

				//check to see if the nonce is different.  update the nonce
				if (nonce > grlEltGetNonce(grlEltPtr) || (((100 > nonce) && (grlEltGetNonce(grlEltPtr) > 220)))) {
					nbrDataSet(&grlEltPtr->nonce, nonce);
				}
				break;
			}
		} else {
			// something bad has happened.  null element.
			return;
		}
	}

	if (!foundRobot) {
		// did not find the robot on the list to update.  if there is room, add this robot to
		// the list and shift the others back.  This assumes the list is already sorted.
		if (globalRobotListPtr->size < GLOBAL_ROBOTLIST_MAX_SIZE) {
			// insert robot into list.  Use reverse insertion to insert in place.
			globalRobotListPtr->size++;
			if(globalRobotListPtr->size != 1){
				for(i = globalRobotListPtr->size; i > 0; i--){

					grlEltPtr = globalRobotListGetElt(globalRobotListPtr, i-1);
					grlEltPtr_Next = globalRobotListGetElt(globalRobotListPtr, i);

					globalRobotListMove(globalRobotListPtr,i-1,i);

					if ((grlEltGetID(grlEltPtr) < robotID) && (grlEltGetID(grlEltPtr) != 0)) {
						nbrDataSet(&(grlEltPtr_Next->ID),robotID);
						nbrDataSet(&(grlEltPtr_Next->nonce),nonce);
						nbrDataSet(&(grlEltPtr_Next->Hops), 0);
						nbrDataSet(&(grlEltPtr_Next->ParentID),0);
						grlEltPtr_Next->updateRound = neighborsGetRound();
						return;
					}
				}
				grlEltPtr = globalRobotListGetElt(globalRobotListPtr,0);
				nbrDataSet(&(grlEltPtr->ID),robotID);
				nbrDataSet(&(grlEltPtr->nonce),nonce);
				nbrDataSet(&(grlEltPtr->Hops), 0);
				nbrDataSet(&(grlEltPtr->ParentID),0);
				grlEltPtr->updateRound = neighborsGetRound();
				return;
			}else{
				grlEltPtr = globalRobotListGetElt(globalRobotListPtr, 0);
				nbrDataSet(&(grlEltPtr->ID),robotID);
				nbrDataSet(&(grlEltPtr->nonce),nonce);
				nbrDataSet(&(grlEltPtr->Hops), 0);
				nbrDataSet(&(grlEltPtr->ParentID),0);
				grlEltPtr->updateRound = neighborsGetRound();
			}
		}
	}
}


void globalRobotListUpdate(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr) {
	int8 i, j;
	Nbr* nbrPtr;


	// first, add myself to the robot list
	grlUpdateElt(globalRobotListPtr, roneID, globalRobotListPtr->nonce);
	(globalRobotListPtr->nonce)++;

	for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
		// get a nbrptr from my neighbor list
		nbrPtr = nbrListGetNbr(nbrListPtr, i);

		/* find the total robot data structure for this neighbor from *it's* total robot list.
		 * This will let us find the current nonce.
		 */

		for (j = 0; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++) {
			uint8 nbrTotalRobotListRobotID = nbrDataGetNbr(&(globalRobotListPtr->list[j].ID), nbrPtr);
			uint8 nbrTotalRobotListRobotNonce = nbrDataGetNbr(&(globalRobotListPtr->list[j].nonce), nbrPtr);
			if (nbrTotalRobotListRobotID == ROBOT_ID_NULL) {
				// no more robots on the list of this neighbor.  break and select the next nbr.
				break;
			}
			// update or add this robot to the list
			grlUpdateElt(globalRobotListPtr, nbrTotalRobotListRobotID, nbrTotalRobotListRobotNonce);
		}
	}
	// go thorugh all the nbrs and remove ones whose update round is too far back

	for (j = 0; j < globalRobotListPtr->size; j++) {
		uint8 nbrTotalRobotListRobotID = nbrDataGet(&(globalRobotListPtr->list[j].ID));
		int nbrUpdateRound = globalRobotListPtr->list[j].updateRound;
		if(nbrTotalRobotListRobotID == ROBOT_ID_NULL) {
			globalRobotListDelete(globalRobotListPtr,j);
			if(globalRobotListPtr->size < 1){
				return;
			}else{
				j--;
			}
		}
		if(neighborsGetRound() > GLOBAL_ROBOTLIST_ROUND_WAIT_TIME){
			if((nbrUpdateRound < (neighborsGetRound()-GLOBAL_ROBOTLIST_ROUND_WAIT_TIME)) ||(nbrUpdateRound > neighborsGetRound()) ){
				globalRobotListDelete(globalRobotListPtr,j);
				if(globalRobotListPtr->size < 1){
					return;
				}else{
					j--;
				}
			}
		}
	}

	//Go through list and Update each Tree

	GlobalRobotListElement* grlEltPtr;
	for (j = 0; j < globalRobotListPtr->size; j++) {
		grlEltPtr = globalRobotListGetElt(globalRobotListPtr, j);
		if(grlEltGetID(grlEltPtr)==roneID){
			nbrDataSet(&(grlEltPtr->Hops), 1);
			nbrDataSet(&(grlEltPtr->ParentID), 0);
		} else{
			globalRobotUpdateTree(globalRobotListPtr, *nbrListPtr, j);
		}
	}

}


void globalRobotUpdateTree(GlobalRobotList* globalRobotListPtr, NbrList nbrListPtr, uint8 idx){
	uint8 i,j,nbrPtrID;
	Nbr* nbrPtr;
	GlobalRobotListElement* SelfgrlEltPtr = globalRobotListGetElt(globalRobotListPtr, idx);
	nbrDataSet(&(SelfgrlEltPtr->Hops), 0);
	for (i = 0; i < nbrListPtr.size; i++){
		nbrPtr = nbrListPtr.nbrs[i];
		nbrPtrID = nbrGetID(nbrPtr);
		if(nbrPtrID == grlEltGetID(SelfgrlEltPtr)){
			nbrDataSet(&(SelfgrlEltPtr->Hops), 2);
			nbrDataSet(&(SelfgrlEltPtr->ParentID), nbrPtrID);
			return;
		}
		for (j = 0; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++) {
			uint8 nbrRobotListID = nbrDataGetNbr(&(globalRobotListPtr->list[j].ID), nbrPtr);
			uint8 nbrRobotListHops = nbrDataGetNbr(&(globalRobotListPtr->list[j].Hops), nbrPtr);
			if(nbrRobotListID ==ROBOT_ID_NULL ){
				//return;
			} else if((nbrRobotListID == grlEltGetID(SelfgrlEltPtr)) && (nbrRobotListHops != 0) && !(nbrRobotListHops > 11) ){
				if(nbrDataGet(&(SelfgrlEltPtr->Hops)) == 0){
					nbrDataSet(&(SelfgrlEltPtr->Hops), nbrRobotListHops + 1);
					nbrDataSet(&(SelfgrlEltPtr->ParentID), nbrPtrID);
				}else if(nbrRobotListHops <= nbrDataGet(&(SelfgrlEltPtr->Hops))){
					nbrDataSet(&(SelfgrlEltPtr->Hops), nbrRobotListHops + 1);
					nbrDataSet(&(SelfgrlEltPtr->ParentID), nbrPtrID);
				}
			}
		}
	}
}


void globalRobotListPrintAllTree(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr){
	int8 i;
	Nbr* nbrPtr;
	globalRobotListPrintSelfTree(globalRobotListPtr);
	for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
		// get a nbrptr from my neighbor list
		nbrPtr = nbrListGetNbr(nbrListPtr, i);
		if(nbrPtr!= NULL){
			globalRobotListPrintNbrTree(globalRobotListPtr,nbrPtr);
		}
	}
}


void globalRobotListPrintSelfTree(GlobalRobotList* globalRobotListPtr) {
	int j;
	GlobalRobotListElement* grlEltPtr;
	rprintf("Self %d Nonce %d, Size %d Round %d\n",roneID, globalRobotListPtr->nonce, globalRobotListPtr->size, neighborsGetRound());
	for (j = 0; j < globalRobotListPtr->size; j++) {
		grlEltPtr = globalRobotListGetElt(globalRobotListPtr, j);
		uint8 nbrTotalRobotListRobotID =grlEltGetID(grlEltPtr);
		rprintf("	ID %d, Nonce %d, Hops %d, ParentID %d, Update Round %d \n",nbrTotalRobotListRobotID, grlEltGetNonce(grlEltPtr),grlEltGetHops(grlEltPtr),grlEltGetParentID(grlEltPtr),globalRobotListPtr->list[j].updateRound);
	}
}

void globalRobotListPrintNbrTree(GlobalRobotList* globalRobotListPtr, Nbr* nbrptr){
	int j;
	rprintf("Origin %d\n",nbrGetID(nbrptr));
	for (j = 0; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++) {
		uint8 nbrRobotListID = nbrDataGetNbr(&(globalRobotListPtr->list[j].ID), nbrptr);
		uint8 nbrRobotListRobotNonce = nbrDataGetNbr(&(globalRobotListPtr->list[j].nonce), nbrptr);
		uint8 nbrRobotListHops = nbrDataGetNbr(&(globalRobotListPtr->list[j].Hops), nbrptr);
		uint8 nbrRobotListParentID = nbrDataGetNbr(&(globalRobotListPtr->list[j].ParentID), nbrptr);
		if(nbrRobotListID ==ROBOT_ID_NULL ){
			return;
		}
		rprintf("	ID %d, Nonce %d, Hops %d, ParentID %d \n",nbrRobotListID,nbrRobotListRobotNonce, nbrRobotListHops,nbrRobotListParentID );
	}
}

void globalRobotListDelete(GlobalRobotList* globalRobotListPtr, uint8 idx) {
	GlobalRobotListElement* grlEltPtr;
	int i,j;
	globalRobotListPtr->size--;
	for(i = idx; i <globalRobotListPtr->size; i++){
		globalRobotListMove(globalRobotListPtr,i,i+1);

	}
	for(j = i; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++){
		grlEltPtr = globalRobotListGetElt(globalRobotListPtr,j);
		nbrDataSet(&grlEltPtr->ID, ROBOT_ID_NULL);
		nbrDataSet(&grlEltPtr->nonce, 0);
		nbrDataSet(&(grlEltPtr->Hops), 0);
		nbrDataSet(&(grlEltPtr->ParentID),0);
		grlEltPtr->updateRound = 0;
	}
}

void globalRobotListMove(GlobalRobotList* globalRobotListPtr, uint8 strtIdx, uint8 finIdx){
	if((strtIdx < GLOBAL_ROBOTLIST_MAX_SIZE)&&(strtIdx < GLOBAL_ROBOTLIST_MAX_SIZE)){
		GlobalRobotListElement* grlEltPtr_Strt = globalRobotListGetElt(globalRobotListPtr, strtIdx);
		GlobalRobotListElement* grlEltPtr_Fin = globalRobotListGetElt(globalRobotListPtr, finIdx);

		nbrDataSet(&(grlEltPtr_Fin->ID), grlEltGetID(grlEltPtr_Strt));
		nbrDataSet(&(grlEltPtr_Fin->nonce),grlEltGetNonce(grlEltPtr_Strt));
		nbrDataSet(&(grlEltPtr_Fin->Hops), grlEltGetHops(grlEltPtr_Strt));
		nbrDataSet(&(grlEltPtr_Fin->ParentID),grlEltGetParentID(grlEltPtr_Strt));

		grlEltPtr_Fin->updateRound = grlEltPtr_Strt->updateRound;
	}

}
