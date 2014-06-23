/*
 * networkSize.c
 *
 *  Created on: Aug 28, 2012
 *      Author: Divya
 */


#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD		300
#define MAX_NEIGHBORS				10
#define MAX_HOPS					7

#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50


typedef struct treeNbrs {
	Nbr *parent;
	Nbr *children[MAX_NEIGHBORS];
	uint8 numChildren;
} treeNbrs;

typedef struct treeMessage {
	NbrMsgField hops;
	NbrMsgField partialSum;
	NbrMsgField totalSum;

	NbrMsgField rootID;
	NbrMsgField parentID;
} treeMessage;

treeMessage *findRoot(treeMessage *treeMsg);
boolean isRoot(treeMessage *treeMsg);
treeNbrs *getParentAndChildren(NbrList *nbrs, treeMessage *treeMsg);
void backgroundTask(void *parameters);
void behaviorTask(void *parameters);

/*
 * TODO: add rest of message
 */
void createMsg(treeMessage *treeMsg) {
	nbrMsgCreate(&(treeMsg->rootID), "rootID", 7, roneID);
	nbrMsgCreate(&(treeMsg->parentID), "parentID", 7, roneID);
	nbrMsgCreate(&(treeMsg->hops), "hops", 7, 0);
	nbrMsgCreate(&(treeMsg->partialSum), "partialSum", 7, 0);
	nbrMsgCreate(&(treeMsg->totalSum), "totalSum", 7, 0);
}

uint8 robotMsgGetRootIDNbr(Nbr *nbrPtr) {
	return nbrGetID(nbrPtr);
}

uint8 robotMsgGetParentID(treeMessage *treeMsg) {
	return nbrMsgGet(&(treeMsg->parentID));
}

uint8 robotMsgGetHops(treeMessage *treeMsg) {
	return nbrMsgGet(&(treeMsg->hops));
}

uint8 robotMsgGetPartialSum(treeMessage *treeMsg) {
	return nbrMsgGet(&(treeMsg->partialSum));
}

uint8 robotMsgGetTotalSum(treeMessage *treeMsg) {
	return nbrMsgGet(&(treeMsg->totalSum));
}

uint8 robotMsgGetHopsNbr(treeMessage *treeMsg, Nbr *nbrPtr) {
	return nbrMsgGetNbr(&(treeMsg->hops), nbrPtr);
}

uint8 robotMsgGetPartialSumNbr(treeMessage *treeMsg, Nbr *nbrPtr) {
	return nbrMsgGetNbr(&(treeMsg->partialSum), nbrPtr);
}

uint8 robotMsgGetTotalSumNbr(treeMessage *treeMsg, Nbr *nbrPtr) {
	return nbrMsgGetNbr(&(treeMsg->totalSum), nbrPtr);
}


//each IR broadcast message should have (hops, PS, total sum)
//define tree based on lowest id
//be able to define parents versus children
//tree downwards, converge cast upwards, broadcast downwards
//calculate partial sum--> partial sums of each child added up + 1
//timestamps
//if see msg with  lower id, update
//See https://people.csail.mit.edu/jamesm/publications/McLurkin-MeasuringAccuracy-DARS2008.pdf as reference

//establish spanning tree for timestep--> return [parent, [children array]], boolean if root, boolean if children
//deal with partial sums based on result spanning tree--> returns total sum
//create struct to hold hops, ps, total sun

//me: write isRoot, return true if root, false if not
//return parent and child arrays

int main(void) {
	volatile uint32 val1, val2;

	systemPreInit();
	systemInit();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	neighborsXmitEnable(TRUE);
	val1 = osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	val2 = osTaskCreate(backgroundTask, "background", 1024, NULL, BACKGROUND_TASK_PRIORITY);

	if ((val1 != pdTRUE)) {
		cprintf("could not create a task");
	}

	/* Start the scheduler. */
	osTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
   	 task. This is a very bad thing, and should not happen.*/
	return 0;
}

treeMessage *findRoot(treeMessage *treeMsg) {
	uint8 i, j, nbrRootID, selfRootID;
	NbrList *nbrs = malloc(sizeof(NbrList));

	// It will take the diameter of the network rounds for messages
	// to propagate, so update that many times plus a few for dropped messages
	for (i = 0; i < MAX_HOPS + 3; i++) {
		neighborsGetMutex();
		nbrListCreate(nbrs);

		// go through neighbors looking for lowest ID
		for (j = 0; j < nbrs->size; j++) {
			nbrRootID = nbrMsgGetNbr(&(treeMsg->rootID), nbrs->nbrs[j]);
			selfRootID = nbrMsgGet(&(treeMsg->rootID));
			// if we find a lower ID, replace our own
			if (nbrRootID < selfRootID) {
				nbrMsgSet(&(treeMsg->rootID), nbrRootID);
			}
		}

		neighborsPutMutex();
	}

	return treeMsg;
}

boolean isRoot(treeMessage *treeMsg) {
	if (roneID == nbrMsgGet(&(treeMsg->rootID))) {
		return TRUE;
	} else {
		return FALSE;
	}
}

treeNbrs *getParentAndChildren(NbrList *nbrs, treeMessage *treeMsg) {
	treeNbrs *tNbrs = malloc(sizeof(treeNbrs));
	uint8 i, hops, nbrHops;

	tNbrs->parent = NULL;   // max robot ID
	tNbrs->numChildren = 0;

	for (i = 0; i < nbrs->size; i++) {
		hops = nbrMsgGet(&(treeMsg->hops));
		nbrHops = nbrMsgGetNbr(&(treeMsg->hops), nbrs->nbrs[i]);

		// If the neighbor is one step closer to the root
		if (nbrHops == hops - 1) {
			// and we don't have a parent yet
			if (tNbrs->parent == NULL) {
				tNbrs->parent = nbrs->nbrs[i];
			}
			// or our parent has a higher ID
			else if (tNbrs->parent->ID < nbrGetID(nbrs->nbrs[i])) {
				tNbrs->parent = nbrs->nbrs[i];
			}
			// it becomes our new parent
		}

		// If the neighbor labels us a parent, they are our child
		else if (nbrMsgGetNbr(&(treeMsg->parentID), nbrs->nbrs[i]) == roneID) {
			tNbrs->children[tNbrs->numChildren] = nbrs->nbrs[i];
			tNbrs->numChildren++;
		}
	}

	// Put our parent's ID in our message.
	nbrMsgSet(&(treeMsg->parentID), tNbrs->parent->ID);

	return tNbrs;
}

treeMessage updatePartialSum (treeMessage msg, treeNbrs nbrs) {
	uint8 i;
	uint32 pSum = 0;
	Nbr* tempNbrPtr; //pointer
	if (nbrs.numChildren == 0) {
		pSum = 0;
	}
	for (i = 0; i < nbrs.numChildren; i++) {
		tempNbrPtr = nbrs.children[i];
		pSum += robotMsgGetPartialSumNbr(&msg, tempNbrPtr);
	}

	nbrMsgSet(&(msg.partialSum), pSum + 1);
	return msg;
}

treeMessage updateHops(treeMessage msg, NbrList *nbrs) {
	uint8 i, lowestHops, tempHops;
	lowestHops = tempHops = MAX_HOPS;
	for (i = 0; i < nbrs->size; i++) {
		tempHops = robotMsgGetHopsNbr(&msg, nbrs->nbrs[i]);
		if (tempHops < lowestHops) {
			lowestHops = tempHops;
		}
	}

	if ((lowestHops + 1) < nbrMsgGet(&(msg.hops))) {
		nbrMsgSet(&(msg.hops), lowestHops + 1);
	}

	return msg;
}

treeMessage updateTotalSum(treeMessage msg, treeNbrs nbrs){
	uint8 i;
	if (roneID != nbrMsgGet(&(msg.rootID))) {
		nbrMsgSet(&(msg.totalSum), robotMsgGetTotalSumNbr(&msg, nbrs.parent));
	}
	return msg;
}

/*
void updateNbrMessage(treeMessage msg){
	boolean checkValid = TRUE;
    if ((roneID != nbrGetID(&msg)) && (checkValid)){
    	nbrMsgSet(&msg->rootID, msg.rootID);
    	nbrMsgSet(&msg->parentID, msg.parentID);
    	nbrMsgSet(&msg->hops, msg.hops);
    	nbrMsgSet(&msg->partialSum, msg.partialSum);
    	nbrMsgSet(&msg->totalSum, msg.totalSum);
    }
}
*/

// the background task runs all the time
void backgroundTask(void* parameters) {
	for (;;) {
		// delay to let other tasks run at same priority
		osTaskDelay(100);
	}
}


void behaviorTask(void* parameters) {
	//initialization
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound;
	uint8 neighbors;
	uint32 currentTime = osTaskGetTickCount();


	NbrList *nbrList = malloc(sizeof(NbrList));

	treeMessage tMsg;
	treeNbrs tNbrs;


	for (;;) {
		neighborsGetMutex();
		nbrListCreate(nbrList);

		tNbrs = *getParentAndChildren(nbrList, &tMsg);
		tMsg = updateHops(tMsg, nbrList);
		tMsg = updatePartialSum (tMsg, tNbrs);
		tMsg = updateTotalSum (tMsg, tNbrs);

	   // updateNbrMessage(tMsg);


		neighborsPutMutex();
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}


