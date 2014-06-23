/*
 * robotList.c
 *
 *  Created on: Apr 14, 2014
 *      Author: mrsl
 */

#include "roneos.h"
#include "ronelib.h"
#include "robotList.h"

void robotListCreate(nbrDataRobotList* totalRobotList, uint8 listMaxLen) {
	uint8 i;

	/* Q: you haven't initialized nonceTimeStamp, should we? */

	for (i = 0; i < listMaxLen; ++i) {
		nbrDataCreate(&(totalRobotList->robotList[i].ID), "ID", 8,
				ROBOT_ID_NULL);
		nbrDataCreate(&(totalRobotList->robotList[i].nonce), "nonce", 8, 0);
	}

	totalRobotList->manipulationRobotCount = 1;
}

void robotListUpdate(nbrDataRobotList* totalRobotList, uint8 listMaxLen, NbrList* nbrListPtr) {
	int8 i, j, k;
	Nbr* nbrPtr;
	nbrDataRobotList* totalNbrDataPtr = 0;

	for (i = 0; i < nbrListPtr->size; i++) {
		// get a nbrptr from my neighbor list
		nbrPtr = nbrListGetNbr(nbrListPtr, i);

		/* find the total robot data structure for this neighbor from *it's* total robot list.
		 * This will let us find the current nonce.
		 */
		boolean foundNbr = FALSE;

		for (j = 0; j < listMaxLen; j++) { // changed ++j to j++

			uint8 nbrTotalRobotListRobotID = nbrDataGetNbr(
					&(totalRobotList->robotList[j].ID), nbrPtr);
			uint8 nbrTotalRobotListRobotNonce = nbrDataGetNbr(
					&(totalRobotList->robotList[j].nonce), nbrPtr);

			/* I have the ID of the bnr from the total robot list of my nbr.  do an insertion
			 * sort onto my total robot list */

			for (k = 0; k < listMaxLen; k++) {
				uint8 localTotalRobotListRobotID = nbrDataGet(
						&(totalRobotList->robotList[k].ID));
				if (nbrTotalRobotListRobotID == localTotalRobotListRobotID) {
					uint8 localTotalRobotListRobotNonce = nbrDataGet(
							&(totalRobotList->robotList[k].nonce));

					/* if local robot nonce does not equal to nbr nonce, set the local one with
					 * its neighbor nonce value; CLEAR THE LOCAL NONCETIMESTAMP (not sure) */
					if (nbrTotalRobotListRobotNonce
							!= localTotalRobotListRobotNonce) {
						NbrData treeElement = totalRobotList->robotList[k].nonce;
						treeElement.value= nbrTotalRobotListRobotNonce;
						totalRobotList->robotList[k].nonceTimestamp = osTaskGetTickCount();
						/* clear the local nonce timestamp (not sure how to clear it, is
						 * there a method for clearing?)*/
						totalRobotList->robotList[k].nonceTimestamp = 0;
					} else {
						/* update timestamp when nonce are the same */
						totalRobotList->robotList[k].nonceTimestamp = osTaskGetTickCount();
					}
					foundNbr = TRUE;
					break;
				}
			}

			// sort totalRobotList by robot ID in ascending order
			if (!foundNbr) {
				uint8 loop_counter = totalRobotList->manipulationRobotCount;
				nbrDataRobotElement RobotElementToBeInserted;
					(RobotElementToBeInserted.ID).value = nbrTotalRobotListRobotID;
					(RobotElementToBeInserted.nonce).value = nbrTotalRobotListRobotNonce;
				// make a duplicate at the end of the list
				(totalRobotList->robotList[loop_counter]) =
						(totalRobotList->robotList[loop_counter - 1]);
				while (loop_counter > 0) {
						if (RobotElementToBeInserted.ID.value < (totalRobotList->robotList[loop_counter]).ID.value) {
					loop_counter--;
					totalRobotList->robotList[loop_counter] =
							totalRobotList->robotList[loop_counter - 1];
						} else {
							totalRobotList->robotList[loop_counter] = RobotElementToBeInserted;
						break;
						}
				}

				if (loop_counter == 0) {
					totalRobotList->robotList[loop_counter] =	RobotElementToBeInserted;

					//int32 manipulationRobotsCounter++;  // we increment robot_counter because we've updated a new tree in array
				} else {
					boolean treeExist = FALSE;
				}

			}
		}
		// go thorugh all the nbrs and remove ones whose timestamps have expired

		/* Note from JJF & Wei: we're not sure how to check if timestamps have expired... so we just left it blank for now :( */
	}

}
