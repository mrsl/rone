/*
 * @file Navigation-midangle.c
 * @brief used to steer a robot through a network of robots without crashing
 * @since Sep 22, 2011
 * @author Golnaz Habibi
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define PARENT_ID_UNDECIDED			0xFF
#define	MIN_PARENT_ID				0
#define BUMP_RELECT_DISTANCE			35

void nbrListPickGuides(NbrList* nbrListGuidesPtr, BroadcastMessage* msgPtr) {
	NbrList parents, siblings, nbrListAll;
	Nbr* nbrPtr;

	// init our neighbor lists
	nbrListCreate(&nbrListAll);
	nbrListGetParents(&parents, &nbrListAll, msgPtr);
	nbrListGetSiblings(&siblings, &nbrListAll, msgPtr);
	nbrListClear(nbrListGuidesPtr);

	nbrPtr = nbrListFindSource(&parents, msgPtr);

	if (nbrPtr) {
		nbrListAddNbr(nbrListGuidesPtr, nbrPtr);
	} else if (parents.size == 0) {
		// do nothing.  return empty list
	} else if (parents.size == 1) {
		nbrPtr = nbrListGetFirst(&parents);
			nbrListAddNbr(nbrListGuidesPtr, nbrPtr);
//			if(siblings.size > 0) {
//				// one parent, >= 0 siblings.  return parent and closest* sibling
//				nbrPtr = nbrListGetClosestNbrToBearing(&siblings, nbrPtr->bearing);
//				nbrListAddNbr(nbrListGuidesPtr, nbrPtr);
//			}
	} else {
		// parents >= 1.  return two closest* parents
		int16 avgBearing = nbrListAverageBearing(&parents);
		nbrPtr = nbrListGetClosestNbrToBearing(&parents, avgBearing);
		nbrListAddNbr(nbrListGuidesPtr, nbrPtr);
		nbrListRemoveNbr(&parents, nbrPtr);
		nbrPtr = nbrListGetClosestNbrToBearing(&parents, avgBearing);
		nbrListAddNbr(nbrListGuidesPtr, nbrPtr);
	}
}


#define NAVIGATION_RV_MAX_SCALE			1000
#define ROTATION_GAIN					40


/*
 *  @brief
 *
 *  @param behPtr for behavior that is updated by this function
 *  @param nbrListPtr for list of neighbors
 *  @param tv desired translational velocity
 *  @returns updated behPtr
 */
Beh* midAngleNavigation(Beh* behPtr, NbrList* nbrListPtr, int32 tv) {
	//boolean returnVal = FALSE;
	int32 rv;
	int32 rvMax = tv * RVTV_RATIO;

	//check to see if we have one or more neighbors
	if (nbrListPtr->size == 0) {
		// no neighbors, do noting.
		//*midAnglePtr = 0;
		*behPtr = behInactive;
	} else {
		// we have at least one neighbor
		int32 thetaGoal;
		thetaGoal = nbrListAverageBearing(nbrListPtr);
		thetaGoal = normalizeAngleMilliRad2(thetaGoal);
		//TODO multiply by tv below to scale rotation speed with translation speed
		rv = (thetaGoal * ROTATION_GAIN) / 100;
		behPtr->rv = boundAbs(rv, rvMax * NAVIGATION_RV_MAX_SCALE / 100);
		behPtr->tv = tv;
		behPtr->active = TRUE;
		//*midAnglePtr = thetaGoal;
			}
	return behPtr;
}

/*
Beh* broadcastMessageNavigate(Beh* behPtr, BroadcastMessage* msgPtr, int32 tv) {
	NbrList nbrList, nbrListParents;
	uint8 hops;

	nbrListCreate(&nbrList);
	nbrListGetParents(&nbrListParents, &nbrList, msgPtr);
	midAngleNavigation(behPtr, &nbrListParents, tv);
}
*/
