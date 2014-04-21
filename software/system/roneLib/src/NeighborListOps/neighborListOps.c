/*
 * @file neighborListOps.c
 * @brief   operations on lists of neighbors:  adding/removing, searching, looking for robots with certain bearings.
 * @details  ???
 *
 * @since Sep 11, 2011
 * @author jmclurkin
 */
#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

/*
 * @brief Adds a neighbor to a neighbor's linked list of neighbors if there is room.
 *
 * \internal
 * TODO: Shouldn't we return 0 on success, and -1 on failure?  I don't like voids (Aaron Becker)
 * \endinternal
 * @param nbrListPtr the neighbor's linked list
 * @param nbrPtr a new neighbor
 * @returns void
 */
void nbrListAddNbr(NbrList* nbrListPtr, Nbr* nbrPtr){
	if((nbrListPtr->size < NEIGHBOR_MAX) && (nbrPtr != NULL)) {
		nbrListPtr->nbrs[nbrListPtr->size] = nbrPtr;
		nbrListPtr->size++;
	}
}

/*
 * @brief Copies the neighbor list from SRC to DST.
 *
 * @param nbrListSrcPtr (unchanged)
 * @param nbrListDstPtr becomes a copy of nbrListSrcPtr
 * @returns void
 */
void nbrListCopy(NbrList* nbrListDstPtr, NbrList* nbrListSrcPtr){
	uint8 i;

	for (i = 0; i < nbrListSrcPtr->size; i++) {
		nbrListDstPtr->nbrs[i] = nbrListSrcPtr->nbrs[i];
	}
	nbrListDstPtr->size = nbrListSrcPtr->size;
}

/*
 * @brief Determines average bearing of all neighbors using sum of vectors method (optimized).
 *
 * @param nbrListPtr list of neighbors
 * @returns average bearing (milliradians)
 */
int16 nbrListAverageBearing(NbrList* nbrListPtr) {
	uint8 i;
	int32 y = 0;
	int32 x = 0;
	int16 avgbearing = 0;

	for (i = 0; i < nbrListPtr->size; i++) {
		y += sinMilliRad(nbrListPtr->nbrs[i]->bearing);
		x += cosMilliRad(nbrListPtr->nbrs[i]->bearing);
	}
	avgbearing = atan2MilliRad(y, x);
	return avgbearing;
}

/*
 * @brief returns the neighbor whose bearing is closest to nbrPtr's bearing, NULL if list is empty.
 *
 * @param nbrListPtr list of neighbors
 * @param nbrPtr a neighbor used as a reference
 * @returns pointer to neighbor whose bearing is closest to nbrPtr's bearing, NULL if list is empty.
 */
Nbr* nbrListGetSmallestAngleDeviation2(NbrList* nbrListPtr, Nbr* nbrPtr) {
	uint8 i;
	int16 bearingMin = MILLIRAD_PI; //set to maximum value
	int16 diff_bearing;
	Nbr* nbrPtr_near = NULL;

	for (i = 0; i < nbrListPtr->size; i++) {
		diff_bearing = smallestAngleDifference(nbrPtr->bearing,(nbrListPtr->nbrs[i]->bearing));
		if(abs(diff_bearing) <= abs(bearingMin)) {
			nbrPtr_near = nbrListPtr->nbrs[i];
			bearingMin=diff_bearing;
		}
	}
	return nbrPtr_near;
}

/*
 * @brief removes nbrPtr from list (if it exists)
 *
 * \internal
 * shouldn't we return 0 on success, and -1 on failure?  I don't like voids (Aaron Becker)
 * \endinternal
 *
 * @param nbrListPtr list of neighbors
 * @param nbrPtr a neighbor to be removed
 * @returns void
 */
void nbrListRemoveNbr(NbrList* nbrListPtr, Nbr* nbrPtr){
	uint8 i;

	for (i = 0; i < nbrListPtr->size; i++) {
		if(nbrListPtr->nbrs[i] == nbrPtr) {
			if(i < (nbrListPtr->size - 1)) { //repair list
				nbrListPtr->nbrs[i] = nbrListPtr->nbrs[nbrListPtr->size - 1];
			}
			nbrListPtr->size--;
			break;
		}
	}
}

/*
 * @brief returns the neighbor whose bearing is closest to bearing, NULL if list is empty.
 *
 * @param nbrListPtr list of neighbors
 * @param bearing a reference bearing
 * @returns pointer to neighbor whose bearing is closest to bearing, NULL if list is empty.
 */
Nbr* nbrListGetClosestNbrToBearing(NbrList* nbrListPtr, int16 bearing) {
	uint8 i;
	int16 bearingMin = MILLIRAD_PI;
	int16 bearingError;
	Nbr* nbrPtr = NULL;

	for (i = 0; i < nbrListPtr->size; i++) {
		bearingError = normalizeAngleMilliRad2(nbrListPtr->nbrs[i]->bearing - bearing);
		if(abs(bearingError) <= abs(bearingMin)) {
			nbrPtr = nbrListPtr->nbrs[i];
			bearingMin = bearingError;
		}
	}
	return nbrPtr;
}

/*
 * @brief Returns a pointer to the neighbor at the head of the list, or NULL
 *
 * @param nbrListPtr pointer for the list of neighbors
 * @returns a pointer to the neighbor at the head of the list, or NULL
 */
Nbr* nbrListGetFirst(NbrList* nbrListPtr) {
	if (nbrListPtr->size > 0) {
		return nbrListPtr->nbrs[0];
	} else {
		return NULL;
	}
}

/*
 * @brief Returns a pointer to the neighbor second in the list, or NULL
 *
 * @param nbrListPtr pointer for the list of neighbors
 * @returns a pointer to the neighbor second in the list, or NULL
 */
Nbr* nbrListGetSecond(NbrList* nbrListPtr) {
	if (nbrListPtr->size > 1) {
		return nbrListPtr->nbrs[1];
	} else {
		return NULL;
	}
}

/*
 * @brief Returns a pointer to the neighbor with ID, or NULL.
 *
 * @param nbrListPtr pointer for the list of neighbors
 * @param ID the unique number of the robot being searched for
 * @returns a pointer to the neighbor with ID, or NULL
 */
Nbr* nbrListGetNbrWithID(NbrList* nbrListPtr, uint8 ID) {
	uint8 i;
	Nbr* nbrPtr = NULL;

	for (i = 0; i < nbrListPtr->size; i++) {
		nbrPtr = nbrListPtr->nbrs[i];
		if(nbrPtr->ID == ID) {
			return nbrPtr;
		}
	}
	return NULL;
}

//TODO: Who uses this/does it work?
///*
// * @brief fills the provided nbrList with the current set of neighbors that are in the same group
// */
//void groupList(NbrList* partialList, int group) {
//	uint8 i;
//	partialList->size = 0;
//	for (i = 0; i < nd.nbrsSize; i++) {
//		if (nd.nbrs[i].active) {
//			if ((nd.nbrs[i].ID % 3) == group) {
//				partialList->nbrs[partialList->size++] = &nd.nbrs[i];
//			}
//		}
//	}
//}

/*
 * @brief Computes the union of the two input neighbor lists.
 *
 * @param nbrListOutPtr pointer to the list created from the union
 * @param nbrList1Ptr first neighbor list
 * @param nbrList2Ptr second neighbor list
 * @returns a pointer to a neighbor list that is the union of the two input lists
 */
NbrList* nbrListUnion(NbrList* nbrListOutPtr, NbrList* nbrList1Ptr, NbrList* nbrList2Ptr) {
	uint8 i;
	Nbr* nbrPtr;

	nbrListCopy(nbrListOutPtr, nbrList1Ptr);
	for (i = 0; i < nbrListGetSize(nbrList2Ptr); ++i) {
		nbrPtr = nbrListGetNbr(nbrList2Ptr, i);
		if (nbrListGetNbrWithID(nbrListOutPtr, nbrGetID(nbrPtr)) == NULL) {
			nbrListAddNbr(nbrListOutPtr, nbrPtr);
		}
	}
	return nbrListOutPtr;
}

NbrList* nbrListIntersection(NbrList* nbrListOutPtr, NbrList* nbrList1Ptr, NbrList* nbrList2Ptr) {
	return nbrListOutPtr;
}

NbrList* nbrListSubtraction(NbrList* nbrListOutPtr, NbrList* nbrList1Ptr, NbrList* nbrList2Ptr) {
	return nbrListOutPtr;
}

NbrList* nbrListFindNbrsWithDataGreaterThan(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val) {

	return nbrListOutPtr;
}

NbrList* nbrListFindNbrsWithDataLessThan(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val) {

	return nbrListOutPtr;
}

NbrList* nbrListFindNbrsWithDataNotEqual(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val) {

	return nbrListOutPtr;
}

/*
 * @brief returns a pointer to the list of neighbors with the same data value
 *
 * @param nbrListOutPtr pointer to output list of neighbors
 * @param nbrListInPtr pointer to input list of neighbors
 * @param nbrDataPtr list of data on messages
 * @param val value of data to match
 * @returns a pointer to the output list of neighbors with the same data value
 */
NbrList* nbrListFindNbrsWithDataEqual(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val) {
	uint8 i;
	Nbr* nbrPtr;

	nbrListClear(nbrListOutPtr);
	if(nbrDataPtr == NULL) {
		return nbrListOutPtr;
	}
	for (i = 0; i < nbrListGetSize(nbrListInPtr); i++) {
		nbrPtr = nbrListGetNbr(nbrListInPtr, i);
		if (nbrDataGetNbr(nbrDataPtr, nbrPtr) == val) {
			nbrListAddNbr(nbrListOutPtr, nbrPtr);
		}
	}
	return nbrListOutPtr;
}


/*
 * @brief Finds neighbors that are robots
 *
 * @param nbrListOutPtr pointer to the neighbor list updated by this function
 * @param nbrListInPtr pointer to the neighbor list to search for robots
 * @returns updated nbrListOutPtr that contains only neighbors that are robots
 */
NbrList* nbrListGetRobots(NbrList* nbrListOutPtr, NbrList* nbrListInPtr) {
	uint8 i;
	Nbr* nbrPtr;

	nbrListClear(nbrListOutPtr);
	for (i = 0; i < nbrListGetSize(nbrListInPtr); i++) {
		nbrPtr = nbrListGetNbr(nbrListInPtr, i);
		if (nbrIsRobot(nbrPtr)) {
			nbrListAddNbr(nbrListOutPtr, nbrPtr);
		}
	}
	return nbrListOutPtr;
}

/*
 * @brief returns a pointer to the list of neighbors with the same data values
 *
 * @param nbrListOutPtr pointer to output list of neighbors
 * @param nbrListInPtr pointer to input list of neighbors
 * @param nbrDataPtr list of data on messages
 * @param val1 first value of data to match
 * @param val2 second value of data to match
 * @returns a pointer to the output list of neighbors with the same data values
 */
NbrList* nbrListFindNbrsWithDataEqual2(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val1, uint8 val2) {
	uint8 i;
	Nbr* nbrPtr;

	nbrListClear(nbrListOutPtr);
	if(nbrDataPtr == NULL) {
		return nbrListOutPtr;
	}
	for (i = 0; i < nbrListGetSize(nbrListInPtr); i++) {
		nbrPtr = nbrListGetNbr(nbrListInPtr, i);
		if ((nbrDataGetNbr(nbrDataPtr, nbrPtr) == val1) || (nbrDataGetNbr(nbrDataPtr, nbrPtr) == val2)) {
			nbrListAddNbr(nbrListOutPtr, nbrPtr);
		}
	}
	return nbrListOutPtr;
}


/*
 * @brief Finds the neighbor with the minimum bearing in a list of neighbors
 *
 * @param nbrListInPtr pointer to the neighbor list to search
 * @returns pointer to the neighbor with the minimum bearing
 */
Nbr* nbrListGetNbrMinBearing(NbrList* nbrListInPtr) {
	uint8 i;
	int16 bearingMax = MILLIRAD_2PI;
	int16 bearing;
	Nbr* nbrPtr;
	Nbr* nbrMinPtr = NULL;

	for (i = 0; i < nbrListGetSize(nbrListInPtr); i++){
		nbrPtr = nbrListGetNbr(nbrListInPtr, i);
		bearing = nbrGetBearing(nbrPtr);
		if (bearing < bearingMax){
			nbrMinPtr = nbrPtr;
			bearingMax = bearing;
		}
	}
	return nbrMinPtr;
}

/*
 * @brief Get neighbor update round
 *
 * @param nbrPtr neighbor pointer
 * @returns the last round this neighbor was heard from
 */

void nbrListMakeCloneExcept(NbrList* nbrListOutPtr, uint8 nbrID) {
	uint8 i;
	Nbr* nbrPtr;
	NbrList nbrList;
	nbrListClear(nbrListOutPtr);
	nbrListCreate(&nbrList);
	for (i = 0; i < nbrListGetSize(&nbrList); ++i) {
		nbrPtr = nbrListGetNbr(&nbrList, i);
		if(nbrPtr->ID != nbrID){
			nbrListAddNbr(nbrListOutPtr, nbrPtr);
		}
	}
}

/*
 * @brief Sorts the nbr List in counter clockwise order by bearing.
 *
 * @param nbrListOutPtr pointer to output list of neighbors
 * @param nbrListInPtr pointer to input list of neighbors
 * @returns a sorted list of neighbors
 */
NbrList* nbrListSortByBearing(NbrList* nbrListOutPtr, NbrList* nbrListInPtr) {
	NbrList nbrListInTemp;
	Nbr* nbrPtr;

	nbrListClear(nbrListOutPtr);
	nbrListCopy(&nbrListInTemp, nbrListInPtr);

	while (nbrListGetSize(&nbrListInTemp) > 0){
		nbrPtr = nbrListGetNbrMinBearing(&nbrListInTemp);
		nbrListRemoveNbr(&nbrListInTemp, nbrPtr);
		nbrListAddNbr(nbrListOutPtr, nbrPtr);
	}
	return nbrListOutPtr;
}


/*
 * @brief Creates a list of neighbors that lie within the specified sector between angleStart and angleEnd
 *
 *
 * @param nbrListOutPtr pointer to the memory location where the generated list should be stored
 * @param nbrListInPtr pointer to the list of neighbors to check
 * @param angleStart starting bearing of the sector
 * @param angleEnd ending bearing of the sector
 * @returns the list of neighbors within the specified sector
 */
NbrList* nbrListIncludeSectorInclusive(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, int16 angleStart,  int16 angleEnd) {
	uint8 i;
	Nbr* nbrPtr;

	angleStart = normalizeAngleMilliRad(angleStart);
	angleEnd = normalizeAngleMilliRad(angleEnd);

	nbrListClear(nbrListOutPtr);

	if (angleStart == angleEnd) {
		return nbrListOutPtr;
	} else if (angleStart > angleEnd) {
		// angles cross the pi line.  you have two sectors
		for (i = 0; i < nbrListGetSize(nbrListInPtr); ++i) {
			nbrPtr = nbrListGetNbr(nbrListInPtr, i);
			if (nbrGetBearing(nbrPtr) >= angleStart) {
				nbrListAddNbr(nbrListOutPtr, nbrPtr);
			}
			if (nbrGetBearing(nbrPtr) <= angleEnd) {
				nbrListAddNbr(nbrListOutPtr, nbrPtr);
			}
		}
	} else {
		// angleStart < angleEnd
		for (i = 0; i < nbrListGetSize(nbrListInPtr); ++i) {
			nbrPtr = nbrListGetNbr(nbrListInPtr, i);
			if ((nbrGetBearing(nbrPtr) >= angleStart) && (nbrGetBearing(nbrPtr) <= angleEnd)) {
				nbrListAddNbr(nbrListOutPtr, nbrPtr);
			}
		}
	}
	return nbrListOutPtr;
}

/*
 * @brief Creates a list of neighbors that lie outside of the specified sector between angleStart and angleEnd
 *
 *
 * @param nbrListOutPtr pointer to the memory location where the generated list should be stored
 * @param nbrListInPtr pointer to the list of neighbors to check
 * @param angleStart starting bearing of the sector
 * @param angleEnd ending bearing of the sector
 * @returns the list of neighbors outside of the specified sector
 */
NbrList* nbrListIncludeSectorExclusive(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, int16 angleStart,  int16 angleEnd) {
	uint8 i;
	Nbr* nbrPtr;

	angleStart = normalizeAngleMilliRad(angleStart);
	angleEnd = normalizeAngleMilliRad(angleEnd);

	nbrListClear(nbrListOutPtr);

	if (angleStart == angleEnd) {
		return nbrListOutPtr;
	} else if (angleStart > angleEnd) {
		// angles cross the pi line.  you have two sectors
		for (i = 0; i < nbrListGetSize(nbrListInPtr); ++i) {
			nbrPtr = nbrListGetNbr(nbrListInPtr, i);
			if (nbrGetBearing(nbrPtr) > angleStart) {
				nbrListAddNbr(nbrListOutPtr, nbrPtr);
			}
			if (nbrGetBearing(nbrPtr) < angleEnd) {
				nbrListAddNbr(nbrListOutPtr, nbrPtr);
			}
		}
	} else {
		// angleStart < angleEnd
		for (i = 0; i < nbrListGetSize(nbrListInPtr); ++i) {
			nbrPtr = nbrListGetNbr(nbrListInPtr, i);
			if ((nbrGetBearing(nbrPtr) > angleStart) && (nbrGetBearing(nbrPtr) < angleEnd)) {
				nbrListAddNbr(nbrListOutPtr, nbrPtr);
			}
		}
	}
	return nbrListOutPtr;
}

