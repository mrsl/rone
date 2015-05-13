/**
 * @file neighborListOps.h
 * @brief Operations on lists of neighbors
 * @details This includes adding/removing neighbors, searching for neighbors, and looking for robots with certain bearings.
 * @since September 13, 2011
 * @author James McLurkin
 */

#ifndef NEIGHBORLISTOPS_H_
#define NEIGHBORLISTOPS_H_


/**
 *  @brief Fills the provided nbrList with the current set of neighbors.
 *
 *  @param nbrListPtr for the list of neighbors
 *  @returns void
 */
void nbrListCreate(NbrList* nbrListPtr);


//TODO: implement or delete?
void groupList(NbrList* partialList, int group);
void groupTestList(NbrList* partialList, int group, NbrList* full);

//TODO: Functions from neighbors.c - move or not?
/**
 *  @brief Sets the size of nbrList to 0
 *
 *  @param nbrListPtr pointer for the list of neighbors to be altered
 *  @returns void
 */
void nbrListClear(NbrList* nbrListPtr);


/*
 * @brief Gets the size of the input neighbor list
 *
 * @param nbrListPtr pointer for the neighbor list
 * @returns size of neighbor list
 */
uint8 nbrListGetSize(NbrList* nbrListPtr);


/*
 * @brief Gets the neighbor at a given index in neighbor list
 *
 * @param nbrListPtr pointer for the neighbor list
 * @param idx index
 * @returns pointer to the neighbor at the input index
 */
Nbr* nbrListGetNbr(NbrList* nbrListPtr, uint8 idx);


/*
 * @brief Prints the IDs of all robots in the neighbor list.
 *
 * Formated as follows:
 * {name, ID, ID, ID, ...}
 * @param nbrListPtr pointer for the neighbor list
 * @param name for the list name
 * @returns void
 */
void nbrListPrint(NbrList* nbrListPtr, char* name);


/**
 * @brief Prints the IDs and update times for all robots in the neighbor list.
 *
 * Formated as follows:
 * {name, ID updateTime, ID updateTime, ...}
 * @param nbrListPtr pointer for the neighborlist
 * @param name for the list name
 * @returns void
 */
void nbrListPrintDebug(NbrList* nbrListPtr, char* name);


/**
 * @brief Adds a neighbor to a neighbor's linked list of neighbors if there is room.
 *
 * \internal
 * Shouldn't we return 0 on success, and -1 on failure?  I don't like voids (Aaron Becker)
 * \endinternal
 * @param nbrListPtr the neighbor's linked list
 * @param nbrPtr a new neighbor
 * @returns void
 */
void nbrListAddNbr(NbrList* nbrListPtr, Nbr* nbrPtr);


/**
 * @brief Removes nbrPtr from list (if it exists).
 *
 * \internal
 * shouldn't we return 0 on success, and -1 on failure?  I don't like voids (Aaron Becker)
 * \endinternal
 *
 * @param nbrListPtr list of neighbors
 * @param nbrPtr a neighbor to be removed
 * @returns void
 */
void nbrListRemoveNbr(NbrList* nbrListPtr, Nbr* nbrPtr);


/**
 * @brief Copies the neighbor list from SRC to DST.
 *
 * @param nbrListSrcPtr (unchanged)
 * @param nbrListDstPtr becomes a copy of nbrListSrcPtr
 * @returns void
 */
void nbrListCopy(NbrList* nbrListDstPtr, NbrList* nbrListSrcPtr);


/**
 * @brief Determines average bearing of all neighbors using sum of vectors method (optimized).
 *
 * @param nbrListPtr list of neighbors
 * @returns average bearing (milliradians)
 */
int16 nbrListAverageBearing(NbrList* nbrListPtr);


/**
 * @brief Returns a pointer to the neighbor at the head of the list, or NULL
 *
 * @param nbrListPtr pointer for the list of neighbors
 * @returns a pointer to the neighbor at the head of the list, or NULL
 */
Nbr* nbrListGetFirst(NbrList* nbrListPtr);


/**
 * @brief Returns a pointer to the neighbor second in the list, or NULL
 *
 * @param nbrListPtr pointer for the list of neighbors
 * @returns a pointer to the neighbor second in the list, or NULL
 */
Nbr* nbrListGetSecond(NbrList* nbrListPtr);


/**
 * @brief Returns a pointer to the neighbor with ID, or NULL.
 *
 * @param nbrListPtr pointer for the list of neighbors
 * @param ID the unique number of the robot being searched for
 * @returns a pointer to the neighbor with ID, or NULL
 */
Nbr* nbrListGetNbrWithID(NbrList* nbrListPtr, uint8 ID);


/**
 * @brief returns the neighbor whose bearing is closest to nbrPtr's bearing, NULL if list is empty.
 *
 * @param nbrListPtr list of neighbors
 * @param nbrPtr a neighbor used as a reference
 * @returns pointer to neighbor whose bearing is closest to nbrPtr's bearing, NULL if list is empty.
 */
Nbr* nbrListGetSmallestAngleDeviation2(NbrList* nbrListPtr, Nbr* nbrPtr);


/**
 * @brief returns the neighbor whose bearing is closest to bearing, NULL if list is empty.
 *
 * @param nbrListPtr list of neighbors
 * @param bearing a reference bearing
 * @returns pointer to neighbor whose bearing is closest to bearing, NULL if list is empty.
 */
Nbr* nbrListGetClosestNbrToBearing(NbrList* nbrListPtr, int16 bearing);


/**
 * @brief returns the neighbor who is closest, NULL if there is no neighbor
 *
 * @param nbrListPtr list of neighbors
 * @returns pointer to neighbor whose range is closest, NULL if list is empty.
 */
Nbr* nbrListGetClosestNbr(NbrList* nbrListPtr);


/**
 * @brief Computes the union of the two input neighbor lists.
 *
 * @param nbrListOutPtr pointer to the list created from the union
 * @param nbrList1Ptr first neighbor list
 * @param nbrList2Ptr second neighbor list
 * @returns a pointer to a neighbor list that is the union of the two input lists
 */
NbrList* nbrListUnion(NbrList* nbrListOutPtr, NbrList* nbrList1Ptr, NbrList* nbrList2Ptr);


/**
 * @brief returns a pointer to the list of neighbors with the same data value
 *
 * @param nbrListOutPtr pointer to output list of neighbors
 * @param nbrListInPtr pointer to input list of neighbors
 * @param nbrDataPtr list of data on messages
 * @param val value of data to match
 * @returns a pointer to the output list of neighbors with the same data value
 */
NbrList* nbrListFindNbrsWithDataEqual(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val);


/**
 * @brief returns a pointer to the list of neighbors with the same data values
 *
 * @param nbrListOutPtr pointer to output list of neighbors
 * @param nbrListInPtr pointer to input list of neighbors
 * @param nbrDataPtr list of data on messages
 * @param val1 first value of data to match
 * @param val2 second value of data to match
 * @returns a pointer to the output list of neighbors with the same data values
 */
NbrList* nbrListFindNbrsWithDataEqual2(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, NbrData* nbrDataPtr, uint8 val1, uint8 val2);


/**
 * @brief Finds the neighbor with the minimum bearing in a list of neighbors
 *
 * @param nbrListInPtr pointer to the neighbor list to search
 * @returns pointer to the neighbor with the minimum bearing
 */
Nbr* nbrListGetNbrMinBearing(NbrList* nbrListInPtr);


/**
 * @brief Sorts the nbr List in counter clockwise order by bearing.
 *
 * @param nbrListOutPtr pointer to output list of neighbors
 * @param nbrListInPtr pointer to input list of neighbors
 * @returns a sorted list of neighbors
 */
NbrList* nbrListSortByBearing(NbrList* nbrListOutPtr, NbrList* nbrListInPtr);


/**
 * @brief Creates a list of neighbors that lie within the specified sector between angleStart and angleEnd
 *
 *
 * @param nbrListOutPtr pointer to the memory location where the generated list should be stored
 * @param nbrListInPtr pointer to the list of neighbors to check
 * @param angleStart starting bearing of the sector
 * @param angleEnd ending bearing of the sector
 * @returns the list of neighbors within the specified sector
 */
NbrList* nbrListIncludeSectorInclusive(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, int16 angleStart,  int16 angleEnd);


/**
 * @brief Creates a list of neighbors that lie outside of the specified sector between angleStart and angleEnd
 *
 *
 * @param nbrListOutPtr pointer to the memory location where the generated list should be stored
 * @param nbrListInPtr pointer to the list of neighbors to check
 * @param angleStart starting bearing of the sector
 * @param angleEnd ending bearing of the sector
 * @returns the list of neighbors outside of the specified sector
 */
NbrList* nbrListIncludeSectorExclusive(NbrList* nbrListOutPtr, NbrList* nbrListInPtr, int16 angleStart,  int16 angleEnd);


/**
 * @brief Finds neighbors that are robots
 *
 * @param nbrListOutPtr pointer to the neighbor list updated by this function
 * @param nbrListInPtr pointer to the neighbor list to search for robots
 * @returns updated nbrListOutPtr that contains only neighbors that are robots
 */
NbrList* nbrListGetRobots(NbrList* nbrListOutPtr, NbrList* nbrListInPtr);


void nbrListMakeCloneExcept(NbrList* nbrListOutPtr, uint8 nbrID);

#endif /* NEIGHBORLISTOPS_H_ */
