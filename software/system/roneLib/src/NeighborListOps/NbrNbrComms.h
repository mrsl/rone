/**
 * @file NbrNbrComms.h
 * @brief The neighbor communication provides an API for accessing data about a robot's neighbors
 * and information on those neighbor's neighbors.
 *
 * @details This code implements a class-type object and may be difficult to understand.
 *
 * @since November 13, 2012
 * @author James McLurkin
 */

#ifndef NBRNBRCOMMS_H_
#define NBRNBRCOMMS_H_


/** @brief All relevant data about a neighbor's neighbor
 *
 */
typedef struct NbrNbr {
	uint8 ID;			/**< Neighbor's ID number */
	int16 bearing;		/**< Robot bearing */
	int16 orientation;	/**< Robot orientation */
} NbrNbr;


/** @brief Array of neighbor's neighbors.
 *
 */
//TODO make field descriptions
typedef struct NbrNbrList {
	Nbr* nbrPtr;					/**< Pointer to Neighbor*/
	uint8 size;						/**< Size*/
	uint32 updateTime;				/**< Update Time */
	NbrNbr nbrNbrs[NEIGHBOR_MAX];	/**< */
} NbrNbrList;


/**
 * @brief Initializes messages for broadcasting and adds a callback.
 *
 * @returns void
 */
void nbrNbrInit(void);
//void nbrNbrUpdate(void);

//NbrNbr* nbrGetNbrNbrWithID(Nbr* nbrPtr, uint8 nbrNbrID);


/**
 *
 * @brief Get list of neighbor's neighbors.
 *
 *
 * @param nbrPtr pointer to neighbor's neighbor
 * @param nbrNbrList pointer to where to place list
 * @returns void
 *
 */
void nbrNbrListGetFromNbr(Nbr* nbrPtr, NbrNbrList* nbrNbrList);


/**
 * @brief Gets the size of this neighbor's "neighbor list."
 *
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @returns number of neighbors, 0 if empty
 */
uint8 nbrNbrListGetSize(NbrNbrList* nbrNbrListPtr);


/**
 * @brief Print information on neighbor (and information of neighbor's neighbors).
 *
 * @details Print roneID and neighbor's ID, bear, orientation, orientation valid
 * If neighbor_nbrnbr_enable is true, print ID and bearing of each neighbor's neighbor.
 * Print name and value of each neighbor field.
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @returns void
 */
void nbrNbrListPrint(NbrNbrList* nbrNbrListPtr);


//TODO: implement or delete?
uint32 nbrNbrListGetUpdateTime(NbrNbrList* nbrNbrList);


/**
 * @brief Returns pointer to the neighbors at the input index of this neighborList
 *
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @param idx index
 * @returns pointer to the neighbors at index idx of this neighborList, NULL if list is empty
 */
NbrNbr* nbrNbrListGetNbrAtIdx(NbrNbrList* nbrNbrListPtr, uint8 idx);


/**
 * @brief Returns pointer to the neighbor with id of this nbr nbr List.
 *
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @param id robot ID
 * @returns pointer to the neighbor with input ID, or NULL
 */
NbrNbr* nbrNbrListGetNbrWithID(NbrNbrList* nbrNbrListPtr, uint8 id);


/**
 * @brief Get ID of neighbor's neighbor.
 *
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @returns the ID of neighbor's neighbor
 */
uint8 nbrNbrGetID(NbrNbr* nbrNbrPtr);


/**
 * @brief Get bearing of neighbor's neighbor.
 *
 * @warning Incomplete.
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @returns bearing of neighbor's neighbor
 */
int16 nbrNbrGetBearing(NbrNbr* nbrNbrPtr);


/**
 * @brief Get orientation of neighbor's neighbor.
 *
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @returns orientation of neighbor's neighbor
 */
int16 nbrNbrGetOrientation(NbrNbr* nbrNbrPtr);



#endif /* NBRNBRCOMMS_H_ */
