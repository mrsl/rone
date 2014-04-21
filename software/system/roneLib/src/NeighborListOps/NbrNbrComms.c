/*
 * @file NbrNbrComms.c
 * @brief The neighbor communication provides an API for
 *  accessing data about a robot's neighbors and information on those neighbor's neighbors
 *
 *  @details  this code implements a class-type object and may be difficult to understand.
 *  @since Nov 13, 2012
 *  @author jamesm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "roneos.h"
#include "ronelib.h"

/******** Defines ********/
#define NBRNBR_LENGTH	3
#define MILLIRAD_BYTE_CONV	25


/******** Variables ********/
// radio message to store the nbr nbr data
NbrMsgRadio nbrRadioMsgNbrNbr;

/******** Functions ********/
/*
 * @brief update this robot's nbrnbr data
 *
 *
 * @param ndPtr pointer to neighbor's data
 * @returns void
 */
static void nbrNbrUpdate(NbrDatabase* ndPtr) {
	char msg[NBR_RADIO_MESSAGE_DATA_LENGTH];
	Nbr* nbrPtr;
	uint8 i;
	uint8 j = 0;

	for (i = 0; i < ndPtr->nbrsSize; ++i) {
		nbrPtr = &(ndPtr->nbrs[i]);
		msg[j++] = nbrGetID(nbrPtr);
		msg[j++] = milliradToByte(nbrGetBearing(nbrPtr));
		msg[j++] = milliradToByte(nbrGetOrientation(nbrPtr));
	}
	nbrMsgRadioSet(&nbrRadioMsgNbrNbr, msg, j);
}

/*
 * @brief Initializes messages for broadcasting and adds a callback.
 *
 * @returns void
 */
void nbrNbrInit(void) {
	nbrMsgRadioCreate(&nbrRadioMsgNbrNbr, "nbrNbr");
	neighborsAddReceiveCallback(nbrNbrUpdate);
}


/*
 * \internal
 * @brief Get list of neighbor's neighbors.
 *
 * Incomplete.
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @param nbrNbrList pointer to where to place list
 * @returns void
 * \endinternal
 */
void nbrNbrListGetFromNbr(Nbr* nbrPtr, NbrNbrList* nbrNbrList){
	NbrMsgRadioNbrData* nbrMsgPtr;
	uint8 i;
	uint8 j = 0;

	if (nbrPtr) {
		nbrMsgPtr = nbrMsgRadioGetNbr(&nbrRadioMsgNbrNbr, nbrPtr);
		if (nbrMsgPtr) {
			for (i = 0; i < nbrMsgPtr->length; ) {
				nbrNbrList->nbrNbrs[j].ID = nbrMsgPtr->data[i++];
				nbrNbrList->nbrNbrs[j].bearing = byteToMillirad((int8)(nbrMsgPtr->data[i++]));
				nbrNbrList->nbrNbrs[j++].orientation = byteToMillirad((int8)(nbrMsgPtr->data[i++]));
			}
			nbrNbrList->updateTime = nbrMsgPtr->timeStamp;
		}
	}
	nbrNbrList->size = j;
	nbrNbrList->nbrPtr = nbrPtr;
}

/*
 * @brief Gets the size of this neighbor's "neighbor list."
 *
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @returns number of neighbors, 0 if empty
 */
uint8 nbrNbrListGetSize(NbrNbrList* nbrNbrListPtr) {
	if (nbrNbrListPtr) {
		return nbrNbrListPtr->size;
	} else {
		return 0;
	}
}


/*
 * @brief Print information on neighbor (and information of neighbor's neighbors).
 *
 * Print roneID and neighbor's ID, bear, orientation, orientation valid
 * If neighbor_nbrnbr_enable is true, print ID and bearing of each neighbor's neighbor.
 * Print name and value of each neighbor field.
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @returns void
 */
void nbrNbrListPrint(NbrNbrList* nbrNbrListPtr) {
	uint8 i;
	NbrNbr* nbrNbrPtr;

	if (nbrNbrListPtr->nbrPtr) {
		cprintf("nbr%2d: nbrnbrs={", nbrGetID(nbrNbrListPtr->nbrPtr));
		for (i = 0; i < nbrNbrListGetSize(nbrNbrListPtr); ++i) {
			nbrNbrPtr = nbrNbrListGetNbrAtIdx(nbrNbrListPtr, i);
			cprintf("%2d: brg %4d ont %4d,",
					nbrNbrGetID(nbrNbrPtr), nbrNbrGetBearing(nbrNbrPtr), nbrNbrGetOrientation(nbrNbrPtr));
		}
		cprintf("}\n");

		//for orientation data collection
		//	cprintf("ndo, %1d, %1d, %1d, %1d", roneID, nbr->ID, nbr->orientation, nbr->bearing);
		//	cprintf("ndo, end");
	}
}

/*
 * @brief Returns pointer to the neighbors at the input index of this neighborList
 *
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @param idx index
 * @returns pointer to the neighbors at index idx of this neighborList, NULL if list is empty
 */
NbrNbr* nbrNbrListGetNbrAtIdx(NbrNbrList* nbrNbrListPtr, uint8 idx) {
	if (idx < nbrNbrListPtr->size) {
		return &nbrNbrListPtr->nbrNbrs[idx];
	} else {
		return NULL;
	}
}


/*
 * @brief returns pointer to the neighbor with id ID of this nbr nbr List
 *
 * @param nbrNbrListPtr list of the neighbors of this neighbor
 * @param ID robot ID
 * @returns pointer to the neighbor with input ID, or NULL
 */
NbrNbr* nbrNbrListGetNbrWithID(NbrNbrList* nbrNbrListPtr, uint8 id) {
	uint8 i;
	NbrNbr* nbrNbrTempPtr;
	NbrNbr* nbrNbrPtr = NULL;

	for (i = 0; i < nbrNbrListGetSize(nbrNbrListPtr); ++i) {
		nbrNbrTempPtr = nbrNbrListGetNbrAtIdx(nbrNbrListPtr, i);
		if (nbrNbrGetID(nbrNbrTempPtr) == id) {
			nbrNbrPtr = nbrNbrTempPtr;
			break;
		}
	}
	return nbrNbrPtr;
}


/******** nbrNbr getters ********/

/*
 * @brief Get ID of neighbor's neighbor.
 *
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @returns the ID of neighbor's neighbor
 */
uint8 nbrNbrGetID(NbrNbr* nbrNbrPtr){
	if(nbrNbrPtr) {
		return nbrNbrPtr->ID;
	} else {
		return 0;
	}
}

/*
 * @brief Get bearing of neighbor's neighbor.
 *
 * @warning Incomplete.
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @returns bearing of neighbor's neighbor
 */
int16 nbrNbrGetBearing(NbrNbr* nbrNbrPtr){
	if(nbrNbrPtr) {
		return nbrNbrPtr->bearing;
	} else {
		return 0;
	}
}

/*
 * @brief Get orientation of neighbor's neighbor.
 *
 * @param nbrNbrPtr pointer to neighbor's neighbor
 * @returns orientation of neighbor's neighbor
 */
int16 nbrNbrGetOrientation(NbrNbr* nbrNbrPtr){
	if(nbrNbrPtr) {
		return nbrNbrPtr->orientation;
	} else {
		return 0;
	}
}


//TODO commented out.  Need to rework with neighbor data fields
//		if (nbr_id & 0x80) {
//			// neighbor nbr message
//			//serial_send_string_crlf("recv nbrnbr msg");
//			uint8 nbr_nbr_id;
//			uint16 nbr_nbr_bearing;
//			nbr_id = nbr_id & 0x7F;
//
//            nbrPtr = nbrsGetWithID(nbr_id);
//            if (nbrPtr) {
//                nbr_nbr_id = irMsgPtr->data[1];
//                nbr_nbr_bearing = byteToMillirad(irMsgPtr->data[2]);
//            	nbrUpdateNbrNbr(nbrPtr, nbr_nbr_id, nbr_nbr_bearing, irMsgPtr->timeStamp);
//				#if defined(RONE_V6)
//					if (nbr_nbr_id == roneID) {
//						// you are the nbrnbr - compute orientation of this robot
//						nbrPtr->orientation = nbr_nbr_bearing;
//						nbrPtr->orientationValid = TRUE;
//					}
//				#endif
//            }
//		} else {
			// neighbor message




//static void nbrUpdateNbrNbr(Nbr* nbrPtr, uint8 nbr_nbr_id, int16 nbr_nbr_bearing, uint32 currentTime) {
//	NbrNbr* nbrNbrPtr = nbrGetNbrNbrWithID(nbrPtr, nbr_nbr_id);
//    if (!nbrNbrPtr) {
//    	// make a new nbr nbr
//    	if (nbrPtr->nbrnbrsSize < NEIGHBOR_MAX) {
//    		nbrNbrPtr = &(nbrPtr->nbrnbrs[nbrPtr->nbrnbrsSize]);
//    		nbrNbrPtr->ID = nbr_nbr_id;
//    		nbrPtr->nbrnbrsSize++;
//    	}
//    }
//	if (nbrNbrPtr) {
//		nbrNbrPtr->bearing = nbr_nbr_bearing;
//		nbrNbrPtr->updateTime = currentTime;
//	}
//}



////timeout old nbr nbrs
//if (neighbor_nbrnbr_enable){
//	for (i = 0; i < nd.nbrsSize; i++) {
//		nbrPtr = &nd.nbrs[i];
//		for (j = 0; j < nbrPtr->nbrnbrsSize; ) {
//			if(currentTime > (nbrPtr->nbrnbrs[j].updateTime + neighbor_timeout)) {
//				nbrPtr->nbrnbrsSize--;
//				if (j < nbrPtr->nbrnbrsSize) {
//					nbrPtr->nbrnbrs[j] = nbrPtr->nbrnbrs[nbrPtr->nbrnbrsSize];
//				}
//			}else{
//				j++;
//			}
//		}
//	}
//}


// send the nbr nbr messages
//TODO commented out until the neighbor data fields are finished
//			if (neighbor_nbrnbr_enable){
//				for (i = 0; i < nd.nbrsSize; i++) {
//					ir_msg.data[0] = roneID | 0x80;
//					ir_msg.data[1] = nd.nbrs[i].ID;
//					ir_msg.data[2] = milliradToByte(nd.nbrs[i].bearing);
//					irCommsSendMessage(&ir_msg);
//				}
//			}


/*
 * @brief Get neighbor pointer of neighbor's specific neighbor.
 *
 * @param nbrPtr neighbor pointer
 * @param nbrNbrID neighbor's neighbor's ID
 * @returns neighbor pointer
 */
//NbrNbr* nbrGetNbrNbrWithID(Nbr* nbrPtr, uint8 nbrNbrID) {
//	uint8 i;
//	for (i = 0; i < nbrPtr->nbrnbrsSize; i++) {
//		if (nbrPtr->nbrnbrs[i].ID == nbrNbrID) {
//			return &(nbrPtr->nbrnbrs[i]);
//		}
//	}
//	return NULL;
//}


