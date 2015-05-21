/*
 * nbrMsgRadio.c
 *
 *  Created on: Nov 17, 2012
 *      Author: jamesm
 */

#include <string.h>
#include "roneos.h"
#include "neighborsInternal.h"

/******** Defines ********/


/******** Variables ********/
static NbrMsgRadio* nbrMsgRadioStartPtr = NULL;
static uint8 neighborMessageRadioCount = 0;


/******** Functions ********/

#if defined(RONE_IRBEACON)
void nbrMsgRadioAddNbr(uint8 ID) {
}

void nbrMsgRadioRemoveNbr(uint8 ID) {
}

void nbrMsgRadioXmit(void) {
}

void nbrMsgRadioCreate(NbrMsgRadio* nbrMsgRadioPtr, const char* name){
}

NbrMsgRadioNbrData* nbrMsgRadioGetNbr(NbrMsgRadio* nbrMsgRadioPtr, Nbr* nbrPtr){
}

#elif defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12)

// a neighbor message was received.  store the radio message in the neighbor radio message struct
static void nbrMsgRadioCallback(RadioCmd* radioCmdPtr, RadioMessage* radioMsgPtr) {
	uint8 i;
	char* radioCmdDataPtr;
	uint8 senderID, length;
	NbrMsgRadio* nbrMsgRadioPtr;
	NbrMsgRadioNbrData* nbrDataPtr;

	//TODO need to get the neighbor system mutex.
	//TODO We'll run free and loose for debugging, then tighten up later

	// first, get the pointer to the radio message struct tat was used to create this message type
	nbrMsgRadioPtr = (NbrMsgRadio*)(radioCmdPtr->externalDataPtr);

	// get the address of (get a pointer to) the data in the radio message
	radioCmdDataPtr = radioCommandGetDataPtr(radioMsgPtr);

	// the senderID and length are the first and second bytes of the message.  Pull them off
	senderID = radioCmdDataPtr[NBR_RADIO_MESSAGES_SENDER_ID_IDX];
	length = radioCmdDataPtr[NBR_RADIO_MESSAGES_LENGTH_IDX];

	// look through the neighbor list for the sender of this message
	for (i = 0; i < NEIGHBOR_MAX; ++i) {
		nbrDataPtr = &(nbrMsgRadioPtr->dataNbr[i]);
		if(nbrDataPtr->ID == senderID) {
			// the sender of this radio message is on our list of neighbors in the radio message struct. update the data
			nbrDataPtr->timeStamp = osTaskGetTickCount();
			nbrDataPtr->length = length;
			memcpy(nbrDataPtr->data, &radioCmdDataPtr[NBR_RADIO_MESSAGES_DATA_IDX], length);
			break;
		}
	}
}


// xmit the neighbor radio messages during the neighbor period
void nbrMsgRadioXmit(void) {
	NbrMsgRadio* nbrMsgRadioPtr = nbrMsgRadioStartPtr;
	RadioMessage radioMsg;
	char* radioMsgDataPtr = radioCommandGetDataPtr(&radioMsg);
	uint8 i;

	while (nbrMsgRadioPtr != NULL) {
		radioMsgDataPtr[NBR_RADIO_MESSAGES_SENDER_ID_IDX] = roneID;
		radioMsgDataPtr[NBR_RADIO_MESSAGES_LENGTH_IDX] = nbrMsgRadioPtr->length;
		memcpy(&radioMsgDataPtr[NBR_RADIO_MESSAGES_DATA_IDX], nbrMsgRadioPtr->data, nbrMsgRadioPtr->length);
		//TODO ugly hack to help nbr data get through.  send each packet multiple times
#if 0
		for (i = 0; i < 3; ++i) {
			radioCommandXmit(&nbrMsgRadioPtr->radioCmd, ROBOT_ID_ALL, &radioMsg);
		}
#endif
		radioCommandXmit(&nbrMsgRadioPtr->radioCmd, ROBOT_ID_ALL, &radioMsg);
		nbrMsgRadioPtr = nbrMsgRadioPtr->nextPtr; // search through next radioCmd ptr
	}
}


/*
 * @brief Create a radio neighbor message.
 *
 * This function is called only once per program execution for each thing.
 * @param fieldPtr the pointer to the radio message
 * @param name the name for the radio message
 * @returns void
 */
void nbrMsgRadioCreate(NbrMsgRadio* nbrMsgRadioPtr, const char* name){
	NbrMsgRadio* nbrMsgRadioTempPtr;
	uint8 i;

	if (neighborMessageRadioCount >= NBR_RADIO_MESSAGES_MAX) {
		error("too many neighbor radio messages");
		return;
	}
	// enough space.  create and add the radio message
	nbrMsgRadioPtr->name = name;
	nbrMsgRadioPtr->length = 0;
	nbrMsgRadioPtr->nextPtr = NULL;

	for (i = 0; i < NEIGHBOR_MAX; ++i) {
		nbrMsgRadioPtr->dataNbr[i].ID = 0;
		nbrMsgRadioPtr->dataNbr[i].length = 0;
		nbrMsgRadioPtr->dataNbr[i].data[0] = 0;
	}
	radioCommandAdd(&(nbrMsgRadioPtr->radioCmd), name, nbrMsgRadioCallback, 0, nbrMsgRadioPtr);

	// put the command on the linked list
	if (nbrMsgRadioStartPtr == NULL) {
		// if list is empty, start with the current radio message
		nbrMsgRadioStartPtr = nbrMsgRadioPtr;
	} else {
		nbrMsgRadioTempPtr = nbrMsgRadioStartPtr;
		while (nbrMsgRadioTempPtr->nextPtr != NULL) {
			nbrMsgRadioTempPtr = nbrMsgRadioTempPtr->nextPtr;
		}
		nbrMsgRadioTempPtr->nextPtr = nbrMsgRadioPtr;
	}
	neighborMessageRadioCount++;
}


void nbrMsgRadioAddNbr(uint8 ID) {
	NbrMsgRadio* nbrMsgRadioPtr = nbrMsgRadioStartPtr;
	uint8 i;

	while (nbrMsgRadioPtr != NULL) {
		for (i = 0; i < NEIGHBOR_MAX; ++i) {
			if(nbrMsgRadioPtr->dataNbr[i].ID == ROBOT_ID_NULL) {
				// we found an empty slot
				nbrMsgRadioPtr->dataNbr[i].ID = ID;
				break;
			}
		}
		nbrMsgRadioPtr = nbrMsgRadioPtr->nextPtr; // search through next radioCmd ptr
	}
}


void nbrMsgRadioRemoveNbr(uint8 ID) {
	NbrMsgRadio* nbrMsgRadioPtr = nbrMsgRadioStartPtr;
	uint8 i;

	while (nbrMsgRadioPtr != NULL) {
		for (i = 0; i < NEIGHBOR_MAX; ++i) {
			if(nbrMsgRadioPtr->dataNbr[i].ID == ID) {
				// we found the right neighbor.  clear the ID, which marks the data area as empty
				nbrMsgRadioPtr->dataNbr[i].ID = ROBOT_ID_NULL;
				break;
			}
		}
		nbrMsgRadioPtr = nbrMsgRadioPtr->nextPtr; // search through next radioCmd ptr
	}
}


/*
 * @brief Gets a radio neighbor message.
 *
 * This function returns a pointer to the NbrMsgRadioNbrData struct for this neighbor
 * @param nbrMsgRadioPtr the pointer to the radio message
 * @param nbrPtr pointer to the neighbor
 * @returns pointer to data or NULL
 */
NbrMsgRadioNbrData* nbrMsgRadioGetNbr(NbrMsgRadio* nbrMsgRadioPtr, Nbr* nbrPtr){
	uint8 i;
	NbrMsgRadioNbrData* dataPtr = NULL;

	if (nbrMsgRadioPtr != NULL) {
		for (i = 0; i < NEIGHBOR_MAX; ++i) {
			if(nbrMsgRadioPtr->dataNbr[i].ID == nbrPtr->ID) {
				// we found the right neighbor.  return the message data
				dataPtr = &(nbrMsgRadioPtr->dataNbr[i]);
				break;
			}
		}
	}
	return dataPtr;
}

/*
 *  @brief Prints the message data for the input neighbor.
 *
 *  @param nbrMsgRadioPtr pointer for the linked list of a neighbor's messages
 *  @param nbrPtr pointer for the desired neighbor
 *  @returns void
 */
void nbrMsgRadioPrint(NbrMsgRadio* nbrMsgRadioPtr, Nbr* nbrPtr){
	uint8 i;
	NbrMsgRadioNbrData* dataPtr = NULL;

	if (nbrMsgRadioPtr != NULL) {
		for (i = 0; i < NEIGHBOR_MAX; ++i) {
			if(nbrMsgRadioPtr->dataNbr[i].ID == nbrPtr->ID) {
				// we found the right neighbor.  print the message data
				dataPtr = &(nbrMsgRadioPtr->dataNbr[i]);
				cprintf("nbr %2d %s=\"%s\"", nbrGetID(nbrPtr), nbrMsgRadioPtr->name, dataPtr->data);
				break;
			}
		}
	}
}


/*
 * @brief Gets a radio neighbor message.
 *
 * This function returns a pointer to the NbrMsgRadioNbrData struct for this neighbor
 * @param nbrMsgRadioPtr the pointer to the radio message
 * @param dataPtr pointer to a char array of max length RADIO_COMMAND_MESSAGE_DATA_LENGTH
 * @returns pointer to data or NULL
 */
void nbrMsgRadioSet(NbrMsgRadio* nbrMsgRadioPtr, char* dataPtr, uint8 length){
	if (nbrMsgRadioPtr != NULL) {
		if (length > NBR_RADIO_MESSAGE_DATA_LENGTH) {
			length = NBR_RADIO_MESSAGE_DATA_LENGTH;
		}
		memcpy(nbrMsgRadioPtr->data, dataPtr, length);
		nbrMsgRadioPtr->length = length;
	}
}
#endif
