/*
 * @file radioCmd.c
 *
 *  Created on: Mar 17, 2012
 *      Author: Sunny Kim
 */

#include <string.h>
#include "roneos.h"

// Only 6-bits available for type, and 10 are reserved for bootloader
#define RADIO_COMMAND_TYPE_MAX				(64 - RADIO_BOOTLOADER_COMMANDS)
#define RADIO_COMMAND_RECEIVE_DELAY			10

//extern RadioCmd radioCmdGUI;
//extern osQueueHandle radioCommsQueueRecv;


void radioSendMessage(RadioMessage *messagePtr);
void radioGetMessageBlocking(RadioMessage *messagePtr);

//extern void (*radioSendMessageStatic)(RadioMessageRaw *);
//extern void (*radioGetMessageBlockingStatic)(RadioMessageRaw *);
//extern static void radioSendMessage(RadioMessageRaw* messagePtr);
//static void radio_send_message_rc(RadioMessageRaw* messagePtr);
//extern static void radioGetMessageBlocking(RadioMessageRaw* messagePtr);
//static void radio_get_message_blocking_rc(RadioMessageRaw* messagePtr);
//static boolean radio_get_message(RadioMessageRaw* messagePtr);
//static boolean radio_get_message_rc(RadioMessageRaw* messagePtr);


/******** variables ********/

static RadioCmd* radioCmdStartPtr = NULL;
static uint8 radioCmdType = 1;
static uint8 radioCommandSubnet;


/******** functions ********/

static RadioCmd* radioCommandFind(uint8 type) {
	RadioCmd* radioCmdPtr = radioCmdStartPtr;
	while (radioCmdPtr != NULL) {
		if (type == radioCmdPtr->type) {
			//we have found the command on our linked list.  return
			break;
		} else {
			radioCmdPtr = radioCmdPtr->nextPtr; // search through next radioCmd ptr
		}
	}
	return radioCmdPtr;
}


void radioCommandAdd(RadioCmd* radioCmdPtr,
		const char* name,
		void(*funcPtr)(RadioCmd* radioCmdPtr, RadioMessage* message),
		uint8 messageQueueSize,
		void* externalDataPtr) {
	RadioCmd* radioCmdTempPtr;

	if (radioCmdType >= RADIO_COMMAND_TYPE_MAX) {
		//we don't have room for this new command. error and return.
		error("too many radio commands");
		return;
	}

	// we have space for the new command.  populate the command.
	radioCmdPtr->type = radioCmdType++;
	radioCmdPtr->name = name;
	radioCmdPtr->externalDataPtr = externalDataPtr;
	radioCmdPtr->lastTimeStamp = 0;
	radioCmdPtr->funcPtr = funcPtr;
	if (!funcPtr && (messageQueueSize > 0)) {
		radioCmdPtr->messageQueue = osQueueCreate(messageQueueSize,
				sizeof(RadioMessage));
	} else {
		radioCmdPtr->messageQueue = NULL;
	}
	radioCmdPtr->nextPtr = NULL;

	// put the command on the linked list
	if (radioCmdStartPtr == NULL) {
		// if radioCmdStart is null set it to radioCmdPtr
		radioCmdStartPtr = radioCmdPtr;
	} else {
		radioCmdTempPtr = radioCmdStartPtr;
		while (radioCmdTempPtr->nextPtr != NULL) {
			radioCmdTempPtr = radioCmdTempPtr->nextPtr;
		}
		radioCmdTempPtr->nextPtr = radioCmdPtr;
	}
}


void radioCommandAddCallback(RadioCmd* radioCmdPtr,const char* name,void(*funcPtr)(RadioCmd* radioCmdPtr,RadioMessage* message)) {
	radioCommandAdd(radioCmdPtr, name, funcPtr, 0, NULL);
}


void radioCommandAddQueue(RadioCmd* radioCmdPtr,
				const char* name,
				uint8 messageQueueSize) {
	radioCommandAdd(radioCmdPtr, name, NULL, messageQueueSize, NULL);
}

static void radioCommandTask(void* parameters) {
	RadioMessage message;
	uint8 type, subnet, destID;
	//uint8 destinationID;
	RadioCmd* radioCmdPtr;
	portBASE_TYPE val;

	while (TRUE) {
		// wait for a radio message
		radioGetMessageBlocking(&message);
		// check the subnet.  are we on the same subnet?
		subnet = radioCommandGetSubnet(&message);
		if(subnet != radioCommandSubnet) {
			// wrong subnet. discard
			continue;
		}
		// check to see if this message is for you
		destID = radioCommandGetDestinationID(&message);
		if((destID != roneID) && (destID != ROBOT_ID_ALL)) {
			// message is not for me.  discard
			continue;
		}

		type = radioCommandGetType(&message);
		radioCmdPtr = radioCommandFind(type);
		//if (radioCmdPtr && ((destinationID == roneID) || (destinationID == ROBOT_ID_ALL))) {
		if (radioCmdPtr) {
			radioCmdPtr->lastTimeStamp = message.command.timeStamp;
			//we have found the command on our list. call the function with the rest of the message
			if (radioCmdPtr->funcPtr) {
				(radioCmdPtr->funcPtr)(radioCmdPtr, &message);
			} else if (radioCmdPtr->messageQueue) {
				// no callback function.  queue the command on the command queue for manual reads
				//TODO handle errors
				val = osQueueSend(radioCmdPtr->messageQueue, (void*)(&message), 0);
			}
		}
	}
}


boolean radioCommandReceive_internal(RadioCmd* radioCmdPtr, RadioMessage* messagePtr, uint32 maxDelay) {
	portBASE_TYPE val;
	if (radioCmdPtr->messageQueue) {
		val = osQueueReceive(radioCmdPtr->messageQueue, (void*)(messagePtr), maxDelay);
		if (val == pdPASS) {
			val = TRUE;
		} else {
			val = FALSE;
		}
	} else {
		val = FALSE;
	}
	return val;
}

/* header information */
boolean radioCommandReceive(RadioCmd* radioCmdPtr, RadioMessage* messagePtr, uint32 maxDelay) {
	/* This will be checked against elapsed time if RC mode is engaged to exit after the
	 * right amount of time has passed.
	 */
	uint32 timer = 0;

	/* OS time get until it exceeds maxDelay then return false. */

	/* If
	 * IF RC mode is on, delay for 10ms until more timer has passed than the
	 * user specified or rc mode is disengaged.
	 */
	while (rcMode == RC_MODE_ON) {
		osTaskDelay(RADIO_COMMAND_RECEIVE_DELAY);
		timer += RADIO_COMMAND_RECEIVE_DELAY; //Shouldn't add delay, should instead get time elapsed from the OS and add that. !!!!!!!
		if (timer >= maxDelay) {
			return FALSE;
		}
	}
	/* Call the RC command to get the message. Subtract how much time has elapsed
	 * waiting for the RC mode to disengage so it delays for the right amount of
	 * time.
	 */
	return radioCommandReceive_internal(radioCmdPtr, messagePtr, maxDelay - timer);
}


boolean radioCommandReceiveNonBlocking_internal(RadioCmd* radioCmdPtr, RadioMessage* messagePtr) {
	//TODO: RADIO: All of the RC stuff shouldn't be defined in headers, only in the top of the files that need them.
	portBASE_TYPE val;
	if (radioCmdPtr->messageQueue) {
		val = osQueueReceive(radioCmdPtr->messageQueue, (void*)(messagePtr), 0);
		if (val == pdPASS) {
			val = TRUE;
		} else {
			val = FALSE;
		}
	} else {
		val = FALSE;
	}
	return val;
}

boolean radioCommandReceiveNonBlocking(RadioCmd* radioCmdPtr, RadioMessage* messagePtr) {
	portBASE_TYPE val;
	if (rcMode == RC_MODE_ON) {
		val = FALSE;
	} else {
		val = radioCommandReceiveNonBlocking_internal(radioCmdPtr, messagePtr);
	}
	return val;
}


void radioCommandXmit_internal(RadioCmd* radioCmdPtr, uint8 destinationID, RadioMessage* messagePtr) {
	messagePtr->command.type = (radioCmdPtr->type & RADIO_COMMAND_TYPE_MASK) | (radioCommandSubnet << RADIO_COMMAND_SUBNET_SHIFTS);
	messagePtr->command.destinationID = destinationID;
	radioSendMessage(messagePtr);
}


void radioCommandXmit(RadioCmd* radioCmdPtr, uint8 destinationID, RadioMessage* messagePtr) {
	if (rcMode == RC_MODE_OFF) {
		radioCommandXmit_internal(radioCmdPtr, destinationID, messagePtr);
	}
}


boolean radioCommandTimeout(RadioCmd* radioCmdPtr, uint32 timeout) {
	if ((osTaskGetTickCount() - radioCmdPtr->lastTimeStamp) > timeout) {
		return TRUE;
	} else {
		return FALSE;
	}
}


uint8 radioCommandGetType(RadioMessage* messagePtr) {
	//return messagePtr->data[RADIO_COMMAND_MESSAGE_TYPE_IDX];
	return (messagePtr->command.type & RADIO_COMMAND_TYPE_MASK);
}


uint8 radioCommandGetSubnet(RadioMessage* messagePtr) {
	//return messagePtr->data[RADIO_COMMAND_MESSAGE_TYPE_IDX];
	return (messagePtr->command.type >> RADIO_COMMAND_SUBNET_SHIFTS);
}


uint8 radioCommandGetDestinationID(RadioMessage* messagePtr) {
	//return messagePtr->data[RADIO_COMMAND_MESSAGE_TYPE_IDX];
	return (messagePtr->command.destinationID);
}


//uint8 radioCommandGetDestID(RadioMessage* messagePtr) {
//	return messagePtr->data[RADIO_COMMAND_MESSAGE_DESTID_IDX];
//}

char* radioCommandGetDataPtr(RadioMessage* messagePtr) {
	return (char*)&(messagePtr->command.data);
}


void radioCommandSetSubnet(uint8 subnet) {
	radioCommandSubnet = subnet & RADIO_COMMAND_SUBNET_MASK;
}


void radioCommandInit(void) {
	// Make a thread to process command strings
	radioCommandSetSubnet(RADIO_COMMAND_DEFAULT_SUBNET);
	osTaskCreate(radioCommandTask, "radioCommand", 1024, NULL, RADIOCOMMAND_TASK_PRIORITY);
	radioCmdStartPtr = NULL;
}
