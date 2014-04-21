/*
 * @file serialCommand.c
 * @brief Processes serial commands and links them with desired function.
 * @since Mar 17, 2012
 * @author Sunny Kim
 */

#include <string.h>

#include "roneos.h"

#define SERIAL_COMMAND_LENGTH		2

extern osSemaphoreHandle serialCommandSemaphore;

static char prevCRLF = 0;
static char prevCRLFCount = 0;
static char serialCommandString[SERIAL_INPUT_STRING_SIZE];
static uint8 serialCommandStringIdx = 0;

static SerialCmd* serialCmdStart = NULL;


static SerialCmd* serialCommandFind(char* cmd) {
	SerialCmd* serialCmdPtr = serialCmdStart;
	while(serialCmdPtr != NULL) {
		if (strncmp(cmd, serialCmdPtr->name, SERIAL_COMMAND_LENGTH) == 0) {
			//we have found the command on our linked list.  return
			break;
		} else {
			serialCmdPtr = serialCmdPtr->nextPtr; // search through next serialCmd ptr
		}
	}
	return serialCmdPtr;
}

/*
 * @brief Get the time this command was last called
 *
 * @param serialCmdPtr pointer to serial command to be added
 * @returns tick time of last call
 */
uint32 serialCommandGetTimestamp(SerialCmd* serialCmdPtr) {
	return serialCmdPtr->timestamp;
}


/*
 * @brief Add serial command to linked list.
 *
 * @param serialCmdPtr pointer to serial command to be added
 * @param name name of serial command
 * @param funcPtr function pointer to function that will be executed when command is sent to serial port
 * @returns void
 */
void serialCommandAdd(SerialCmd* serialCmdPtr, char* name, void(*funcPtr)(char* message)) {
	SerialCmd* serialCmdTempPtr;

	if (strlen(name) != SERIAL_COMMAND_LENGTH) {
		//checking if name of serial command is correct size (2 characters). error and return.
		error("serial command not the correct length");
		return;
	}

	serialCmdTempPtr = serialCommandFind(name);
	if (serialCmdTempPtr) {
		//we have found the command on our linked list. error and return.
		error("duplicate serial command");
		return;
	}

	// not a duplicate.  populate the command.
	serialCmdPtr->name = name;
	serialCmdPtr->funcPtr = funcPtr;
	serialCmdPtr->nextPtr = NULL;

	// put the command on the linked list
	if (serialCmdStart == NULL) {
		// if serialCmdStart is null set it to serialCmdPtr
		serialCmdStart = serialCmdPtr;
	} else {
		serialCmdTempPtr = serialCmdStart;
		while (serialCmdTempPtr->nextPtr != NULL) {
			serialCmdTempPtr = serialCmdTempPtr->nextPtr;
		}
		serialCmdTempPtr->nextPtr = serialCmdPtr;
	}
}


static void serialCmdCall(char* message) {
	SerialCmd* serialCmdPtr;

	if (strlen(message) < SERIAL_COMMAND_LENGTH) {
		//we have less than two characters, no valid command, abort
		error("invalid command");
		return;
	}

	serialCmdPtr = serialCommandFind(message);
	if (serialCmdPtr) {
		//we have found the command on our linked list. call the function with the rest of the message
		serialCmdPtr->timestamp = osTaskGetTickCount();
		// pass the characters after the command to the message
		(serialCmdPtr->funcPtr)(message);
	}
}

// We provide no way to read strings directly.  Only through command processing.
// Not sure if this is the right API, time will tell...
///**
// * Get a string message.  Does not return the newline
// * this function will block
// */
//void serialGetString(char* string) {
//	osQueueReceive(serialStringQueueRecv, string, portMAX_DELAY);
//}
//
//
///**
// * Get a string message up to the newline.
// * Returns true or false, depending on the message was received or not.
// */
//boolean serialGetStringNonBlocking(char* string) {
//	portBASE_TYPE val;
//
//	val = osQueueReceive(serialStringQueueRecv, string, 0);
//    if (val == pdPASS) {
//        return TRUE;
//    } else {
//    	string[0] = 0;
//        return FALSE;
//    }
//}


/*
 * @brief Pull characters out of the low-level serial buffer.
 * Process CRLF silliness, then put the processed messages onto a queue
 * needs to be called periodically.  Called from heartbeat thread at 50hz
 * with 256 byte buffer @ 115200bps, the buffer will fill in 0.022 sec
 * should be OK...
 *
 * TODO: Is it still at 115200bps???
 */
static boolean serialStringUpdate(void) {
	int val;
	char c;
	boolean returnVal = FALSE;

	do {
		val = sgetchar();
		if (val != -1) {
			c = (char)val;

			// process cr/lf silliness
			if ((c == '\r') || (c == '\n')) {
				if ((prevCRLF == 0) || (prevCRLF == c)){
					// We have a new string: either with a new crlf char, or a repeated cr or lf
					serialCommandString[serialCommandStringIdx] = 0;
					serialCommandStringIdx = 0;
					prevCRLF = c;
					returnVal = TRUE;
					break;
				} else {
					// your prev char was a cr or lf, but not the same as c.  ignore this one
					// this deals with crlf or lfcr
					prevCRLF = 0;
				}
			} else {
				if(serialCommandStringIdx < (SERIAL_INPUT_STRING_SIZE - 1)) {
					serialCommandString[serialCommandStringIdx++] = c;
				} else {
					serialCommandString[serialCommandStringIdx] = 0;
				}
				prevCRLF = 0;
			}
		}
	//} while (c != -1);
	} while (val != -1); //TODO: how could this have previously worked?
	return returnVal;
}


static void serialCommandTask(void* parameters) {
//	osSemaphoreTake(serialCommandSemaphore, portMAX_DELAY);

	while(TRUE) {
		// wait for a potential crlf
		osSemaphoreTake(serialCommandSemaphore, portMAX_DELAY);

		// process the serial data.  Is there a command in here?
		if (serialStringUpdate()) {
			serialCmdCall(serialCommandString);
		}
	}
}


/*
 * 	@brief Initializes serial command processing
 *
 *	@returns void
 */
void serialCommandInit() {
	// Make a thread to process serial command strings
	osTaskCreate(serialCommandTask, "serialCommand", 2048, NULL, SERIALIO_TASK_PRIORITY);
}
