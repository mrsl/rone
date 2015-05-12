/**
 * @file serialCommand.h
 *
 * @brief Serial UART communication functions
 * @since July 21, 2010
 * @author MRSL
 */

#ifndef SERIALCMD_H_
#define SERIALCMD_H_

/******** Defines ********/

#define SERIAL_INPUT_STRING_SIZE 				128


/******** Struct ********/

/**
 * @brief Serial commands are a linked list of commands containing the message and name
 */
typedef struct SerialCmd {
	char* name;
	void (*funcPtr)(char* message);
	struct SerialCmd* nextPtr;
	uint32 timestamp;
} SerialCmd;

/******** Functions ********/

/**
 * @brief Add serial command to linked list.
 *
 * @param serialCmdPtr pointer to serial command to be added
 * @param name name of serial command
 * @param funcPtr function pointer to function that will be executed when command is sent to serial port
 * @returns void
 */
void serialCommandAdd(SerialCmd* serialCmdPtr, char* name, void(*funcPtr)(char* message));


/**
 * @brief Get the command data from the serial string.  Removes the command prefix.
 *
 * @param command pointer to serial command string
 * @returns the pointer to the command text without the command prefix
 */
char* serialCommandRemovePrefix(char* command);


/**
 * @brief Get the time this command was last called
 *
 * @param serialCmdPtr pointer to serial command to be added
 * @returns tick time of last call
 */
uint32 serialCommandGetTimestamp(SerialCmd* serialCmdPtr);


/**
 * 	@brief Initializes serial command processing
 *
 *	@returns void
 */
void serialCommandInit(void);

#endif /* SERIALCMD_H_ */
