/**
 * @file radioCommand.h
 *
 * @brief Controls radio commands sent between robots.
 *
 * @since March 17, 2012
 * @author James McLurkin
 */

#ifndef RADIOCMD_H_
#define RADIOCMD_H_

/******** Defines ********/
// Command masks
#define RADIO_COMMAND_TYPE_MASK			0x3F
#define RADIO_COMMAND_SUBNET_MASK		0x03
#define RADIO_COMMAND_SUBNET_SHIFTS		6
#define RADIO_COMMAND_DEFAULT_SUBNET	0

// Bootloader commands
#define RADIO_BOOTLOADER_COMMANDS			10
#define RADIO_COMMAND_TYPE_PROGRAM_TIME		0x3F
#define RADIO_COMMAND_TYPE_CRC_TABLE		0x3E
#define RADIO_COMMAND_TYPE_QUERY_REQUEST	0x3D
#define RADIO_COMMAND_TYPE_SEGMENTS			0x3C

#define RADIO_COMMAND_TYPE_REBOOT			0x36



/******** Structs ********/
//TODO Add field documentation
/** @brief A radio command is a linked list of the radio commands received.
 */
typedef struct RadioCmd {
	uint8 type;			/**< */
	const char* name;	/**< */
	void (*funcPtr)(struct RadioCmd* radioCmdPtr, RadioMessage* message);	/**< */
	osQueueHandle messageQueue;	/**< */
	uint32 lastTimeStamp;		/**< */
	void* externalDataPtr;		/**< */
	struct RadioCmd* nextPtr;	/**< */
} RadioCmd;

/******** Functions ********/

/**
 * 	@brief Initializes radio command processing
 *
 *	@returns void
 */
void radioCommandInit(void);

/**
 * @brief Creates radio command that will call function.
 *
 * @param radioCmdPtr is pointer to specific radio command group to be sent
 * @param name is the name of the command
 * @param funcPtr points to function that radio message will be sent to
 * @returns void
 */
void radioCommandAddCallback(RadioCmd* radioCmdPtr, const char* name, void(*funcPtr)(RadioCmd* radioCmdPtr, RadioMessage* message));

/**
 * @brief Create radio command that uses a queue of specified size.
 *
 * @param radioCmdPtr is pointer to specific radio command
 * @param name is the name of the command
 * @param messageQueueSize is maximum size of queue - the number of
 * messages that it can hold, not the size of the message.  The radio
 * message size is RADIO_COMMAND_MESSAGE_DATA_LENGTH, currently
 * @returns void
 */
void radioCommandAddQueue(RadioCmd* radioCmdPtr, const char* name, uint8 messageQueueSize);

/**
 * @brief Creates a radio command that can call a function or a queue.
 *
 * @param radioCmdPtr is pointer to specific radio command group to be sent
 * @param name is the name of the command
 * @param funcPtr points to function that radio message will be sent to
 * @param messageQueueSize is maximum size of queue - the number of
 * messages that it can hold, not the size of the message.  The radio
 * message size is RADIO_COMMAND_MESSAGE_DATA_LENGTH, currently
 * @param externalDataPtr is a void* that can be set to whatever structure the user
 * needs access to during the callback.
 * @returns void
 */
void radioCommandAdd(RadioCmd* radioCmdPtr,
		const char* name,
		void(*funcPtr)(RadioCmd* radioCmdPtr, RadioMessage* message),
		uint8 messageQueueSize,
		void* externalDataPtr);

/**
 * @brief Receives specific radio commands.
 *
 * @param radioCmdPtr The command group of radio messages that will go into messagePtr
 * @param messagePtr  Will contain the next message for the given radio command group
 * @param maxDelay	  The maximum time the call will take before returning. If 0, will
 *                    not block (call radioCommandReceiveNonBlocking() instead)..
 * @return TRUE if there is a message. FALSE otherwise.
 */
boolean radioCommandReceive(RadioCmd* radioCmdPtr, RadioMessage* messagePtr, uint32 maxDelay);

/**
 * @brief Receives specific radio commands.
 *
 * @param radioCmdPtr The command group of radio messages that will go into messagePtr
 * @param messagePtr Will contain the next message for the given radio command group
 * @return TRUE if there is a message. FALSE otherwise.
 */
boolean radioCommandReceiveNonBlocking(RadioCmd* radioCmdPtr, RadioMessage* messagePtr);

/**
 * @brief Sends a radio message to the specified ID if RC mode is off.
 *
 * @param radioCmdPtr The command group of the radio message to be sent
 * @param destinationID Which robots the message is being sent to. 0xFF (ROBOT_ID_ALL)
 *                      sends to all robots
 * @param message the message to be sent
 */
void radioCommandXmit(RadioCmd* radioCmdPtr, uint8 destinationID, RadioMessage* message);

/**
 *@brief Check if time since last time stamp has exceeded timeout.
 *
 *@param radioCmdPtr The command group of the radio message to be sent
 *@param timeout is the amount of time in milliseconds before the function returns
 *@returns if timeout has been exceeded, return True; else, return False
 */
boolean radioCommandTimeout(RadioCmd* radioCmdPtr, uint32 timeout);


/**
 * @brief Get radio command type.
 *
 * @param messagePtr is the message to examine
 * @returns radio command message type
 */
uint8 radioCommandGetType(RadioMessage* messagePtr);

/**
 * @brief Get radio command subnet.
 *
 * @param messagePtr is the message to examine
 * @returns radio command message subnet
 */
uint8 radioCommandGetSubnet(RadioMessage* messagePtr);

/**
 * @brief Get radio command destination ID.
 *
 * @param messagePtr is the message to examine
 * @returns radio command destination robot ID
 */
uint8 radioCommandGetDestinationID(RadioMessage* messagePtr);


/**
 * @brief Sets the radio command subnet.
 *
 * @param subnet is the subnet to join. Range is from 0-3.
 * @returns void
 */
void radioCommandSetSubnet(uint8 subnet);

/**
 * @brief Get radio command data pointer.
 *
 * @param messagePtr is the message to examine
 * @returns pointer to data associated with the message
 */
char* radioCommandGetDataPtr(RadioMessage* messagePtr);


//TODO: implement, delete, or debug?
///**
// * @brief Get radio command destination ID.
// *
// * @param messagePtr will contain the next message for the given radio command group
// * @returns radio command message destination ID
// */
//uint8 radioCommandGetDestID(RadioMessageRaw* messagePtr);
//void radioCommandXmit_internal(RadioCmd* radioCmdPtr, uint8 destinationID, RadioMessageRaw* radioMsgPtr);
//boolean radioCommandReceive_internal(RadioCmd* radioCmdPtr, RadioMessageRaw* messagePtr, uint32 maxDelay);
//boolean radioCommandReceiveNonBlocking_internal(RadioCmd* radioCmdPtr, RadioMessageRaw* messagePtr);


#endif /* RADIOCMD_H_ */
