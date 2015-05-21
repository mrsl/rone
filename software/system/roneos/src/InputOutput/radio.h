/**
 * @file radio.h
 * @brief Turns WiFi radio on or off, sends and receives radio messages, and radio interrupts.
 * @since Jul 9, 2010
 * @author MRSL
 */
//TODO: sjb2 is the real author...who is this?

#ifndef RADIO_H_
#define RADIO_H_

/******** Defines ********/

#define RADIO_MESSAGE_LENGTH_RAW    		32
#define RADIO_COMMAND_HEADER_LENGTH				2
#define RADIO_COMMAND_MESSAGE_DATA_LENGTH 		(RADIO_MESSAGE_LENGTH_RAW - RADIO_COMMAND_HEADER_LENGTH)

#define RADIO_COMMS_QUEUE_RECV_SIZE		8
#define RADIO_COMMS_QUEUE_XMIT_SIZE		3

// Subnets
#define RADIO_SUBNET_ALL				0
#define RADIO_SUBNET_1					1
#define RADIO_SUBNET_2					2
#define RADIO_SUBNET_3					3



/******** Structs ********/

/** @brief radio message and metrics
 *
 * This it the raw radio message.  It should only be used at the low level, or for network snooping
 * Note: This must be the same length as the RadioMessageCommand for
 * the union to work properly.
 */
//TODO Add field descriptions
typedef struct RadioMessageRaw {
	uint8 linkQuality;						/**< */
    uint32 timeStamp;						/**< */
    char data[RADIO_MESSAGE_LENGTH_RAW];	/**< */
} RadioMessageRaw;


/** @brief radio message and metrics
 *
 * This is the radio message command.  This is processed by the threads and filtered
 * for type and subnet.
 * Note: This must be the same length as the RadioMessageRaw for
 * the union to work properly.
 */
//TODO Add field descriptions
typedef struct RadioMessageCommand {
	uint8 linkQuality;								/**< */
    uint32 timeStamp;								/**< */
    uint8 type;										/**< */
    uint8 destinationID;							/**< */
    char data[RADIO_COMMAND_MESSAGE_DATA_LENGTH];	/**< */
} RadioMessageCommand;


/** @brief radio message and metrics union
 *
 * This is the union type for raw messages and commands.
 */
typedef union RadioMessage {
	RadioMessageRaw raw;			/**< A RadioMessageRaw structure*/
	RadioMessageCommand command;	/**< A RadioMessageCommand structure*/
} RadioMessage;

/******** Functions ********/

/**
 * @brief Initializes the radio.
 *
 * @returns void
 */
void radioInit(void);


/**
 * @brief Enables radio interrupt.
 *
 * @returns void
 */
void radioIntEnable(void);


/**
 * @brief Disables radio interrupt.
 *
 * @returns void
 */
void radioIntDisable(void);

void radioPrintCounters(void);


#endif /* RADIO_H_ */
