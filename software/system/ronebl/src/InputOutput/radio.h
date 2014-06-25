/**
 * @file radio.h
 * @brief Turns WiFi radio on or off, sends and receives radio messages, and radio interrupts.
 * @since Jul 9, 2010
 * @author MRSL
 */

#ifndef RADIO_H_
#define RADIO_H_

/******** Defines ********/
#define RADIO_SUBNET_ALL						0
#define RADIO_SUBNET_1							1
#define RADIO_SUBNET_2							2
#define RADIO_SUBNET_3							3

#define RADIO_BOOTLOADER_DEFAULT_SUBNET			RADIO_SUBNET_ALL

#define RADIO_MESSAGE_LENGTH_RAW    			32
#define RADIO_COMMAND_HEADER_LENGTH				2
#define RADIO_COMMAND_MESSAGE_DATA_LENGTH 		(RADIO_MESSAGE_LENGTH_RAW - RADIO_COMMAND_HEADER_LENGTH)
#define RADIO_PROGRAM_HEADER_LENGTH				4
// This must be multiple of 4
#define RADIO_PROGRAM_MESSAGE_DATA_LENGTH		(RADIO_MESSAGE_LENGTH_RAW - RADIO_PROGRAM_HEADER_LENGTH)


#define RADIO_COMMS_QUEUE_RECV_SIZE				8
#define RADIO_COMMS_QUEUE_XMIT_SIZE				1

// Radio Command def
#define RADIO_COMMAND_TYPE_MASK					0x3F
#define RADIO_COMMAND_SUBNET_MASK				0x03
#define RADIO_COMMAND_SUBNET_SHIFTS				6
#define RADIO_COMMAND_DEFAULT_SUBNET			0

// Radio Command Types (6-bit max)
// Bootloader commands
#define RADIO_BOOTLOADER_COMMANDS				10
#define RADIO_COMMAND_TYPE_PROGRAM_TIME_V11		0x3F
#define RADIO_COMMAND_TYPE_PROGRAM_TIME_V12		0x3E
#define RADIO_COMMAND_TYPE_PROGRAM_TIME_V13		0x3D
#define RADIO_COMMAND_TYPE_PROGRAM_TIME_V14		0x3C
#define RADIO_COMMAND_TYPE_CRC_TABLE			0x3B
#define RADIO_COMMAND_TYPE_QUERY_REQUEST		0x3A
#define RADIO_COMMAND_TYPE_SEGMENTS				0x39
// ...
#define RADIO_COMMAND_TYPE_REBOOT				0x36

extern uint8 bootloaderSubnet;

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


typedef struct RadioMessageProgram {
	uint8 linkQuality;								/**< */
    uint32 timeStamp;								/**< */
    uint8 type;										/**< */
    uint8 segment;									/**< */
    uint8 subsegment;								/**< */
    uint8 robotID;
    char data[RADIO_PROGRAM_MESSAGE_DATA_LENGTH];	/**< */ //TODO why not uint8?

} RadioMessageProgram;

/** @brief radio message and metrics union
 *
 * This is the union type for raw messages and commands.
 */
typedef union RadioMessage {
	RadioMessageRaw raw;			/**< A RadioMessageRaw structure*/
	RadioMessageCommand command;	/**< A RadioMessageCommand structure*/
	RadioMessageProgram program;
} RadioMessage;


/******** Functions ********/

/**
 * @brief Initializes the radio. (no interrupt)
 *
 * @returns void
 */
void radioInit(void);


/**
 * @brief Sends a message through the radio.
 *
 * @param messagePtr Pointer to the message to be sent. The message should be 32-bytes long
 *
 * @returns void
 */
void radioSendMessage(RadioMessage* messagePtr);


/*
 * @brief Checks for receive messages. The received message is copied into messagePtr and returns true.
 * if there is no message received, returns false.
 *
 * @param messagePtr Pointer to the RadioMessage that stores the received message
 *
 * @returns TRUE, if there is a received message; FALSE, if there is not.
 */
boolean radioRecvMessage(RadioMessage* messagePtr);


#endif /* RADIO_H_ */
