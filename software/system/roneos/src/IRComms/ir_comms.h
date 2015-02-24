/**
 * @file ir_comms.h
 * @brief Handles IR communication between robots using the ring of IR transmitters/receivers.
 * @since April 12, 2012
 * @author James McLurkin
 */

#ifndef IR_COMMS_H_
#define IR_COMMS_H_

/******** Defines ********/

#define IR_COMMS_MESSAGE_LENGTH_MAX         6
#define IR_COMMS_MESSAGE_BIT_LENGTH_MAX		(IR_COMMS_MESSAGE_LENGTH_MAX * 8)


#define IR_COMMS_NUM_OF_RECEIVERS           8
#define IR_COMMS_RECEIVER_FRONT_LEFT        0
#define IR_COMMS_RECEIVER_LEFT_FRONT        1
#define IR_COMMS_RECEIVER_LEFT_REAR         2
#define IR_COMMS_RECEIVER_REAR_LEFT         3
#define IR_COMMS_RECEIVER_REAR_RIGHT        4
#define IR_COMMS_RECEIVER_RIGHT_REAR        5
#define IR_COMMS_RECEIVER_RIGHT_FRONT       6
#define IR_COMMS_RECEIVER_FRONT_RIGHT       7

#define IR_COMMS_RECEIVER_FRONT_LEFT_BIT 	(1 << IR_COMMS_RECEIVER_FRONT_LEFT)
#define IR_COMMS_RECEIVER_LEFT_FRONT_BIT 	(1 << IR_COMMS_RECEIVER_LEFT_FRONT)
#define IR_COMMS_RECEIVER_LEFT_REAR_BIT 	(1 << IR_COMMS_RECEIVER_LEFT_REAR)
#define IR_COMMS_RECEIVER_REAR_LEFT_BIT 	(1 << IR_COMMS_RECEIVER_REAR_LEFT)
#define IR_COMMS_RECEIVER_REAR_RIGHT_BIT 	(1 << IR_COMMS_RECEIVER_REAR_RIGHT)
#define IR_COMMS_RECEIVER_RIGHT_REAR_BIT 	(1 << IR_COMMS_RECEIVER_RIGHT_REAR)
#define IR_COMMS_RECEIVER_RIGHT_FRONT_BIT 	(1 << IR_COMMS_RECEIVER_RIGHT_FRONT)
#define IR_COMMS_RECEIVER_FRONT_RIGHT_BIT 	(1 << IR_COMMS_RECEIVER_FRONT_RIGHT)

#define IR_COMMS_RECEIVER_0_ANGLE	        0.3927
#define IR_COMMS_RECEIVER_1_ANGLE	        1.1781
#define IR_COMMS_RECEIVER_2_ANGLE	        1.9635
#define IR_COMMS_RECEIVER_3_ANGLE	        2.7489
#define IR_COMMS_RECEIVER_4_ANGLE	        3.5343
#define IR_COMMS_RECEIVER_5_ANGLE	        4.3197
#define IR_COMMS_RECEIVER_6_ANGLE	        5.1051
#define IR_COMMS_RECEIVER_7_ANGLE	        5.8905

#define IR_COMMS_NUM_OF_TRANSMITTERS        8
#define IR_COMMS_POWER_MAX					255

/******** Structs ********/

/**
 * @brief Message received over IR ring (contains the sending transmitter ID and receiving ID)
 *
 * @warning If you are using the neighbor system, you have 8 less bits than
 * this says.
 */
typedef struct IRCommsMessage {
    uint8 data[IR_COMMS_MESSAGE_LENGTH_MAX];
    uint8 receiverBits;
    uint8 orientationBitMatrix[IR_COMMS_NUM_OF_RECEIVERS];
    uint32 rangeBits;	//### 32-bit range data
    uint32 timeStamp;
} IRCommsMessage;

/******** Functions ********/

/**
 * @brief Sets the size, in bits, of the IR comms message
 *
 * @param size the size of the data portion of the ir message.
 */
void irCommsSetSize(uint8 size);

/**
 * @brief Sets IR Comms xmit power.
 *
 * @param power range from 0-256 changes duty cycle from 0-100%
 * @returns void
 */
void irCommsSetXmitPower(uint8 power);


/**
 * @brief Sends out a message through IR transmitters if RC mode isn't active. Message is 1-5 bytes of data, first byte is robot ID
 *
 * @param irMessagePtr  pointer to the irCommsMessage struct that contains the message to be sent
 * @returns whether     the message is sent (TRUE/FALSE)
 */
boolean irCommsSendMessage(IRCommsMessage* irMessagePtr);


/**
 * @brief Gets a message through IR transmitters if RC mode not engaged.
 *
 * @param irMessagePtr pointer to the irCommsMessage struct that contains receiver information
 * @returns TRUE if there is a message, FALSE if not
 */
boolean irCommsGetMessage(IRCommsMessage* irMessagePtr);

////Sets the length of all the IR messages.  Allowable valued are from 1-5
//void ir_comms_set_message_length(uint8 msg_length);


/**
 * @brief Prints orientation bit matrix.
 *
 * @param orientationBitsMatrixPtr is the pointer to an orientation bit matrix
 * @param rangeBitsMatrixPtr
 * @returns void
 */
void irCommsOrientationBitMatrixPrint(uint8* orientationBitsMatrixPtr);


/*
 * @brief returns the number of bits received in a given ir message
 *
 * @param irMessagePtr the message received
 * @return number of bits received in message
 */
uint8 irCommsGetMessageReceiveBits(IRCommsMessage* irMessagePtr);

/**
 * @brief Initializes IRComms.
 *
 * Creates the OS message queues.
 * Initializes IR port for GPIO and set it as input.
 * Sets PWM pins; computes and sets pwm period based on system clock.
 * Enables PWM generators and output state.
 * Enables a 1250hz (800us) interrupt.
 * @returns void
 */
void irCommsInit(void);

/**
 * @brief compute range from a single IR message, but seems useless
 */
uint16 irCommsComputeNbrRange(uint32 rangeBits); //### 32-bit


#endif /* IR_COMMS_H_ */
