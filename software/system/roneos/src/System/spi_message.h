/**
 * @file spi_message.h
 * @brief SPI commands for the MSP430
 * @since Apr 2, 2012
 * @author James McLurkin
 */

#ifndef MSP430_H_
#define MSP430_H_

/******** Defines ********/

#define MSP430_PAYLOAD_LENGTH	22
#define EXPAND0_PAYLOAD_LENGTH	3


/******** Functions ********/

/*
 *  @brief Initialize MSP430
 *
 * This is a system function not a user function.  Do not call this from user code.
 *
 * use 400us interrupt for SPI comms.  This lets us xmit our 24-byte message in 9.6
 * ms.  At the end, we use a 32us interrupt for SPI comms.  This is about as fast
 * as the MSP430 can read bytes from the buffer.  This double byte signals the end of the
 * message for synchronizing the two processors
 * @returns void
*/
void msp430Init(void);


/**
 * @brief Enable MSP430 interrupt.
 *
 * @returns void
 */
void msp430InterruptEnable(void);


/**
 * @brief Disable MSP430 interrupt.
 *
 * @returns void
 */
void msp430InterruptDisable(void);


/**
 * @brief Build a command for MSP430 message and reset command.
 *
 * Used to build the command for the actual command sent to the MSP430
 * Reset the command to normal mode
 * @returns void
 */
void msp430SystemCommandBuildMessage(uint8* msg);


/**
 * @brief Set the command to shutdown.
 *
 * @returns void
 */
void msp430SystemShutdownCommand(void);


/**
 * @brief Set the next command to the MSP430 to be to enter the boot-loader.
 *
 * @returns void
 */
void msp430SystemReprogramCommand(void);


/**
 * @brief Get the command.
 *
 * @returns systemMSP430 the command
 */
uint8 msp430SystemGetCommandMessage(void);


/**
 * @brief Gets systemMSP430CommsValid
 *
 * @returns systemMSP430CommsValid
 */
boolean msp430GetCommsValid(void);


/**
 * @brief Sets systemMSP430CommsValid
 * @param newCommsValid new boolean value to set systemMSP430CommsValid
 * @returns void
 */
void msp430SetCommsValid(boolean newCommsValid);

/**
 * @brief Initialize expand0 SPI
 * @returns void
 */
void expand0Init(void);

/**
 * @brief Enables expand0 SPI
 *
 * @returns void
 */
void expand0Enable();


/**
 * @brief Disables expand0 SPI
 *
 * @returns void
 */
void expand0Disable();


/**
 * @brief Export incoming payload from expand0 board
 * @param msg Destination for the message export (no code and checksum, length = EXPAND0_PAYLOAD_LENGTH)
 * @returns void
 */
void expand0MsgRead(uint8* msg);


/**
 * @brief Prepare the next outgoing message to be sent out
 * @param msg Source for the message export (no code and checksum, length = EXPAND0_PAYLOAD_LENGTH)
 * @returns void
 */
void expand0MsgWrite(uint8* msg);

#endif /* MSP430_H_ */
