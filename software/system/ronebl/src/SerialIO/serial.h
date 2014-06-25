/**
 * @file serial.h
 *
 * @brief Serial UART communication functions
 * @since July 21, 2010
 * @author MRSL
 */

#ifndef SERIAL_H_
#define SERIAL_H_

/******** Defines ********/

//NOTE SERIAL_BUFFER_SIZE must be a power of 2 for the FIFO buffer pointers to work
#define SERIAL_BUFFER_SIZE 			(128)
#define SERIAL_BAUDRATE				(230400)
//#define SERIAL_BAUDRATE				(460800)
//#define SERIAL_BAUDRATE				(921600)

/******** Globals ********/

/******** Functions ********/

/**
 * 	@brief Initializes serial I/O.
 *
 * 	Enable the peripherals used by this example.
 * 	Enable processor interrupts.
 * 	Set GPIO A0 and A1 as UART pins.
 * 	Configure the UART for 115,200, 8-N-1 operation.
 *	Enable the UART interrupt.
 *	@returns void
 */
void serialInit(void);


/**
 * @brief Wait and get a byte from UART/USB with interrupt disabled.
 * Program is indefinitely blocked until there's an available byte.
 *
 * @return the byte value on success;
 */
int getSerialByte();


/**
 * @brief Wait and get a byte from UART/USB with interrupt disabled.
 * Program is blocked until there's an available byte or timeout.
 *
 * @param timeout Polling time before exiting (us)
 * @return the byte value on success; -1 if there's nothing in the buffer or timeout.
 */
int getSerialByteWithTimeout(uint32 timeout);


/**
 * @brief Check and get a byte from UART/USB with interrupt disabled.
 *
 * @return the byte value on success; -1 if there's nothing in the buffer.
 */
int getSerialByteNonblocking(void);


/**
 * @brief Send a byte through serial port FIFO (UART)
 * @param value Data to be sent
 * @returns void
 */
void sendByte(uint8 value);


#endif /* SERIAL_H_ */
