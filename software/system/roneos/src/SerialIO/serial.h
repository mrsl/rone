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

/* The remote control variables for the serial port. */
extern uint8 rcMode;
extern uint16 rcTimer;
#define RC_MODE_OFF					0
#define RC_MODE_ON					1
#define RC_TIMEOUT					350

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
 * 	@brief Buffers a character on the serial output buffer.
 *
 * 	Sends the character c to the transmit FIFO for the port specified by UARTO_BASE (base address).
 * 	@param c is the character to be transmitted
 * 	@returns void
 */
void sputchar(char c);

//TODO: These seem like the user shouldn't see them.
void sputcharFlush(void);
void sputcharTickHook(void);

/**
 * 	@brief Gets a character from serial receive buffer.
 *
 * 	@returns first character from serial receive buffer.
 */
int sgetchar(void);

//void serial_send_string(const char* string);

#endif /* SERIAL_H_ */
