/*
 * @file serial.c
 *
 * @brief Serial UART communication functions
 * @since July 21, 2010
 * @author MRSL
 */

//TODO: Real author sjb2?

#include <stdio.h>

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"

#include "roneos.h"

#define SERIAL_UART_FIFO_SIZE			16

#define SPUTCHAR_FLUSH_TIME				10
#define SPUTCHAR_BUFFER_MAX_DELAY		10

uint8 sputcharTimer = 0;

#define SERIAL_RX_BUFFER_SIZE 			SERIAL_BUFFER_SIZE
#define SERIAL_RX_BUFFER_MASK			((uint8)(SERIAL_RX_BUFFER_SIZE - 1))
char serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
uint8 serial_rx_buffer_out = 0;
uint8 serial_rx_buffer_in = 0;

#define SERIAL_TX_BUFFER_SIZE 			SERIAL_BUFFER_SIZE
#define SERIAL_TX_BUFFER_MASK			((uint8)(SERIAL_TX_BUFFER_SIZE - 1))
char serial_tx_buffer[SERIAL_TX_BUFFER_SIZE];
uint8 serial_tx_buffer_out = 0;
uint8 serial_tx_buffer_in = 0;

/* Remote control variables. */
uint8 rcMode;
uint16 rcTimer;

#define SERIAL_MODE_COMMAND	 			0
#define SERIAL_MODE_BINAY	 			1

#define SERIAL_MAX_COUNT				128
#define SERIAL_INITIAL_COUNT			0


uint8 serialMode = SERIAL_MODE_COMMAND;
osSemaphoreHandle serialCommandSemaphore;

/**
 * 	@brief Sends out a character. (DEPRICATED - has hard to solve race condition problems, and is not used by cprintf)
 *
 * 	Sends the character c to the transmit FIFO for the port specified by UARTO_BASE (base address).
 * 	@param c is the character to be transmitted
 * 	@returns void
 */
//void sputchar(char c) {
//	// first, buffer the char.  If the interrupt happens, it will still go out
//	if (serial_tx_buffer_in != ((uint8)(serial_tx_buffer_out - 1) & SERIAL_TX_BUFFER_MASK)) {
//		serial_tx_buffer[serial_tx_buffer_in++] = c;
//		serial_tx_buffer_in &= SERIAL_TX_BUFFER_MASK;
//	}
//
//	// check to see if the uart is not busy.  If not, then load up the fifo to start the process.
//	if (!MAP_UARTBusy(UART0_BASE)) {
//		MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);
//		//c = serial_tx_buffer[serial_tx_buffer_out++];
//		//serial_tx_buffer_out &= SERIAL_TX_BUFFER_MASK;
//		//MAP_UARTCharPut(UART0_BASE, c);
//
//		// fill up the xmit fifo
//		while (MAP_UARTSpaceAvail(UART0_BASE)) {
//			if (serial_tx_buffer_out != serial_tx_buffer_in) {
//				// Take the next character from the xmit buffer and increment the out index
//				c = serial_tx_buffer[serial_tx_buffer_out++];
//				serial_tx_buffer_out &= SERIAL_TX_BUFFER_MASK;
//				MAP_UARTCharPutNonBlocking(UART0_BASE, c);
//			} else {
//				break;
//			}
//		}
//		MAP_UARTIntEnable(UART0_BASE, UART_INT_TX);
//	}
//}


/*
 * 	@brief Buffers a character on the serial output buffer.
 *
 * 	Sends the character c to the transmit FIFO for the port specified by UARTO_BASE (base address).
 * 	@param c is the character to be transmitted
 * 	@returns void
 */
void sputchar(char c) {
	uint8 i;
	// buffer the char xmit buffer.
	// This call does not start a uart xmit, but does set the timer for a flush
	for (i = 0; i < SPUTCHAR_BUFFER_MAX_DELAY; ++i) {
		if (serial_tx_buffer_in != ((uint8)(serial_tx_buffer_out - 1) & SERIAL_TX_BUFFER_MASK)) {
			break;
		} else {
			osTaskDelay(1);
		}
	}
	if(i < SPUTCHAR_BUFFER_MAX_DELAY) {
		MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);
		taskENTER_CRITICAL();
		serial_tx_buffer[serial_tx_buffer_in++] = c;
		serial_tx_buffer_in &= SERIAL_TX_BUFFER_MASK;
		sputcharTimer =  SPUTCHAR_FLUSH_TIME;
		taskEXIT_CRITICAL();
		MAP_UARTIntEnable(UART0_BASE, UART_INT_TX);
	} else {
		error("ERROR: sputchar failed");
		//TODO set error flag.  Can't call error here, because we can't print
	}
}


// note - this needs to be called with systick and uart interrupts disabled
// this can happen from within an interrupt context, or by manually disabling the interrupts
static void sputcharBufferFlushInternal(void) {
	char c;
	while (MAP_UARTSpaceAvail(UART0_BASE) && (serial_tx_buffer_out != serial_tx_buffer_in)) {
		// Take the next character from the xmit buffer and increment the out index
		c = serial_tx_buffer[serial_tx_buffer_out++];

		// wrap the pointer around
		serial_tx_buffer_out &= SERIAL_TX_BUFFER_MASK;

		// send the char
		MAP_UARTCharPutNonBlocking(UART0_BASE, c);
	}
}

/*
 * 	@brief Flushes the buffer and starts the serial xmit
 *
 * 	loads the UART fifo from the RAM buffer.  This starts a xmit.
 * 	@returns void
 */
void sputcharFlush(void) {
	// fill up the xmit fifo to start transmitting
	MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);
	taskENTER_CRITICAL();
	sputcharBufferFlushInternal();
	sputcharTimer = 0;
	taskEXIT_CRITICAL();
	MAP_UARTIntEnable(UART0_BASE, UART_INT_TX);
}


// this function is called from the systick interrupt, so keep it short.
// flush the serial buffer if needed.  note this function must be carefully mutexed
void sputcharTickHook(void) {
	if(sputcharTimer > 0) {
		sputcharTimer--;
		if(sputcharTimer == 0) {
			// flush the uart buffer
			sputcharBufferFlushInternal();
		}
	}
}


/*
 * 	@brief Gets a character from serial receive buffer.
 * 	@returns first character from serial receive buffer.
 */
int sgetchar(void) {
	int c;

	if (serial_rx_buffer_out != serial_rx_buffer_in) {
		// Take the next character from the queue and increment the out index
		c = serial_rx_buffer[serial_rx_buffer_out++];
		serial_rx_buffer_out &= SERIAL_RX_BUFFER_MASK;
	} else {
		c = -1;
	}

	return c;
}

/*
 * @brief Interrupt that handles bytes coming through serial port.
 *
 * @returns void
 */
void uartIRQHandler(void) {
	unsigned long ulStatus;
	int c;
	uint8 i;
	portBASE_TYPE taskWoken;

	// Get the interrupt status.
	ulStatus = MAP_UARTIntStatus(UART0_BASE, true);

	// Clear the asserted interrupts.
	MAP_UARTIntClear(UART0_BASE, ulStatus);

	if(ulStatus & (UART_INT_RX | UART_INT_RT)) {
		// receive chars.  read them from the low-level fifo
		// Loop while there are characters in the receive FIFO.
		for (i = 0; i < SERIAL_UART_FIFO_SIZE; i++) {
			// Read the next character from the UART
			c = UARTCharGetNonBlocking(UART0_BASE);
			if (c < 0) {
				//no more chars.  break out of the loop
				break;
			}
			if ((c == '\r') || (c == '\n')) {
				// Set a semphore to indicate that there might be a crlf in the buffer
				if (serialMode == SERIAL_MODE_COMMAND) {
					osSemaphoreGiveFromISR(serialCommandSemaphore, &taskWoken);
				}
			}

			// check to see if the receive buffer is full.  if not, then put the char on it.
			if (serial_rx_buffer_in != ((uint8)(serial_rx_buffer_out - 1) & SERIAL_RX_BUFFER_MASK)) {
				serial_rx_buffer[serial_rx_buffer_in++] = (char)c;
				serial_rx_buffer_in &= SERIAL_RX_BUFFER_MASK;
			} else {
				// serial buffer overflow
				/* If there is overflow caused by the GUI, it should trigger the breakpoint here. */
				c++;
				// have to respect the size of the buffer
				error("ERROR: Serial buffer overflow occured.");
			}
		}
	}

	if(ulStatus & (UART_INT_TX)) {
		// transmit interrupt.  The fifo is below the desired limit.  fill it up from the xmit buffer
		while (MAP_UARTSpaceAvail(UART0_BASE)) {
			if (serial_tx_buffer_out != serial_tx_buffer_in) {
				// Take the next character from the xmit buffer and increment the out index
				c = serial_tx_buffer[serial_tx_buffer_out++];
				serial_tx_buffer_out &= SERIAL_TX_BUFFER_MASK;
				MAP_UARTCharPutNonBlocking(UART0_BASE, c);
			} else {
				// no more chars in the xmit buffer.  let the fifo finish the xmit
				break;
			}
		}
	}

	// execute a context switch to start the serial command thread if we got a crlf
	/*
	 * TODO: QUILLAN: Read about this in the manual.
	 *
	 * forces a check for a context switch
	 */
	portEND_SWITCHING_ISR(taskWoken);
}


/*
 * 	@brief Initializes serial I/O.
 *
 * 	Enable the peripherals used by this example.
 * 	Enable processor interrupts.
 * 	Set GPIO A0 and A1 as UART pins.
 * 	Configure the UART for 115,200, 8-N-1 operation.
 *	Enable the UART interrupt.
 *	@returns void
 */
void serialInit() {
	serialMode = SERIAL_MODE_COMMAND;

	// Make a semaphore to synchronize the serial interrupt with the command processor
	//osSemaphoreCreateBinary(serialCommandSemaphore);
	if ((serialCommandSemaphore = osSemaphoreCreateCounting(SERIAL_MAX_COUNT,
			SERIAL_INITIAL_COUNT)) == NULL) {
		error("ERROR: Insufficient heap memory available for FreeRTOS to allocate"
				"semaphore.");
	}

	/* Set rc mode to off initially. */
	rcMode = RC_MODE_OFF;
	rcTimer = 0;

	// Enable the peripherals.
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Set GPIO A0 and A1 as UART pins.
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 230,400, 8N1 operation.
	MAP_UARTConfigSetExpClk(UART0_BASE, SYSCTL_CLOCK_FREQ, SERIAL_BAUDRATE,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	// Disable the UART fifos (for double-buffered usage)
	//UARTFIFODisable(UART0_BASE);

	// set the fifos for 1/8 empty on transmit and 3/4 full on receive
	MAP_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX6_8);

	// Enable the UART interrupts.
	MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
	MAP_IntPrioritySet(INT_UART0, SYSTEM_INTERRUPT_PRIORITY);
	MAP_IntEnable(INT_UART0);
}
