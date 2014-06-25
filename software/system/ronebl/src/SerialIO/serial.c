/*
 * @file serial.c
 *
 * @brief Serial UART communication functions
 * @since July 21, 2010
 * @author MRSL
 */
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

#include "System/bootloader.h"

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
	// Enable the peripherals.
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Set GPIO A0 and A1 as UART pins.
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 230,400, 8N1 operation.
	MAP_UARTConfigSetExpClk(UART0_BASE, SYSCTL_CLOCK_FREQ, SERIAL_BAUDRATE,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

//	MAP_UARTFIFOEnable(UART0_BASE);
//	// set the fifos for 1/8 empty on transmit and 3/4 full on receive
//	MAP_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX1_8);

	// Enable the UART interrupts (receive only).
	MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
//	MAP_IntPrioritySet(INT_UART0, SYSTEM_INTERRUPT_PRIORITY);
//	MAP_IntEnable(INT_UART0);
}


/*
 * @brief Wait and get a byte from UART/USB with interrupt disabled.
 * Program is indefinitely blocked until there's an available byte.
 *
 * @return the byte value on success;
 */
int getSerialByte() {
	uint32 i;

	while (1) {
		if (UARTCharsAvail(UART0_BASE)) {
			return UARTCharGetNonBlocking(UART0_BASE);
		}
		systemDelayTenMicroSecond(1);
	}

	// Control should not reach here
	return -1;
}


/*
 * @brief Wait and get a byte from UART/USB with interrupt disabled.
 * Program is blocked until there's an available byte or timeout.
 *
 * @param timeout Polling time before exiting (us)
 * @return the byte value on success; -1 if there's nothing in the buffer or timeout.
 */
int getSerialByteWithTimeout(uint32 timeout) {
	uint32 i;

	for (i = 0; i < timeout; i++) {
		if (UARTCharsAvail(UART0_BASE)) {
			return UARTCharGetNonBlocking(UART0_BASE);
		}
		systemDelayTenMicroSecond(1);
	}

	return -1;
}


/*
 * @brief Check and get a byte from UART/USB with interrupt disabled.
 *
 * @return the byte value on success; -1 if there's nothing in the buffer.
 */
int getSerialByteNonblocking(void) {
	int c = -1;

	if (UARTCharsAvail(UART0_BASE)) {
		c = UARTCharGetNonBlocking(UART0_BASE);
	}

	return c;
}


/*
 * @brief Send a byte through serial port FIFO (UART)
 * @param value Data to be sent
 * @returns void
 */
void sendByte(uint8 value) {
	MAP_UARTCharPutNonBlocking(UART0_BASE, value);
}

