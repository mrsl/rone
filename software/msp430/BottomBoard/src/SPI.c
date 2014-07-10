#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"
#include "system.h"
#include "SPI_8962.h"
#include "I2C.h"
#include "BumpSensors.h"
#include "SPIMessage.h"

#define SPI_8962_SEL_BIT			BIT3
#define SPI_8962_SEL_PORT_SEL		P3SEL
#define SPI_8962_SEL_PORT_DIR		P3DIR
#define SPI_8962_SEL_PORT_REN		P3REN
#define SPI_8962_SEL_PORT_OUT		P3OUT
#define SPI_8962_SEL_PORT_IN		P3IN
#define SPI_8962_SEL_PORT_IE		P3IE
#define SPI_8962_SEL_PORT_IES		P3IES
#define SPI_8962_SEL_PORT_IFG		P3IFG

#define SPI_A0_CLK_BIT				BIT0
#define SPI_A0_SEL_BIT				BIT3
#define SPI_A0_MISO_BIT				BIT5
#define SPI_A0_MOSI_BIT				BIT4

#define SPI_IRQ_MSG_WAIT			25
#define SPI_MESSAGE_TIMEOUT			20

uint8 SPI8962MessageIn[MSP430_MSG_LENGTH];
uint8 SPI8962MessageOut[MSP430_MSG_LENGTH];
uint8 SPI8962MessageInBuf[MSP430_MSG_LENGTH];
uint8 SPI8962MessageOutBuf[MSP430_MSG_LENGTH];
uint8 SPI8962MessageIdx = 0;
uint8 sync = 0;

volatile boolean SPI8962MessageDone = FALSE;

void SPI8962Init(void) {
	uint8 i;
	
// 	//Init SPI A0 comms with the 8962
//	UCA0CTL0 = (                  UCMSB | UCMODE_0 | UCSYNC);		//clocked on the falling edge but not shifted
//	//UCA0CTL0 = (         UCCKPL | UCMSB | UCMODE_0 | UCSYNC);		//clocking on the rising edge but shifted over one bit
//	//UCA0CTL0 = (UCCKPH |          UCMSB | UCMODE_0 | UCSYNC); 	//clocking on the rising edge...4Pin SPI Slave
//	//UCA0CTL0 = (UCCKPH | UCCKPL | UCMSB | UCMODE_0 | UCSYNC);		//falling edge not shifted
//	UCA0CTL1 = UCSSEL_2;                     	// SMCLK
//	UCA0BR0 = 0x02;
//	UCA0BR1 = 0;
//	UCA0MCTL = 0;
//	P3SEL |= (SPI_A0_CLK_BIT | SPI_A0_MOSI_BIT | SPI_A0_MISO_BIT | SPI_A0_SEL_BIT);

	// 4-wire comms  This is for the RONE_V9
	UCA0CTL0 = (UCMSB | UCMODE_2 | UCSYNC);		//clocked on the falling edge, but not shifted, normal clock
	UCA0CTL1 = UCSSEL_2;                     	// SMCLK
	UCA0BR0 = 0x02;
	UCA0BR1 = 0;
	UCA0MCTL = 0;
	// put all pins under SPI control for slave mode with select line
	P3SEL |= (SPI_A0_CLK_BIT | SPI_A0_SEL_BIT | SPI_A0_MISO_BIT | SPI_A0_MOSI_BIT);
	
	SPI8962InterruptEnable();                 // Enable USCI0 RX interrupt
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	
	SPI8962MessageDone = FALSE;
	
	// Initialize the Buffers
	for(i = 0; i < MSP430_MSG_LENGTH; i++) {
		SPI8962MessageIn[i] 	= 0;
		SPI8962MessageInBuf[i] 	= 0;
		SPI8962MessageOut[i] 	= 0;
		SPI8962MessageOutBuf[i] = 0;
	}
}


void SPI8962Shutdown(void) {
	SPI8962InterruptDisable();
	UCA0CTL1 |= UCSWRST;                     // **Initialize USCI state machine**
	P3SEL &= ~(SPI_A0_CLK_BIT | SPI_A0_SEL_BIT | SPI_A0_MISO_BIT | SPI_A0_MOSI_BIT);
}


// Updates the TX buffer with a byte. This byte will be sent the next time
// that the RX ISR is called. This method will then need to be called before
// the next RX ISR, otherwise a 0x00 will be sent.
void SPI8962RegisterByteToSend(uint8 data) {
	UCA0TXBUF = data;
}


uint8 messageChecksum(uint8* msg, uint8 codeLen, uint8 msgLen) {
	uint8 i;
	uint8 sum = 0;

	for (i = codeLen; i < msgLen - 1; i++) {
		sum += (uint8)msg[i];
	}

	return sum;
}


void shiftBuffer(uint8* msg, uint8 len) {
	int i;

	for (i = 0; i < len - 1; i++) {
		msg[i] = msg[i + 1];
	}
}


uint32 SPIMessageTicks = 0;


// SPI 8962 (A0) Receive interrupt handling function
void SPI8962RX_ISR() {
	uint8 i;
	uint8 checksum;

	// Rotate received buffer
	shiftBuffer(SPI8962MessageIn, MSP430_MSG_LENGTH);

	// Retrieve received data
	SPI8962MessageIn[MSP430_MSG_LENGTH - 1] = UCA0RXBUF;

	// Send a byte
	while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
	if (sync) {
		UCA0TXBUF = SPI8962MessageOut[SPI8962MessageIdx];
	} else {
		UCA0TXBUF = 0;
	}

	if (!sync) {
		// When out of sync, find the first appearance of code to sync up
		for (i = 0; i < MSP430_CODE_LENGTH; i++) {
			if (SPI8962MessageIn[i] != MSP430Code[i])
				break;
		}
		if (i == MSP430_CODE_LENGTH) {
			SPI8962MessageIdx = 0;
			sync = 1;
		}
	} else if (SPI8962MessageIdx == MSP430_MSG_LENGTH - 1) {
		// Checkcode
		for (i = 0; i < MSP430_CODE_LENGTH; i++) {
			if (SPI8962MessageIn[i] != MSP430Code[i])
				break;
		}
		// If code matches, checksum. Else out of sync
		if (i == MSP430_CODE_LENGTH) {
			// If checksum matches, carry on. Else out of sync.
			checksum = messageChecksum(SPI8962MessageIn, MSP430_CODE_LENGTH, MSP430_MSG_LENGTH);
			if (checksum == SPI8962MessageIn[MSP430_MSG_LENGTH - 1]) {
				// Success!
				SPI8962MessageDone = TRUE;
				SPIMessageTicks = SPI_MESSAGE_TIMEOUT;

				// Copy the message that was read into the message buffer
				memcpy(SPI8962MessageInBuf, SPI8962MessageIn, MSP430_MSG_LENGTH);

				// Also copy the next message that needs to be sent into the buffer
				memcpy(SPI8962MessageOut, SPI8962MessageOutBuf, MSP430_MSG_LENGTH);
			} else {
				// Fail checksum
				sync = 0;
			}
		} else {
			// Fail checkcode
			sync = 0;
		}
		SPI8962MessageIdx = 0;						// Reset index counter for sync or next message
	} else {
		// In sync and no special event, carry on
		SPI8962MessageIdx++;
	}

	IFG2 &= ~UCA0RXIFG;
}


void SPI8962InterruptEnable(void) {
	IE2 |= UCA0RXIE;
}


void SPI8962InterruptDisable(void) {
	IE2 &= ~UCA0RXIE;
}


boolean SPI8962GetMessage(uint8* message) {
	if(SPI8962MessageDone) {
		// Copy the last message that was read into the place given by
		SPI8962InterruptDisable();
		memcpy(message, SPI8962MessageInBuf, MSP430_MSG_LENGTH);
		SPI8962InterruptEnable();
		return TRUE;
	} else {
		return FALSE;
	}
}


void SPI8962SetMessage(uint8* message) {
	SPI8962InterruptDisable();
	// Set the message as a new message
	SPI8962MessageDone = FALSE;
	// Copy the message over to be output as the next message
	memcpy(SPI8962MessageOutBuf, message, MSP430_MSG_LENGTH);
	SPI8962InterruptEnable();
}
