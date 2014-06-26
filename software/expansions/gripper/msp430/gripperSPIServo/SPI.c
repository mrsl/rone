/*
 * SPI.c
 *
 * Takes multi-byte message and puts into out buffer. At TX/RX interrupt pulls next byte from buffer and put into hardware buffer to be shifted out.
 * Also shifts in a byte which is put into in buffer. Once filled, in buffer is copied to gripperMessageInBuf[].
 *
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */

#include "typeDefs.h"
#include "currentSense.h"
#include "SPI.h"
#include "IO.h"

const uint8 gripperCode[] = {'R', 'O', 'B'};		//special code sent from gripper to 8962

uint8 gripperMessageIn[GRIPPER_MSG_LENGTH];
uint8 gripperMessageInBuf[GRIPPER_MSG_LENGTH];
uint8 gripperMessageOut[GRIPPER_MSG_LENGTH];
uint8 gripperMessageOutBuf[GRIPPER_MSG_LENGTH];
uint8 gripperMessageIdx = 0;
uint8 gripperMessageOutBufLock = 0;
uint8 gripperMessageInBufLock = 0;
uint8 sync = 0;
uint8 led = 0;


uint8 messageGetServoValue (void){
	return gripperMessageInBuf[GRIPPER_MSG_SERVO_VALUE_IDX];
}

/* Updates buffer with most recent current averages, discards 2 LSB */
void messagePackCurrentAverage(current currentValue){
	gripperMessageOutBufLock = 1;
	gripperMessageOutBuf[GRIPPER_MSG_CURRENT_VALUE_IDX] = (currentValue.average);
	gripperMessageOutBufLock = 0;
}

/* Updates buffer with most recent servo values*/
void messagePackServoValues(volatile uint8 servoValue) {
	gripperMessageOutBufLock = 1;
	gripperMessageOutBuf[GRIPPER_MSG_SERVO_VALUE_IDX] = servoValue;
	gripperMessageOutBufLock = 0;
}

/* Updates buffer with most recent IO values*/
void messagePackIO (uint8 message){
	gripperMessageOutBufLock = 1;
	gripperMessageOutBuf[GRIPPER_MSG_IO_IDX] = message;
	gripperMessageOutBufLock = 0;
}

/* Initilized the SPI communication */
void initSPI(void) {
	// put all pins under SPI control for slave mode with select line
	P1SEL |= (SPI_A0_CLK_BIT | SPI_A0_SEL_BIT | SPI_A0_MISO_BIT | SPI_A0_MOSI_BIT);
	P1SEL2 |= (SPI_A0_CLK_BIT | SPI_A0_SEL_BIT | SPI_A0_MISO_BIT | SPI_A0_MOSI_BIT);

	UCA0CTL1 = UCSWRST; // Put the SPI state machine in reset
	/*
	 * Note here that the MSP430 is INCAPABLE of running on the rising edge in
	 * slave mode because of a design limitation of the USCI module.
	 */
	UCA0CTL0 |= UCMODE_2 + UCMSB + UCSYNC; //clocked on the falling edge, but not shifted, normal clock (idle low), slave mode
	UCA0CTL1 &= ~UCSWRST; // Initialize the USCI state machine
	IE2 |= UCA0RXIE;
}

/* Shifts the buffer array toward the 0th index. The 0th index is discarded */
void shiftBuffer(uint8* message) {
	unsigned int i;
	P3OUT |= LED;				// Turn LED off
	for (i = 0; i < GRIPPER_MSG_LENGTH - 1; i++) {
		message[i] = message[i + 1];
	}
}


uint8 SPIChecksum(uint8* message) {
	uint8 messageCount;
	uint8 sum = 0;
	for (messageCount = GRIPPER_CODE_LENGTH; messageCount < GRIPPER_MSG_LENGTH - 1; messageCount++) {
		sum += (uint8)message[messageCount];
	}
	return sum;
}

void initMessage(void) {
	unsigned int messageCount;
	gripperMessageIdx = 0;
	// Initialize the message vectors, set each message byte to 0
	for (messageCount = 0; messageCount < GRIPPER_MSG_LENGTH; messageCount++) {
		gripperMessageOut[messageCount] = 0;
		gripperMessageOutBuf[messageCount] = 0;
		gripperMessageIn[messageCount] = 0;
		gripperMessageInBuf[messageCount] = 0;
	}
	// Pack sync code
	for (messageCount = 0; messageCount < GRIPPER_CODE_LENGTH; messageCount++) {
		gripperMessageOut[messageCount] = gripperCode[messageCount];
		gripperMessageOutBuf[messageCount] = gripperCode[messageCount];
	}
	// Init servo values before receiving messages
	gripperMessageInBuf[GRIPPER_MSG_SERVO_VALUE_IDX] = 125;
	// Pack checksum
	gripperMessageOut[GRIPPER_MSG_LENGTH - 1] = SPIChecksum(gripperMessageOut);
}

void messagePackOut(void) {
	if (!gripperMessageOutBufLock) {
		gripperMessageOutBuf[GRIPPER_MSG_LENGTH - 1] = SPIChecksum(gripperMessageOutBuf);
		memcpy(gripperMessageOut, gripperMessageOutBuf, GRIPPER_MSG_LENGTH);
	}
}

// Test for valid RX and TX character
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
	uint8 i;
	uint8 checksum;

	// Rotate received buffer
	shiftBuffer(gripperMessageIn);

	// Retrieve received data
	gripperMessageIn[GRIPPER_MSG_LENGTH - 1] = UCA0RXBUF;


	// Send a byte
	while (!(IFG2 & UCA0TXIFG));            // USCI_A0 TX buffer ready?

	if (sync)
		UCA0TXBUF = gripperMessageOut[gripperMessageIdx];
	else
		UCA0TXBUF = 0xBE;  					// send 0 if not in sync

	if (!sync) {
		// when out of sync, find the first appearance of code to sync up
		for (i = 0; i < GRIPPER_CODE_LENGTH; i++) {
			if (gripperMessageIn[i] != gripperCode[i])
				break;
		}
		if (i == GRIPPER_CODE_LENGTH) {
			gripperMessageIdx = 0;
			sync = 1;
		}
	} else if (gripperMessageIdx == GRIPPER_MSG_LENGTH - 1) {
		// Checkcode
		for (i = 0; i < GRIPPER_CODE_LENGTH; i++) {
			if (gripperMessageIn[i] != gripperCode[i]) //check for ROB
				break;
		}
		// If code matches, checksum. Else out of sync
		if (i == GRIPPER_CODE_LENGTH) {
			// If checksum matches, carry on. Else out of sync.
			checksum = SPIChecksum(gripperMessageIn);
			if (checksum == gripperMessageIn[GRIPPER_MSG_LENGTH - 1]) {
				// Success!
				//P3OUT &= ~LED;
				if (!gripperMessageInBufLock) {
					memcpy(gripperMessageInBuf, gripperMessageIn, GRIPPER_MSG_LENGTH);
				}
				// Pack received code to next out message
				messagePackOut();
			} else {
				// Fail checksum
				sync = 0;
				//P3OUT |= LED;
			}
		} else {
			// Fail checkcode
			sync = 0;
		}
		gripperMessageIdx = 0;						// Reset index counter for sync or next message

	} else {
		// In sync and no special event, carry on
		gripperMessageIdx++;
	}

	IFG2 &= ~UCA0RXIFG;

}

