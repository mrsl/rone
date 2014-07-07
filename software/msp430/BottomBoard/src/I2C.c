#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"
#include "I2C.h"
#include "System.h"


// Port Definitions
#define I2C_PORT_SEL			P3SEL
#define I2C_PORT_DIR			P3DIR
#define I2C_PORT_REN			P3REN
#define I2C_PORT_OUT			P3OUT
#define I2C_PORT_IN				P3IN
#define I2C_PORT_IE				P3IE
#define I2C_PORT_IES			P3IES
#define I2C_PORT_IFG			P3IFG

#define I2C_SDA_BIT				BIT1
#define I2C_SCL_BIT				BIT2

#define I2C_PIN_BITMASK			(I2C_SDA_BIT | I2C_SCL_BIT)

// Timeout and Speed Definitions
#define I2C_SPEED_PRESCALER		50		// fSCL = SMCLK/50 = ~320kHz

// Static variables for I2C
uint8* dataPtr;
volatile uint8 byteCtr;
uint8 receiveMode;
uint8 interuptCount;
uint8 address;
uint8 subAddress;
uint8 txTempData;
volatile uint8 rxTempData;

// Function Definitions
void endianSwap(uint8* data) {
	uint8 temp = data[0];
	data[0] = data[1];
	data[1] = temp;
}

void I2CInit(void) {
	I2C_PORT_SEL |= I2C_PIN_BITMASK;        // Assign I2C pins to USCI_B0
	
	UCB0CTL1 = UCSWRST;                    	// Enable SW reset
	UCB0CTL0 = (UCMST | UCMODE_3 | UCSYNC);	// I2C Master, synchronous mode
	UCB0CTL1 = (UCSSEL_2 | UCSWRST);      	// Use SMCLK, keep SW reset
	UCB0BR0 = I2C_SPEED_PRESCALER;                         	
	UCB0BR1 = 0;
	UCB0CTL1 &= ~UCSWRST;                 	// Clear SW reset, resume operation
	receiveMode = FALSE;
	UCB0I2CIE = UCNACKIE;
	IE2 |= UCB0TXIE;                       	// Enable TX interrupt
	IE2 |= UCB0RXIE;                      	// Enable RX interrupt
}


void I2CReset(void) {
	// Since initializing the I2C controller resets it, 
	// go ahead and just do that
	I2CInit();
}


void I2CShutdown(void) {
	I2CInterruptDisable();
	UCB0CTL1 = UCSWRST;                    	// Enable SW reset
	P3SEL &= ~I2C_PIN_BITMASK;              // Assign I2C pins to GPIO
}


void I2CInterruptDisable(void) {
	IE2 &= ~UCB0TXIE;                       // Disable TX interrupt
	IE2 &= ~UCB0RXIE;                      	// Disable RX interrupt	
}


void I2CSendArray(uint8 addressArg, uint8 subAddressArg, uint8* dataPtrArg, uint8 length) {
	volatile uint32 i;
	
	// Wait until the I2C bus is not busy
	for(i = 0; (UCB0STAT & UCBBUSY) && (i < I2C_MAX_WHILE_DELAY) ; i++) {}
	if(i == I2C_MAX_WHILE_DELAY) {
		i = 0;
		I2CReset();
		return;
	}
	
	address = addressArg;
	subAddress = subAddressArg;		// The address of the register to write to			
	dataPtr = dataPtrArg;
	
	byteCtr = length;				// Setup the number of bytes to read				
	interuptCount = 0;
	receiveMode = FALSE;
	
	// Set the slave address, start the I2C handler in transmitter mode,
	// and send a start condition. This will initate the
	// I2C transmission.
	UCB0I2CSA = addressArg;					// Slave Address
	UCB0CTL1 |= (UCTR | UCTXSTT);			// Put I2C into TX mode and send the start, this starts the transmission
	
	// wait until the entire xfer is finished and the line is no longer busy
	for(i = 0; (UCB0STAT & UCBBUSY) && (i < I2C_MAX_WHILE_DELAY) ; i++) {}
	if(i == I2C_MAX_WHILE_DELAY) {
		i = 0;
		I2CReset();
		return;
	}
}


void I2CSend(uint8 addressArg, uint8 subAddressArg, uint8 data) {
	txTempData = data;
	I2CSendArray(addressArg, subAddressArg, &txTempData, 1);
}


void I2CReceiveArray(uint8 addressArg, uint8 subAddressArg, uint8* dataPtrArg, uint8 length) {
	volatile uint32 i;
	
	//Don't handle 1 byte or less recieves
	//TODO need to special case single byte receive.  Or not.
	if (length <= 1) {
		return;
	}
	
	// Ensure I2C bus is not busy
	for(i = 0; (UCB0STAT & UCBBUSY) && (i < I2C_MAX_WHILE_DELAY) ; i++) {}
	if(i == I2C_MAX_WHILE_DELAY) {
		i = 0;
		I2CReset();
		return;
	}
	
	address = addressArg;
	subAddress = subAddressArg;
	dataPtr = dataPtrArg;
	
	byteCtr = length;
	interuptCount = 0;
	receiveMode = TRUE;
	
	// Set the slave address, start the I2C handler in transmitter mode 
	// (The interupt will switch this to reciver mode after sending the
	// sub address), and send a start condition. This will initate the
	// I2C recive transmission.
	UCB0I2CSA = address;					// Slave Address
	UCB0CTL1 |= (UCTR | UCTXSTT);			// I2C TX, start condition
	
	// Wait until the tranmission is done and the I2C bus is not busy
	for(i = 0; (UCB0STAT & UCBBUSY) && (i < I2C_MAX_WHILE_DELAY) ; i++) {}
	if(i == I2C_MAX_WHILE_DELAY) {
		i = 0;
		I2CReset();
		return;
	}				
}



//------------------------------------------------------------------------------
// The USCIAB0TX_ISR is structured such that it can be used to transmit any
// number of bytes by pre-loading TXByteCtr with the byte count. Also, TXData
// points to the next byte to transmit. This interupt is also (correctly, see
// the Family Users Guide) used to recived information over I2C. Set byteCtr
// equal to the number of bytes you wish to read.
//------------------------------------------------------------------------------
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	// I2C in RX Mode (UCTR Cleared)
	if (IFG2 & UCB0RXIFG){
		if (byteCtr > 0) {
			*dataPtr = UCB0RXBUF;		// Move RX data to address PRxData
			dataPtr++;
			byteCtr--;
			if (byteCtr == 1) {
				UCB0CTL1 |= UCTXSTP;	// Generate I2C stop condition after the
										// reception of the second to last byte
										// (during the reception of the last byte)
			}
		} else {
			rxTempData = UCB0RXBUF;		// Read the RX buffer even if it isn't used to prevent crashes
			UCB0CTL1 |= UCTXSTP;		// Generate I2C stop condition, just in case
			IFG2 &= ~UCB0RXIFG;      	// Clear USCI_B0 RX int flag
		}
	}
	
	// I2C in TX Mode (UCTR Set)
	if(IFG2 & UCB0TXIFG) {
		if(receiveMode == TRUE) {
			if(interuptCount == 0) {
				UCB0TXBUF = subAddress;	// Load TX buffer
				//UCB0CTL1 |= UCTXSTT;	// I2C TX, start condition
				interuptCount++;
			} else if(interuptCount == 1) {
				IFG2 &= ~UCB0TXIFG;		// Clear USCI_B0 TX int flag
				UCB0CTL1 &= ~UCTR;		// change to RX mode
				UCB0I2CSA = address;	// Assign the address
				UCB0CTL1 |= UCTXSTT;	// repeated start				
				interuptCount++;
			} else {
				//UCB0CTL1 |= UCTXSTP;	// Generate I2C stop condition
				IFG2 &= ~UCB0TXIFG;		// Clear USCI_B0 TX int flag
			}
		} else {
			if(interuptCount == 0) {
				UCB0TXBUF = subAddress;	// Load TX buffer
				//UCB0CTL1 |= UCTXSTT;	// I2C TX, start condition
				interuptCount++;
			} else {
				if (byteCtr > 0) {			// Check TX byte counter
					UCB0TXBUF = *dataPtr;	// Load TX buffer
					dataPtr++;
					byteCtr--;
				} else {
					UCB0CTL1 |= UCTXSTP;  	// I2C stop condition
					IFG2 &= ~UCB0TXIFG;  	// Clear USCI_B0 TX int flag
				}
			}
		}
	}
}

