#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"


#define SPI_A0_CLK_BIT				BIT0
#define SPI_A0_SEL_BIT				BIT3
#define SPI_A0_MISO_BIT				BIT5
#define SPI_A0_MOSI_BIT				BIT4

#define SPI_BSL_CHECKSUM_LENGTH         2  // verification
#define SPI_BSL_MAX_ARRAY_SIZE         64  

//slave message
#define MSP430_MSG_VALID_MSG_BITS     0x81
#define MSP430_MSG_CONFIRM_BIT        0x02
#define MSP430_MSG_RESEND_BIT         0x04
#define MSP430_MSG_READY_BIT          0x08

//slave state
#define MSP430_STATE_RECIEVE_DATA      0
#define MSP430_STATE_READY             1
#define MSP430_STATE_TX_COMPLETE       2
#define MSP430_STATE_INIT_TRANS        3

//master message
#define ARM_MSG_VALID_MSG_BITS        0x81
#define ARM_MSG_NEW_TRANS_BIT         0x02
#define ARM_MSG_START_DATA_BIT        0x04
#define ARM_MSG_FINISH_TX_BIT         0x08
#define ARM_MSG_READY_BIT             0x10

uint16 calcMessageChecksum(uint8 * buffer, int length){
	uint8 i;
	uint16 checksum = 0x01;  // start at 1
	for(i=0; i<length; i++){
		checksum = checksum + (uint16)(i*(*(buffer+i)));
	}
	return checksum;
}

// Exchange one byte with the ARM (blocking)
// This takes whatever is in the TX buffer and sends it. The buffer is then
// reset to 0.
uint8 exchangeSPIByte() {
	register uint8 recieveMSG;

	//one communication with Arm
	while(!(IFG2 & UCA0RXIFG)){}
	recieveMSG = UCA0RXBUF;
	UCA0TXBUF  = 0x00;
	IFG2 &= ~UCA0RXIFG;
	
	return recieveMSG;
}

// Register a byte to send the next time that echangeSPIByte is called.
void setUpSPIMsgToSend(uint8 byteToSend) {
	UCA0TXBUF  = byteToSend;
}

void main(){
	//disbale watchdog
	WDTCTL = WDTPW | WDTHOLD;
	
	//disable iterrupts
	__bic_SR_register(GIE);
	
	// run at 16mhz
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	
	//SMCLK
	BCSCTL2 &= 0xF1; // sets SMCLK to DCO clock w/ division 1
	
	// Flash Controller register setup
	FCTL2 = FWKEY | FN4 | FN2 | FN1 | FN0 | FSSEL_2;  // use SMCLK with divisor of 40
	FCTL3 = FWKEY;                   // unlocks flash memory for erasing
	
	//initiate erasing of segments s2-last block  with a dummy write
	uint8 * addresses[14] = {(uint8*)0xE400, //s2
		                     (uint8*)0xE600, //s3
		                     (uint8*)0xE800, //s4
		                     (uint8*)0xEA00, //s5
		                     (uint8*)0xEC00, //s6
		                     (uint8*)0xEE00, //s7
		                     (uint8*)0xF000, //s8
		                     (uint8*)0xF200, //s9
		                     (uint8*)0xF400, //s10
		                     (uint8*)0xF600, //s11
		                     (uint8*)0xF800, //s12
		                     (uint8*)0xFA00, //s13
		                     (uint8*)0xFC00, //s14
		                     (uint8*)0xFE00 }; //s15
	register volatile uint8 i;	                     
	for(i=0; i<14; i++){
		FCTL1 = FWKEY | ERASE;
		*addresses[i] = 0x0000;
	}
	
	FCTL1 = FWKEY; //set write to 0
	FCTL3 = FWKEY | LOCK; // set lock
	
	////////SETTING UP SPI COMUNICATIONS////////////////////////
	UCA0CTL0 = (UCMSB | UCMODE_2 | UCSYNC);	// clocked on the falling edge but not shifted. MSP430 is slave
	UCA0CTL1 = UCSSEL_2;                    // SMCLK
	UCA0BR0 = 0x02;                         // set prescaler to 512
	UCA0BR1 = 0;
	UCA0MCTL = 0;
	// put all pins under SPI control for slave mode with select line
	P3SEL |= (SPI_A0_CLK_BIT | SPI_A0_SEL_BIT | SPI_A0_MISO_BIT | SPI_A0_MOSI_BIT);
	UCA0CTL1 &= ~UCSWRST; 
	
	/////////////////////////////////////////////////////////////
	uint8 msp430MSG = 0x00;
	setUpSPIMsgToSend(msp430MSG);
	
	uint8 msp430State = MSP430_STATE_READY;
	uint8 dataLength;
	uint8 *dataAddress;
	uint16 msgChecksum;
	uint16 calcChecksum;
	uint8 buffer[SPI_BSL_MAX_ARRAY_SIZE];
	register uint8 recieveMSG;
	register uint8 data;
	while(msp430State != MSP430_STATE_TX_COMPLETE){
		//Exchange a byte with the arm. Send the byte in the TX Buffer 
		recieveMSG = exchangeSPIByte();
		
		switch(msp430State){
			case MSP430_STATE_READY:
				// This switch-case is used to alter msp430State to handle
				// command transmissions from ARM
				switch(recieveMSG){
					case (ARM_MSG_VALID_MSG_BITS | ARM_MSG_NEW_TRANS_BIT):
						msp430State = MSP430_STATE_INIT_TRANS;
						msp430MSG   = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT);
						break;
					case (ARM_MSG_VALID_MSG_BITS | ARM_MSG_START_DATA_BIT):
						msp430State = MSP430_STATE_RECIEVE_DATA;
						msp430MSG   = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT); 
						break;
					case (ARM_MSG_VALID_MSG_BITS | ARM_MSG_FINISH_TX_BIT):
						msp430State = MSP430_STATE_TX_COMPLETE;
						msp430MSG   = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT); 
						break;
					case (ARM_MSG_VALID_MSG_BITS | ARM_MSG_READY_BIT):
						// the ARM is processing a confirm or a resend message
						msp430MSG   = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_READY_BIT);
						break;
				    default:
						msp430MSG   = (MSP430_MSG_VALID_MSG_BITS); 
				    	break;
				}
				break;
				
			case MSP430_STATE_INIT_TRANS:
				msp430State = MSP430_STATE_READY;
				///////////////GET INIT DATA ////////////////////////////
				for(i=0; i<3; i++){
					setUpSPIMsgToSend((uint8)i);
					buffer[i] = exchangeSPIByte();
				}
				
				//////////////GET CHECKSUM ///////////////////////////////
				//assuming checksum of 16 bits
				msgChecksum = 0x0000;
				for(i=0; i<SPI_BSL_CHECKSUM_LENGTH; i++){
					setUpSPIMsgToSend((uint8)i);
					data = exchangeSPIByte();
					msgChecksum = (msgChecksum << 8) | (uint16)data;
		    	}
		    	calcChecksum = calcMessageChecksum(buffer, 3);
		    	if(calcChecksum == msgChecksum){
		    		dataLength = buffer[0];
		    		dataAddress = (uint8 *)((((uint16)buffer[1]) << 8) | ((uint16)buffer[2]));
		    		msp430MSG = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT);
		    		msp430State = MSP430_STATE_READY;
		    	} else {
		    		msp430MSG = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_RESEND_BIT);
		    		msp430State = MSP430_STATE_READY;
		    	}
				break;
		    case MSP430_STATE_RECIEVE_DATA:
		    	msp430State = MSP430_STATE_READY;
				
		    	//////////RECIEVE DATA FROM ARM////////////////////////////
				msp430MSG = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT);
		    	for(i=0; i<dataLength; i++){
		    		setUpSPIMsgToSend((uint8)i);
					buffer[i] = exchangeSPIByte();
		    	}
				
		    	//////////RECIEVE CHECKSUM FROM ARM ///////////////////////
		    	//Assuming checksum length == 2 see #define above
		    	msgChecksum = 0x0000;
				msp430MSG = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT);
		    	for(i=0; i<SPI_BSL_CHECKSUM_LENGTH; i++){
		    		setUpSPIMsgToSend((uint8)i);
					data = exchangeSPIByte();
					msgChecksum = (msgChecksum << 8) | (uint16)data;
		    	}
		    	calcChecksum = calcMessageChecksum(buffer, dataLength);
		    	
		    	if(calcChecksum == msgChecksum){
					// Flushes RAM buffer into actual flash memory

					// Flash Controller register setup
					FCTL3 = FWKEY;                   // unlocks flash memory for erasing
					//set up Flash Controller for word/byte write
					FCTL1 = FWKEY | WRT;
					
		    		for(i=0; i<dataLength; i+=2){
						// constructs a word from 2 consecutive bytes					
						uint16 word = (uint16) buffer[i];
						// if(there's another byte in program){						
						if((i+1) < dataLength){
							// put in first 8 bits } else {						
							word = word | (((uint16)buffer[i+1]) << 8);                                 
						}
						//    leave padding }
						*((uint16 *)(dataAddress + i)) = word;
		    		}
					
					// Stop writing
					FCTL1 = FWKEY; //set write to 0
					FCTL3 = FWKEY | LOCK; // set lock
					
					// Exchange some useless bytes to fix the USCI state
					// machine after it was ignored because of the flash
					// writing. Poor state machine...
					exchangeSPIByte();

		    		msp430MSG = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT);
		    		msp430State = MSP430_STATE_READY;
		    	} else {
		    		msp430MSG = (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_RESEND_BIT);
		    		msp430State = MSP430_STATE_READY;
		    	}
		    	break;
		   	default:
		   		break;
		}
		
		// Register the byte that is in the msp430MSG to be sent next time
		setUpSPIMsgToSend(msp430MSG);
	}
	
	// Re-Enable Intupts. This is important becasue we are going to
	// come back to life as off.
	__bis_SR_register(GIE);
	
	
	// Cause a reset. This forces the program to start in the MSP430 main
	// program again. Do this by doing an invalid write to the watch dog
	WDTCTL = 0x0000;
	//asm("		mov  &0FFFEh ,PC ; Branch to MSP430 program from bootloader");
}

/*

Each Transaction Is Independent

ARM BYTE

0 MUST BE SET TO BE A VALID MESSAGE
0 START DATA
0 END TRANSMISSION
0 READY
0 
0
0 MUST BE SET TO BE A VALID MESSAGE

MSP430 BYTE

0 MUST BE SET TO BE A VALID MESSAGE
0 CONFIRM (TRANSMIT NEXT BLOCK)
0 RESEND  (RESEND LAST BLOCK)
0 READY
0
0
0
0 MUST BE SET TO BE A VALID MESSAGE
	
 */
