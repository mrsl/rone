/*
 * @file msp430Bootloader.c
 * @brief  boot loader functions on MSP430
 * @since Jul 31, 2012
 * @author mrdouglass
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/flash.h"

#include "roneos.h"

#if defined(RONE_V9) || defined(RONE_V12)

// Include the correct MSP430 Data file
#if defined(RONE_V9)
	#include "msp430ProgramDataRoneV11.h"
#elif defined(RONE_V12)
	#include "msp430ProgramDataRoneV12.h"
#endif

//data
#define SPI_BSL_CHECKSUM_LENGTH         2  // verification
#define SPI_BSL_MAX_ARRAY_SIZE         64

//master message
#define ARM_MSG_VALID_MSG_BITS        0x81
#define ARM_MSG_NEW_TRANS_BIT         0x02
#define ARM_MSG_START_DATA_BIT        0x04
#define ARM_MSG_FINISH_TX_BIT         0x08
#define ARM_MSG_READY_BIT             0x10

//slave message
#define MSP430_MSG_VALID_MSG_BITS     0x81
#define MSP430_MSG_CONFIRM_BIT        0x02
#define MSP430_MSG_RESEND_BIT         0x04
#define MSP430_MSG_READY_BIT          0x08

//ARM state
#define ARM_STATE_SYNC						1
#define ARM_STATE_INIT_TRANSMISSION_1		2
#define ARM_STATE_INIT_TRANSMISSION_2		3
#define ARM_STATE_SETUP_TRANSMISSION		4
#define ARM_STATE_SIGNAL_DATA_1				5
#define ARM_STATE_SIGNAL_DATA_2				6
#define ARM_STATE_SEND_DATA					7
#define ARM_STATE_FINISH_TRANSMISSION_1		8
#define ARM_STATE_FINISH_TRANSMISSION_2		9

#define MSP430_BSL_BYTE_PERIOD         20000
#define MSP430_BSL_DESELECT_PERIOD	   400

//ARM SPI STATE
#define SPI_STATE_XMIT                   1
#define SPI_STATE_DESELECT               2

//validation
#define BSL_VALID_TRANSMISSION           0
#define BSL_INVALID_TRANSMISSION         1

//ARM and SPI state
uint8 armState;
uint8 armMessage;
uint8 spiState;

//general data positioning data
uint8 currentSection;
uint16 dataIndex;

//transmission setup
uint16 dataStartAddress;
uint16 dataLength;
uint8 setupData[5];
uint8 setupIndex;

//checksum
uint32 checksum;
uint8  checksumIndex;

//validation
uint8 transmissionState;

boolean blinkyLEDToggleState = FALSE;

/*
 * @brief
 * This method is a 16-bit, weighted checksum on x-mitted data
 * @returns uint16
 */
uint16 calcMessageChecksum(const uint8 * buffer, int start, int length){
	uint8 i;
	uint16 checksum = 0x01;  // start at 1
	for(i=0; i<length; i++){
		checksum = checksum + (uint16)(i*(*(buffer+start+i)));
	}
	return checksum;
}

/**
 * @brief Gets msp430 local software version
 * @returns msp430 local software version
 */

uint8 msp430BSLGetLocalVersionNumber(void) {
	uint8 temp = MSP430_PROGRAM_LOCAL_VERSION_NUMBER;
	return temp;
}


/**
 * @brief Gets msp430 hardware version
 * @returns msp430 hardware version
 */

uint8 msp430BSLGetLocalVersionHardwareNumber(void) {
	uint8 temp = MSP430_HARDWARE_VERSION_NUMBER;
	return temp;
}


/*
 * @brief
 * This method is used to initialize the MSP430 and interrupts for the BSL
 * @returns void
 */
void msp430BSLInit(void){
	MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, MSP430_BSL_BYTE_PERIOD);
	MAP_TimerEnable(TIMER1_BASE, TIMER_B);
	armState = ARM_STATE_SYNC;
	spiState = SPI_STATE_XMIT;
	currentSection = 0;
	dataStartAddress = MSP430_PROGRAM[currentSection].MSP430_SECTION_ADDRESS;
	dataIndex = 0;
	dataLength = 0;
	transmissionState = BSL_VALID_TRANSMISSION;
	blinkyLEDToggleState = FALSE;
	setMSP430SPIOperationState(MSP430_SPI_BOOT_LOADER_MODE);
}

/*
 * @brief MSP430 Boot Loader Handler
 *
 *  This method sends bytes periodically to the MSP430 to handle the boot loader function
 *  It transmits bytes about every 32us, which is about as fast as the MSP430 can Handle
 *  @returns void
 */
void msp430BSLHandler(){
	uint32 data;

	// clear the timer interrupt for the next time
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);


	if(spiState == SPI_STATE_DESELECT){
		SPIDeselectISR();
		spiState = SPI_STATE_XMIT;
		MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, MSP430_BSL_BYTE_PERIOD);
		MAP_TimerEnable(TIMER1_BASE, TIMER_B);
	} else if(spiState == SPI_STATE_XMIT){
		//get ready to transmit and set spiState to deselect soon
		SPISelectDeviceISR(SPI_MSP430);
		spiState = SPI_STATE_DESELECT;
		MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, MSP430_BSL_DESELECT_PERIOD);
		MAP_TimerEnable(TIMER1_BASE, TIMER_B);

		switch(armState){
			case ARM_STATE_SYNC: {
				uint32 i;	// short delay to make sure the bottomboard is ready
				for (i = 0; i < 10000; i++);

				// Send a ready message and wait for a response that the MSP is also ready
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_READY_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_READY_BIT)) {
					armState = ARM_STATE_INIT_TRANSMISSION_1;
				} else {
					// Failure this time, try again
					armState = ARM_STATE_SYNC;
				}
				break;
			}

			case ARM_STATE_INIT_TRANSMISSION_1:
				// Send the message that we want to start
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_NEW_TRANS_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				// Get the response
				armState = ARM_STATE_INIT_TRANSMISSION_2;

				break;

			case ARM_STATE_INIT_TRANSMISSION_2:
				// Get the response back, data that is sent is junk
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_READY_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT)){
					// Toggle the Blinky LED
					blinkyLEDToggleState = !blinkyLEDToggleState;
					blinkyLedSet(blinkyLEDToggleState);

					//the last transmission was validated. We need to set up a new section of data for TX
					if(transmissionState == BSL_VALID_TRANSMISSION){
						if(dataIndex == MSP430_PROGRAM[currentSection].MSP430_SECTION_SIZE){
							// the previous data transmission included the last portion of the previous data section
							// so we need to increment the currentSection variable to indicate we are now on the next
							// data section
							currentSection++;
							dataStartAddress = MSP430_PROGRAM[currentSection].MSP430_SECTION_ADDRESS;
							dataIndex = 0;
						} else {
							//The previous data transmission left off somewhere in the middle of a large chunk of data.
							dataStartAddress = MSP430_PROGRAM[currentSection].MSP430_SECTION_ADDRESS + dataIndex;
						}
					} else if(transmissionState == BSL_INVALID_TRANSMISSION){
						//last section of data failed validation. Bring dataIndex back to the starting address
						//of previous section for retransmission
						dataIndex = 0;
						transmissionState = BSL_VALID_TRANSMISSION;
					}

					// Calculate the length of the data to send
					if((dataIndex + 64) > MSP430_PROGRAM[currentSection].MSP430_SECTION_SIZE){
						//we cannot construct another transmission of 64 bytes so we need to make the message
						//end at the end of section of data.
						dataLength = MSP430_PROGRAM[currentSection].MSP430_SECTION_SIZE - dataIndex;
					} else {
						dataLength = 64;
					}

					// Set up the data fields
					setupData[0] = dataLength;
					setupData[1] = (uint8)(dataStartAddress >> 8);
					setupData[2] = (uint8)(dataStartAddress);
					checksum = calcMessageChecksum(setupData, 0, 3);
					setupData[3] = (uint8)(checksum >> 8);
					setupData[4] = (uint8)checksum;
					setupIndex = 0;
					armState = ARM_STATE_SETUP_TRANSMISSION;
				} else {
					// The message was a failure. Synchronize and try again
					armState = ARM_STATE_SYNC;
				}
				break;

			case ARM_STATE_SETUP_TRANSMISSION:
				if(setupIndex < 5){
					uint32 rubish;
					MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)setupData[setupIndex]);
					MAP_SSIDataGet(SSI0_BASE, &rubish);
					setupIndex++;
				} else {
					uint32 data;

					// Send a dummy message out to get the response
					armMessage = (ARM_MSG_VALID_MSG_BITS  | ARM_MSG_READY_BIT);
					MAP_SSIDataPutNonBlocking(SSI0_BASE, armMessage);
					MAP_SSIDataGet(SSI0_BASE, &data);

					if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT)){
						checksumIndex = 0;
						checksum = calcMessageChecksum(MSP430_PROGRAM[currentSection].MSP430_SECTION_DATA, dataIndex, dataLength);
						armState = ARM_STATE_SIGNAL_DATA_1;
					} else if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_RESEND_BIT)){
						armState = ARM_STATE_SYNC;
						transmissionState = BSL_INVALID_TRANSMISSION;
					}
				}
				break;

			case ARM_STATE_SIGNAL_DATA_1:

				// Signal the boot loader that we are about to send data
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_START_DATA_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				armState = ARM_STATE_SIGNAL_DATA_2;

				break;

			case ARM_STATE_SIGNAL_DATA_2:

				// Get the response back, data that is sent is junk
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_READY_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT)){
					armState = ARM_STATE_SEND_DATA;
				} else {
					armState = ARM_STATE_SIGNAL_DATA_1;
				}
				break;

			case ARM_STATE_SEND_DATA:

				if(MSP430_PROGRAM[currentSection].MSP430_SECTION_ADDRESS + dataIndex < dataStartAddress + dataLength){ //if there's more data
					uint32 rubish;
					MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)MSP430_PROGRAM[currentSection].MSP430_SECTION_DATA[dataIndex]);
					MAP_SSIDataGet(SSI0_BASE, &rubish);
					dataIndex++;
				} else if(checksumIndex < 2){ //else if we're still sending the checksum
					uint32 rubish;
					if(checksumIndex == 0){
						MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)((uint8)(checksum >> 8)));
						MAP_SSIDataGet(SSI0_BASE, &rubish);
					}else if(checksumIndex == 1){
						MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)((uint8)checksum));
						MAP_SSIDataGet(SSI0_BASE, &rubish);
					}
					checksumIndex++;
				} else {
					// Wait for the MSP430 to confirm or ask for a re-send
					uint32 data;
					armMessage = (ARM_MSG_VALID_MSG_BITS  | ARM_MSG_READY_BIT);
					MAP_SSIDataPutNonBlocking(SSI0_BASE, armMessage);
					MAP_SSIDataGet(SSI0_BASE, &data);

					if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT)){
						if(dataIndex >= MSP430_PROGRAM[currentSection].MSP430_SECTION_SIZE && currentSection >= (MSP430_PROGRAM_NUM_SECTIONS - 1)){
							//we are through sending data to the MSP430
							armState = ARM_STATE_FINISH_TRANSMISSION_1;
						} else {
							//we need to send another section
							armState = ARM_STATE_SYNC;
							transmissionState = BSL_VALID_TRANSMISSION;
						}
					} else if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_RESEND_BIT)){
						armState = ARM_STATE_SYNC;
						transmissionState = BSL_INVALID_TRANSMISSION;
					} else {
						// Sometimes the flash writing process may still be
						// going on. Ask the MSP if it is done again
						armState = ARM_STATE_SEND_DATA;
					}
				}
				break;

			case ARM_STATE_FINISH_TRANSMISSION_1:
				cprintf("\nreprogrammed\n");
				// Send the done message
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_FINISH_TX_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				armState = ARM_STATE_FINISH_TRANSMISSION_2;

				break;

			case ARM_STATE_FINISH_TRANSMISSION_2:
				// Get a response back
				armMessage = (ARM_MSG_VALID_MSG_BITS | ARM_MSG_READY_BIT);
				MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)armMessage);
				MAP_SSIDataGet(SSI0_BASE, &data);

				if((uint8)data == (MSP430_MSG_VALID_MSG_BITS | MSP430_MSG_CONFIRM_BIT)){
					// Horray!!
					setMSP430SPIOperationState(MSP430_SPI_NORMAL_OPERATION);
				} else {
					// Try again???
					armState = ARM_STATE_FINISH_TRANSMISSION_1;
				}
				break;
		}
	}
}

#endif

