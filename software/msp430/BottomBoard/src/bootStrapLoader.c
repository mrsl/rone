#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"
#include "bootStrapLoader.h"
#include "bootloaderProgramData.h"

// This function is used to transfer the data of a new version of the boot loader into the 
// first two segments of main falsh memory (s0, s1). The function recognizes the need to 
// reprogram these segments of memory by cross referencing the version number stored at E001
// with the current version number defined. The above structure contains the data of the 
// bootloader as well as the starting address of that data. This function erases the information
// within these first two segments and then transfers new data via word-writes. Finally the
// version number at E001 is updated and the address of the first bootloader instruction is
// stored at E002
void updateMSP430BSLVersion(void){
	
	//disbale watchdog
	WDTCTL = WDTPW | WDTHOLD; // Disable WDT
	
	//SMCLK set to run at full 16MGHz
	BCSCTL2 &= 0xF1; // sets SMCLK to DCO clock w/ division 1
	
	// Flash Controller register setup
	FCTL2 = FWKEY | FN4 | FN2 | FN1 | FN0 | FSSEL_2;  // use SMCLK with divisor of 40
	FCTL3 = FWKEY;                   // unlocks flash memory for erasing
	FCTL1 = FWKEY | ERASE;           // sets up Flash Controller to segment erase mode
	
	//initiate erasing of s0 with a dummy write
	uint8 *addr = (uint8 *)0xE000;   // first address of s0
	(*addr) = 0;                     // write to address tp begin erase
	//initiate erasing of s1 with a dummy write
	FCTL1 = FWKEY | ERASE;           // sets up Flash Controller to segment erase mode
	addr = (uint8 *)0xE200;         // first address of s1
	(*addr) = 0;                     // write to address to begin erase
	
	//set up Flash Controller for word/byte write
	FCTL1 = FWKEY | WRT;
	
	// write data from MSP430_PROGRAM[0].DATA to segments 0 and 1 
	// starting at the address MSP430_PROGRAM[0].MSP430_SECTION_ADDRESS
	int i;
	for(i=0; i< MSP430_PROGRAM[0].MSP430_SECTION_SIZE; i+=2){
		uint16 word = (uint16) MSP430_PROGRAM[0].MSP430_SECTION_DATA[i];        // constructs a word from 2 consecutive bytes
		//word = word << 8;                                                     // puts first of two bytes into last 8 bits
		if(i+1 != MSP430_PROGRAM[0].MSP430_SECTION_SIZE){                       // if(there's another byte in program){
			word |= ((uint16)MSP430_PROGRAM[0].MSP430_SECTION_DATA[i+1] << 8);  //    put in first 8 bits } else {
		}                                                                       //    leave padding }
		(*((uint16 *)(MSP430_PROGRAM[0].MSP430_SECTION_ADDRESS + i))) = word;
	}
	
    //update bsl version number
    *((uint8 *) MSP430_BSL_VERSION_ADRESS) = MSP430_BSL_LOCAL_VERSION_NUMBER;
	
	// First address of bootloader program (TOTO: note that this is a little
	// hackish. There is no way to know for sure that this is actually the
	// start address of the bootloader program, we are just guessing.
	uint16 BSLBeginAddress = (uint16)MSP430_PROGRAM[1].MSP430_SECTION_DATA[0];
	BSLBeginAddress          |= ((uint16)MSP430_PROGRAM[1].MSP430_SECTION_DATA[1] << 8);
	
	// temporary storage of the first instruction
	uint16 * tempBSLBeginAddress    = (uint16 *)0xE002;
	(*tempBSLBeginAddress) = BSLBeginAddress;
	
	FCTL1 = FWKEY; //set write to 0
	FCTL3 = FWKEY | LOCK | LOCKA; // set lock
}

void startBSL(void){
	// This in-line assembly statement sets the Program Counter within the CPU to point to
	// the address of the first bootloader instruction (whose address is stored within word E002)
	__bic_SR_register(GIE);
	asm("		mov  &0E002h ,PC ; Branch to bootloader");
}

uint8 getStoredBSLVersion(void){
	return MSP430_BSL_LOCAL_VERSION_NUMBER;
}
