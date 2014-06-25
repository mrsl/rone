/**
 * @file bootloader.c
 *
 * @brief R-One Bootloader main
 *
 * @since July 26, 2013
 * @author William Xie
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(PART_LM3S8962)
	#include "inc/lm3s8962.h"
#endif

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

#include "System/bootloader.h"

#define	DEFAULT_RONEID				126

const uint8 MSP430Code[] = {'B', 'O', 'T'};

uint8 roneID;
RadioMessage radioMessage;

int main() {
	uint32 i, state;

	systemInit();

	// Read bootloader state value
	state = readBootloaderState();

	// Direct branch code, use for testing
//	if (state == BL_STATE_NORMAL) {
//		bootOS(OS_START_ADDRESS);
//	} else {
//		while (1) {
//			blinkyLedToggle();
//			systemDelay(1000);
//		}
//	}

	switch (state) {
		case BL_STATE_NORMAL:
			break;
		case BL_STATE_HOST:
			faultPrintSerial("Bootloader radio host mode\n");
			faultPrintSerial("ID = ");
			faultPrintSerialDec(roneID);
			faultPrintSerial("\n\n");
			radioProgrammingInput();
			radioProgramming(RADIO_PROGRAMMING_HOST);
			// Control should not reach here
			break;
		case BL_STATE_RECEIVE:
			faultPrintSerial("Bootloader radio receiver mode\n");
			faultPrintSerial("ID = ");
			faultPrintSerialDec(roneID);
			faultPrintSerial("\n\n");
			radioProgramming(RADIO_PROGRAMMING_REMOTE);
			// Contorl should not reach here
			break;
		case BL_STATE_LIMBO:
			faultPrintSerial("Bootloader radio limbo mode\n");
			faultPrintSerial("ID = ");
			faultPrintSerialDec(roneID);
			faultPrintSerial("\n\n");
			radioProgramming(RADIO_PROGRAMMING_LIMBO);
			// Control should not reach here
			break;
		case BL_STATE_XMODEM:
			faultPrintSerial("Bootloader XMODEM mode\n\n");
			// To let the user know the robot is in xmodem mode
			blinkyLedSet(1);
			if (xmodemDownalodBinary() != 0) {
				faultPrintSerial("Error: XMODEM programming failed\n");
				while(1) {
					blinkyLedToggle();
					systemDelay(1000);
				}
			}
			break;
		default:
			faultPrintSerial("Error: invalid bootloader state\n\n");
			while(1) {
				blinkyLedToggle();
				systemDelay(1000);
			}
			break;
	}

	// Reset robot and run main program after programming
	writeBootloaderState(BL_STATE_NORMAL);
	hardReset();

	return 0;
}


/*
 * @brief Branch to program or bootloader using starting address
 * @param ulStartAddr Starting address of the main robot program
 * @returns void
 */
void __attribute__((naked))
bootOS(uint32 ulStartAddr) {
	// Disable all active and pending interrupts
	HWREG(NVIC_DIS0) = 0xFFFFFFFF;
	HWREG(NVIC_DIS1) = 0x00000FFF;

	HWREG(NVIC_UNPEND0) = 0xFFFFFFFF;
	HWREG(NVIC_UNPEND1) = 0x00000FFF;

	// Set the vector table to the beginning of the app in flash.
	HWREG(NVIC_VTABLE) = ulStartAddr;

	__asm("    ldr     r1, [r0]			\n" /* Load the stack pointer from the application's vector table. */
	      "    mov     sp, r1			\n"
		  "    ldr     r0, [r0, #4]		\n" /* Load the initial PC from the application's vector table and branch to the application's entry point */
	      "    bx      r0				\n"
		);
}


/*
 * @brief Initialize the minimal amount of hardware for the bootloader to function
 * @returns void
 */
void systemInit(void) {
	uint32 i, j;

	MAP_IntMasterDisable();

	// increase LDO voltage so that PLL operates properly
	MAP_SysCtlLDOSet(SYSCTL_LDO_2_75V);
	#ifdef PART_LM3S8962
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	#endif

	systemIDInit();
	blinkyLedInit();
	buttonsInit();
	srand(roneID);
	serialInit();
	#if defined(RONE_V9) || defined(RONE_V12)
	SPIInit();
	radioInit();
	#endif // defined(RONE_V9) || defined(RONE_V12)

	// Triple blink blinky, startup signal
	for (i = 0; i < 3; i++) {
		blinkyLedSet(1);
		for (j = 0; j < 150000;) {
			j++;
		}
		blinkyLedSet(0);
		for (j = 0; j < 250000;) {
			j++;
		}
	}

	// Initialize 24-bit Systick
	SysTickPeriodSet(0xffffff);
	SysTickEnable();
}


/*
 * @brief Gets the robot ID.
 *
 * roneID is assigned robot ID. If robot is uninitialized, roneID is DEFAULT_RONEID.
 * The ID is set using LM Flash.  The format for the ID is: 00-50-C2-00-01-XX
 * where XX is the robotID
 * @returns void
 */
void systemIDInit(void) {
	uint32 ureg0;
	uint32 ureg1;
	uint32 id = 0;
	int i;

	MAP_FlashUserGet(&ureg0, &ureg1);
	for (i = 0; i < 2; i++) {
		id = id << 8;
		ureg1 = ureg1 >> 8;
		id |= (ureg1 & 0xFF);
	}

	if (id == 0xFFFF) {
		// Uninitialized robot.  return DEFAULT_RONEID
		roneID = DEFAULT_RONEID;
	} else {
		roneID = id & 0xFF;
	}
}

/*
 * @brief Return the ID for this robot
 * @return Robot's ID
 */
uint8 getSelfID(void) {
	return roneID;
}


/*
 * @brief Systick backed delay in milisecond increment (imprecise)
 * @param delay Delay time (ms)
 * @return void
 */
void systemDelay(uint32 delay) {
	unsigned long start, end;
	uint32 i;
	boolean done;

	// Systick counts down, max interval = 0xfffff (16777216)
	// CPU runs at 50 MHz
	// 1 ms = 50,000 ticks
	for (i = 0; i < delay; i++) {
		start = SysTickValueGet();
		done = FALSE;
		while (!done) {
			end = SysTickValueGet();
			if (start - end > 50000 ) {
				done = TRUE;
			}
		}
	}
}


/*
 * @brief Systick backed delay in 10 microseconds increment (imprecise)
 * @param delay Delay time (10 us)
 * @return void
 */
void systemDelayTenMicroSecond(uint32 delay) {
	unsigned long start, end;
	uint32 i;
	boolean done;

	// Systick counts down, max interval = 0xfffff (16777216)
	// CPU runs at 50 MHz
	// 1 us = 50 ticks
	// Because 50 is too short of a delta to sample, 1 microsecond delay is very inaccurate
	for (i = 0; i < delay; i++) {
		start = SysTickValueGet();
		done = FALSE;
		while (!done) {
			end = SysTickValueGet();
			if (start - end > 500) {
				done = TRUE;
			}
		}
	}
}


/*
 * @brief Edit the bootloader state words (subnet and bootloader state)
 * @param state The value of the bootloader state
 * @return void
 */
void writeBootloaderState(unsigned long state) {
	uint32 tempBootloaderSubnet = bootloaderSubnet;
	// address of the last word (4 bytes) of state flash = 0x0003FFFC
	// address of last 1kb block = 0x0003FC00
	FlashErase(BL_STATE_BLOCK_ADDRESS);
	FlashProgram(&state, BL_STATE_WORD_ADDRESS, sizeof(uint32));
	// address of the second to the last word in state flash
	FlashProgram(&tempBootloaderSubnet, BL_STATE_SUBNET_ADDRESS, sizeof(uint32));
}


/*
 * @brief Read the bootloader subnet word
 * @return the subnet value
 */
uint32 readBootloaderState() {
	return *((uint32 *)BL_STATE_WORD_ADDRESS);
}


/*
 * @brief Read the bootloader state word
 * @return the state value
 */
uint32 readBootloaderSubnet() {
	return *((uint32 *)BL_STATE_SUBNET_ADDRESS);
}


/*
 * @brief Reset the top board by transmitting a reset message to bottom board.
 * @return void
 */
void hardReset() {
	uint32 data, i;
	uint8 MSP430MessageOut[MSP430_MSG_LENGTH];

	// Set up bottomboard SPI channel
	SPIConfigureDevice(SPI_MSP430);
	SPISelectDeviceISR(SPI_MSP430);

	// Pack reset message
	for (i = 0; i < MSP430_CODE_LENGTH; i++) {
		MSP430MessageOut[i] = MSP430Code[i];
	}
	for (i = MSP430_CODE_LENGTH; i < MSP430_MSG_LENGTH; i ++) {
		MSP430MessageOut[i] = 0;
	}
	MSP430MessageOut[MSP430_CMD_COMMAND_IDX] =  MSP430_CMD_COMMAND_RESET;
	MSP430MessageOut[MSP430_CMD_CHECKSUM_IDX] =  MSP430_CMD_COMMAND_RESET;

	// Send reset message. Syncing is done on the bottomboard side
	while (1) {
		for (i = 0; i < MSP430_MSG_LENGTH; i++) {
			MAP_SSIDataPutNonBlocking(SSI0_BASE,  (uint32)MSP430MessageOut[i]);
			MAP_SSIDataGet(SSI0_BASE, &data);
			systemDelay(1);
		}
	}
}

/*
 * @brief print out the string in parameter over serial. Serial COM must first be initialized.
 */
void faultPrintSerial(const char *string) {
	int i, j, len;

	len = strlen(string);

	for (i = 0; i < len; i++) {
		// Block until UART buffer has more room
		while (!MAP_UARTSpaceAvail(UART0_BASE));
		MAP_UARTCharPutNonBlocking(UART0_BASE, string[i]);
		if (string[i] == '\n') {
			while (!MAP_UARTSpaceAvail(UART0_BASE));
			MAP_UARTCharPutNonBlocking(UART0_BASE, '\r');
		}
	}
}


/*
 * @brief print out hex value in parameter over serial. Serial com must first be initialized.
 * The hex value is at most 8 characters long. The print out value is trailed by \n and \0.
 */
void faultPrintSerialHex(unsigned long num)
{
	unsigned long temp = num;
	short i, j, exp = 0;
	char h[16] = {'0','1','2','3','4','5','6','7','8','9','A','B',' C','D','E','F'};
	char hex[2 + 8 + 2];

	hex[0] = '0';
	hex[1] = 'x';

	while(temp >= 16)
	{
		exp++;
		temp /= 16;
	}

	// For each digit in hex
	for (i = 0; i <= exp; ++i)
	{
		temp = num;

		// Divide the right amount of time to get remainder for each digit
		for (j = i; j <= exp; ++j)
		{
			if (j == exp )
				hex[2 + i] = h[temp % 16];
			else
				temp /= 16;
		}
	}

	hex[2 + exp + 1] = '\n';
	hex[2 + exp + 2] = '\0';

	faultPrintSerial(hex);
}


/*
 * @brief Simple itoa then print over serial. Very similar to faultPrintSerialHex.
 *  The printout value is trailed by \0.
 */
void faultPrintSerialDec(unsigned long num)
{
	unsigned long temp = num;
	short i, j, exp = 0;
	char h[10] = {'0','1','2','3','4','5','6','7','8','9'};
	char dec[8 + 1];

	while(temp >= 10)
	{
		exp++;
		temp /= 10;
	}

	// For each digit in hex
	for (i = 0; i <= exp; ++i)
	{
		temp = num;

		// Divide the right amount of time to get remainder for each digit
		for (j = i; j <= exp; ++j)
		{
			if (j == exp )
				dec[i] = h[temp % 10];
			else
				temp /= 10;
		}
	}

	dec[exp + 1] = '\0';

	faultPrintSerial(dec);
}


/*
 * @brief fault handler that is called in a fault trap. It pulls the pushed register values from the stack
 * and prints them out over serial. Special fault register values are also printed.
 */
void faultHandler(unsigned int *fault_args, unsigned int exType) {
	int i;

	while (1) {
		// Print the fault message over serial (serial must be initialized first)
		faultPrintSerial("\n- Fault handler -\n\n");

		faultPrintSerial("ronebl\n");
		faultPrintSerial("type = ");
		switch (exType) {
		case 1:
			faultPrintSerial("hard fault\n");
			break;
		case 2:
			faultPrintSerial("mpu fault\n");
			break;
		case 3:
			faultPrintSerial("bus fault\n");
			break;
		case 4:
			faultPrintSerial("usage fault\n");
			break;
		default:
			faultPrintSerial("unknown\n");
			break;
		}
		faultPrintSerial("SP_Process = ");
		faultPrintSerialHex((unsigned int)fault_args);

		// Print register values
		faultPrintSerial("R0 = ");
		faultPrintSerialHex((unsigned long)fault_args[0]);
		faultPrintSerial("R1 = ");
		faultPrintSerialHex((unsigned long)fault_args[1]);
		faultPrintSerial("R2 = ");
		faultPrintSerialHex((unsigned long)fault_args[2]);
		faultPrintSerial("R3 = ");
		faultPrintSerialHex((unsigned long)fault_args[3]);
		faultPrintSerial("R12 = ");
		faultPrintSerialHex((unsigned long)fault_args[4]);
		faultPrintSerial("LR = ");
		faultPrintSerialHex((unsigned long)fault_args[5]);
		faultPrintSerial("PC = ");
		faultPrintSerialHex((unsigned long)fault_args[6]);
		faultPrintSerial("PSR = ");
		faultPrintSerialHex((unsigned long)fault_args[7]);

		faultPrintSerial("BFAR = ");
		faultPrintSerialHex(*((volatile unsigned long *)(0xE000ED38))); // BusFault status register
		faultPrintSerial("CFSR = ");
		faultPrintSerialHex(*((volatile unsigned long *)(0xE000ED28))); // Configurable (usage, bus, and mem. Manage faults)
		faultPrintSerial("HFSR = ");
		faultPrintSerialHex(*((volatile unsigned long *)(0xE000ED2C))); // Hard fault
		faultPrintSerial("DFSR = ");
		faultPrintSerialHex(*((volatile unsigned long *)(0xE000ED30))); // Data fault
		faultPrintSerial("AFSR = ");
		faultPrintSerialHex(*((volatile unsigned long *)(0xE000ED3C))); // Auxiliary fault

		// Delay loop to reprint the crash log
		for (i = 0; i < 0xFFFFFE; i++) {
			i++;
			i--;
		}
	}
}

void __attribute__ ((naked)) __cs3_isr_hard_fault (void) {
	/* Assume that main stack frame problem is unrecoverable and select only the process stack. */
	__asm volatile ("	mrs r0, psp										\n"  /* Only get process tack pointer */
					"	mov r1, #1										\n"  /* Load fault type to second argument */
					"   ldr r4, pFaultHandler   						\n"
					"   bx r4			         		           		\n"
					"													\n"
					"	.align 2										\n"
					"   pFaultHandler: .word faultHandler				\n"
			        );
}
