/**
 * @file bootloader.h
 *
 * @brief R-One Bootloader main
 *
 * @since July 26, 2013
 * @author William Xie, James McLurkin
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#ifdef PART_LM3S8962
	#if defined(RONE_V9)
		#warning "Compiling for rone_v9"
	#elif defined(RONE_V12)
		#warning "Compiling for rone_v12"
	#elif defined(RONE_IRBEACON)
		#warning "Compiling for IR beacon"
	#else
		#error "robot version not defined. one of RONE_V9, RONE_V12, or RONE_IRBEACON"
	#endif
#endif

// Includes
#include "System/typedefs.h"
#include "System/spi.h"
#include "System/xmodem.h"
#include "System/crc.h"
#include "System/radioProgramming.h"
#include "InputOutput/blinkyLed.h"
#include "InputOutput/buttons.h"
#include "InputOutput/radio.h"
#include "SerialIO/serial.h"


#define SYSCTL_CLOCK_FREQ		50000000
// Flash size = 256 KB = program + bootloader + state
// Linkerscript need to be changed as well
#define MAX_PROGRAM_SIZE		232
#define MAX_BL_SIZE				23
// Bootloader address values
// Bootloader high memory values (be sure to change system.h values too)
#define OS_START_ADDRESS 		0x00000000
#define BL_START_ADDRESS		(OS_START_ADDRESS + (MAX_PROGRAM_SIZE * 1024))
#define BL_STATE_BLOCK_ADDRESS	(BL_START_ADDRESS + (MAX_BL_SIZE * 1024))
#define BL_STATE_WORD_ADDRESS 	(BL_START_ADDRESS + ((MAX_BL_SIZE + 1) * 1024) - (4 * 1))
#define BL_STATE_SUBNET_ADDRESS (BL_START_ADDRESS + ((MAX_BL_SIZE + 1) * 1024) - (4 * 2))
// Bootloader low memory values
//#define OS_START_ADDRESS 		0x00008000
//#define BL_START_ADDRESS		0x00000000
//#define BL_STATE_BLOCK			0x00007C00
//#define BL_STATE_WORD 			0x00007FFC
// Bootloader state values
#define BL_STATE_NORMAL			0xFFFFFF00
#define BL_STATE_HOST			0xFFFFFF01
#define BL_STATE_RECEIVE		0xFFFFFF02
#define BL_STATE_LIMBO			0xFFFFFF03
#define BL_STATE_XMODEM			0xFFFFFF05
#define BL_STATE_BOOT			0xFFFFFF0a


// Bottomboard communication constants
#define MSP430_PAYLOAD_LENGTH			22
#define MSP430_CHECKSUM_LENGTH			1
#define MSP430_CODE_LENGTH				3
#define MSP430_MSG_LENGTH				(MSP430_CODE_LENGTH + MSP430_PAYLOAD_LENGTH + MSP430_CHECKSUM_LENGTH)
#define MSP430_CMD_COMMAND_IDX			0 + MSP430_CODE_LENGTH
#define MSP430_CMD_COMMAND_NORMAL		0x00
#define MSP430_CMD_COMMAND_SHUTDOWN		0x01
#define MSP430_CMD_COMMAND_RESET	    0x0A
#define MSP430_CMD_COMMAND_REPROGRAM    0xA1
#define MSP430_CMD_CHECKSUM_IDX			MSP430_MSG_LENGTH - 1
extern const uint8 MSP430Code[];

// From FreeRTOS
#define configKERNEL_INTERRUPT_PRIORITY 		( ( unsigned char ) 7 << ( unsigned char ) 5 )	/* Priority 7, or 255 as only the top three bits are implemented.  This is the lowest priority. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( ( unsigned char ) 5 << ( unsigned char ) 5 )  /* Priority 5, or 160 as only the top three bits are implemented. */

/* Size of the stack allocated to the uIP task. */
#define BASIC_TASK_STACK_SIZE            	(configMINIMAL_STACK_SIZE * 3)
#define SYSTEM_INTERRUPT_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY + ( ( unsigned char ) 2 << ( unsigned char ) 5 ))

extern uint8 roneID;
#define ROBOT_ID_MIN					1
#define ROBOT_ID_MAX					255


/**
 * @brief Branch to program or bootloader using starting address
 * @param ulStartAddr Starting address of the main robot program
 * @returns void
 */
void bootOS(uint32 ulStartAddr);

/**
 * @brief Initialize the minimal amount of hardware for the bootloader to function
 * @returns void
 */
void systemInit(void);


/**
 * @brief Gets the robot ID.
 *
 * roneID is assigned robot ID. If robot is uninitialized, roneID is DEFAULT_RONEID.
 * The ID is set using LM Flash.  The format for the ID is: 00-50-C2-00-01-XX
 * where XX is the robotID
 * @returns void
 */
void systemIDInit(void);


/**
 * @brief Return the ID for this robot
 * @return Robot's ID
 */
uint8 getSelfID(void);


/**
 * @brief Systick backed delay in milisecond increment (imprecise)
 * @param delay Delay time (ms)
 * @return void
 */
void systemDelay(uint32 delay);


/**
 * @brief Systick backed delay in 10 microseconds increment (imprecise)
 * @param delay Delay time (10 us)
 * @return void
 */
void systemDelayTenMicroSecond(uint32 delay);


/**
 * @brief Edit the bootloader state words (subnet and bootloader state)
 * @param state The value of the bootloader state
 * @return void
 */
void writeBootloaderState(unsigned long state);


/**
 * @brief Read the bootloader state word
 * @return the state value
 */
uint32 readBootloaderState();


/**
 * @brief Read the bootloader state word
 * @return the state value
 */
uint32 readBootloaderSubnet();


/*
 * @brief Reset the top board by transmitting a reset message to bottom board.
 * @return void
 */
void hardReset();


/**
 * @brief print out the string in parameter over serial. Serial com must first be initialized.
 */
void faultPrintSerial(const char *string);


/**
 * @brief print out hex value in parameter over serial. Serial com must first be initialized.
 * The hex value is at most 8 characters long. The printout value is trailed by \n and \0.
 */
void faultPrintSerialHex(unsigned long num);


/**
 * @brief Simple itoa then print over serial. Very similar to faultPrintSerialHex.
 *  The printout value is trailed by \0.
 */
void faultPrintSerialDec(unsigned long num);
#endif /* BOOTLOADER_H_ */
