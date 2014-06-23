/**
 * @file system.h
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

/******** Defines ********/

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
// Bootloader state values
#define BL_STATE_NORMAL			0xFFFFFF00
#define BL_STATE_HOST			0xFFFFFF01
#define BL_STATE_RECEIVE		0xFFFFFF02
#define BL_STATE_LIMBO			0xFFFFFF03
#define BL_STATE_XMODEM			0xFFFFFF05
#define BL_STATE_BOOT			0xFFFFFF0a

/* Size of the stack allocated to the uIP task. */
#define BASIC_TASK_STACK_SIZE            	(configMINIMAL_STACK_SIZE * 3)
#define SYSTEM_INTERRUPT_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY + ( ( unsigned char ) 2 << ( unsigned char ) 5 ))
//#define SYSTEM_INTERRUPT_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY + 1)
#define SYSTEM_INTERRUPT_PRIORITY_IR		(configMAX_SYSCALL_INTERRUPT_PRIORITY + ( ( unsigned char ) 1 << ( unsigned char ) 5 ))

/* Task priorities. */
#define SERIALIO_TASK_PRIORITY				( tskIDLE_PRIORITY + 6 )
#define RADIOCOMMAND_TASK_PRIORITY			( tskIDLE_PRIORITY + 6 )
#define HEARTBEAT_TASK_PRIORITY				( tskIDLE_PRIORITY + 5 )
#define RPRINTFTERMINAL_TASK_PRIORITY		( tskIDLE_PRIORITY + 4 )
#define MIDI_TASK_PRIORITY					( tskIDLE_PRIORITY + 4 )
#define NEIGHBORS_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define BACKGROUND_TASK_PRIORITY			( tskIDLE_PRIORITY)

// MSP430 SPI States
#define MSP430_SPI_NORMAL_OPERATION  0
#define MSP430_SPI_BOOT_LOADER_MODE  1

/******** Functions ********/

/* TODO: QUILLAN: Look into storing the error messages into something so that you can plug the robot into a computer, send 'er' through the serial port and it will spew the last error messages. */

#define systemPrintStartup() 				_systemPrintStartup(__FILE__)
#define error(errMsg) 						_error(__FILE__, __LINE__, errMsg)
//#define warning(warningMessage)				_warning(__FILE__, __LINE__, warningMessage)


/**
 * @brief Flag that the heartbeat task needs to print the info once comms are valid
 * @param fileName - char pointer of file name
 * @returns void
 */
void _systemPrintStartup(char* fileName);


/**
 * @brief Create log of messages and make error noise.
 *
 * Takes new error message and adds it to a log of error messages.
 * Makes error noise.
 * @param fileName name of file
 * @param lineNumber line number
 * @param errMsg error message that user specifies
 * @returns void
 */
void _error(char* fileName, int lineNumber, char* errMsg);


//void _warning(char *fileName, int lineNumber, char *errMsg);


//Static Varible of roneID
extern uint8 roneID;


/**
 * @brief Initializes the r-one hardware.
 *
 * Initalizes roneID, charger, blinky, buttons, IRBeacon, SPI, LED, and serial.
 * the heartbeat light blinks three times after the initializations are done.
 * Initializes encoder, light sensor, motor, gyro, accelerometer, IR_comms, radio, ad cfprintf, sd card.
 * Prints out the date, time, and roneID after everything is initialized.
 * Rone starts heartbeat after this initialization.
 * @returns void
 */
void systemInit(void);


/**
 * @brief Background tasks performed during each heartbeat.
 *
 * Each heartbeat = every 16 milliseconds.
 * Updates blinky, IRBeacon, leds, accelerometer, motor velocity, pose, and motor command timer.
 * @returns void
 */
void systemHeartbeatTask(void* parameters);


/**
 * @brief Turn off the main power supply
 *
 * Shuts down everything in the Rone robot. This processor is turned off, the
 * MSP430 will be put into LPM4 and all LEDs and motors are also turned off,
 * along with the main power supply.
 * @returns void
 */
void systemShutdown(void);


/**
* @brief Print the heap and stack usage.
*
* @returns void
*/
void systemPrintMemUsage(void);


/**
 * @brief Check USBlevel.
 *\internal
 *TODO fix code
 *DOES THIS NEED TO BE DELETED?
 *\endinternal
 * Unfinished, and returns FALSE unconditionally.
 *
 * @returns FALSE
 */
uint32 systemUSBConnected(void);


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
 * @brief simple counting delay
 * @param delay the amount of time to delay the system
 * @returns void
 */
void systemDelay(uint32 delay);


/**
 * @brief Sets Operational state acodring to paramter.
 * @param state Operational state that needs to be set eg. MSP430_SPI_NORMAL_OPERATION
 * @return void
 */
void setMSP430SPIOperationState(uint8 state);


/**
 * @brief Branch to program or bootloader using starting address
 * @param ulStartAddr Starting address of the main robot program
 * @returns void
 */
void bootOS(uint32 ulStartAddr);


// RONE_V9+ functions.  Will not work on v6
/*
 * @brief Initialize the system IO
 * Systems includes battery voltage, usb voltage, powerbutton, mspversion and prevVal .RONE_V9+ functions.  Will not work on v6
 * @return void
 */
void systemIOInit(void);


/**
 * Update the battery voltage.
 *
 * @param val the new voltage value
 * @returns void
 */
void systemBatteryVoltageUpdate(uint8 val);


/**
 * @brief Retrieve the battery voltage.
 * The voltage will be over 10
 * @returns batteryVoltage the battery voltage
 */
float systemBatteryVoltageGet(void);


/**
 * @brief Retrieve the battery voltage and place ones and tenths values in appropriate spaces.
 *
 * @param onesPtr the pointer to where the ones value of the voltage will be stored
 * @param tenthsPtr the pointer to where the tenths value of the voltage will be stored
 * @returns void
 */
void systemBatteryVoltageGet2(uint8* onesPtr, uint8* tenthsPtr);


/**
 * @brief Update the USB voltage.
 *
 * @param val the new USB voltage value
 * @returns void
 */
void systemUSBVoltageUpdate(uint8 val);


/**
 * @brief Get the USB voltage.
 *
 * @returns usbVoltage the USB voltage
 */
uint8 systemUSBVoltageGet(void);


/**
 * @brief Update the power button.
 *
 * @param val the new power button value
 * @returns void
 */
void systemPowerButtonUpdate(uint8 val);


/**
 * @brief Get the power button value.
 *
 * @returns the power button value
 */
uint8 systemPowerButtonGet(void);


/**
 *@brief Update the MSP hardware and software version.
 * If update is same as prevVal and is not zero it will update mspVersion
 *@returns void
 */

void systemMSPVersionUpdate(uint8 val);

/**
 *@brief Get the MSP version.
 *
 *@returns mspVersion the MSP version
 */
uint8 systemMSPVersionGet(void);

/**
 *@brief Get the MSP version.
 *
 *@returns mspVersion the MSP version
 */
uint8 systemMSPVersionHardwareGet(void);


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


/**
 * @brief Stop all interfering subcomponents and branch to bootloader
 */
void bootloading();


#endif /* SYSTEM_H_ */
