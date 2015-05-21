/*
 * @file system.c
 * @brief System-level code: initialize and shutdown the robot, monitor power, set delays, etc.
 * @since Jul 26, 2010
 */

#define HEARTBEAT_PERIOD 16

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(PART_LM3S8962)
	#include "inc/lm3s8962.h"
#elif defined(PART_LM3S9B92)
	#include "inc/lm3s9b92.h"
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
#include "driverlib/pwm.h"

#include "roneos.h"

#define	DEFAULT_RONEID				126
#define ERROR_LIST_SIZE				10
#define WARNING_LIST_SIZE			(ERROR_LIST_SIZE)

/******** structs ********/
/*
 * @brief Error message includes information to track error
 */
typedef struct errorMsg {
	char *fileName;
	int lineNumber;
	char *errorMessage;
	uint32 timeStamp;
} errorMsg;

/*
 * @brief Warning message includes information to track warning
 */
typedef struct warningMessage {
	char *fileName;
	int lineNumber;
	char *warningMessage;
	uint32 timeStamp;
} warningMessage;


/******** external functions ********/

// we define these prototypes here because these functions are internal, and not
// part of the external API
void radioIntHandler(void);
void uartIRQHandler(void);
void irCommsHandler(void);
void IRBeaconIRQHandler(void);
void msp430InterruptHandler(void);
void msp430BSLHandler(void);
void ledsInit(void);
void ledsUpdate(void);



/******** variables ********/

static errorMsg errorMessages[ERROR_LIST_SIZE];
static warningMessage warningMessages[WARNING_LIST_SIZE];
uint8 roneID;

uint8 msp430SPIOperationState = MSP430_SPI_NORMAL_OPERATION;


void playError();

/*
 * FreeRTOS API hooks.
 */
void vApplicationIdleHook(void) {
}

void vApplicationTickHook(void) {
	sputcharTickHook();
}

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	//TODO add panic light and serial output
	for(;; );
}

// random variable for FreeRTOS
volatile unsigned long ulHighFrequencyTimerTicks;

void systemHeartbeatTask(void* parameters);

static volatile boolean systemPrintStartupInfoFlag = FALSE;
static char * systemPrintStartupFilename;



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

	// Set the vector table to the beginning of the program in flash.
	HWREG(NVIC_VTABLE) = ulStartAddr;

	__asm("    ldr     r1, [r0]			\n" /* Load the stack pointer from the application's vector table. */
	      "    mov     sp, r1			\n"
		  "    ldr     r0, [r0, #4]		\n" /* Load the initial PC from the application's vector table and branch to the application's entry point */
	      "    bx      r0				\n"
		);
}


/*
 * @brief Stop all interfering subcomponents and branch to bootloader
 */
void bootloading() {
	MAP_IntMasterDisable();
	// Turn off wheels
	MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);
	MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);
	MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, 0);
	MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 0);

	// Branch to bootloader
	bootOS(BL_START_ADDRESS);
}


/*
 * @brief Initializes the r-one hardware.
 *
 * Initalizes roneID, charger, blinky, buttons, IRBeacon, SPI, LED, and serial.
 * the heartbeat light blinks three times after the initializations are done.
 * Initializes encoder, light sensor, motor, gyro, accelerometer, IR_comms, radio, ad cfprintf, sd card.
 * Prints out the date, time, and roneID after everything is initialized.
 * Rone starts heartbeat after this initialization.
 * @returns void
 */
void systemInit(void) {
	int i;
	unsigned long val;

	MAP_IntMasterDisable();

	// increase LDO voltage so that PLL operates properly
	MAP_SysCtlLDOSet(SYSCTL_LDO_2_75V);
	#ifdef PART_LM3S8962
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
	#endif

	#ifdef PART_LM3S9B92
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	#endif

	MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	// startup core hardware needed before FreeRTOS is started
	systemIDInit();
	blinkyLedInit();
	buttonsInit();
	srand(roneID);
	#if defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12)
	charger_init();
	IRBeaconPreinit();
	#elif defined(RONE_IRBEACON)
	pwmInitializeBeacon();
	#endif
	systemPrintStartupInfoFlag = FALSE;

	// triple flash the blinky LED for the heartbeat
	for (i = 0; i < 3; i++) {
		uint32 j;
		blinkyLedSet(1);
		for (j = 0; j < 150000;) {
			j++;
		}
		blinkyLedSet(0);
		for (j = 0; j < 250000;) {
			j++;
		}
	}


	// startup the rest of the hardware
#if defined(RONE_V9) || defined(RONE_V12)
	serialInit();
	msp430SPIOperationState = MSP430_SPI_NORMAL_OPERATION;
	SPIInit();
	serialCommandInit();
	radioInit();
	radioCommandInit(); //osTaskCreate has a TODO (silly word hack)
	systemCommandsInit();
	cprintfInit();
	rprintfInit();
	encoderInit();
	lightSensorInit();
	motorInit();
	ledsInit();
	IRBeaconInit();
	irCommsInit();
	systemIOInit();
	msp430Init();  // this needs to be called after irCommsInit because they share the same timer code
	gyro_init();
	accelerometer_init();
	audioInit();
	//neighborsInit(300);
	MIDIInit();
	MIDIFilePlay(MIDIFile_PowerOn); //has a TODO in the function - can't find referenced code
#endif

#if defined(RONE_IRBEACON)
	serialInit();
	serialCommandInit();
	systemCommandsInit();
	cprintfInit();
	ledsInit();
	IRBeaconInit();
	irCommsInit();
#endif

	// Run bootloader routine
//	bootloading();

	// 512 bytes of stack is ok to run, but not to print
	//osTaskCreate(systemHeartbeatTask, "heartbeat", 512, NULL, HEARTBEAT_TASK_PRIORITY);
	osTaskCreate(systemHeartbeatTask, "heartbeat", 1536, NULL, HEARTBEAT_TASK_PRIORITY);
}


#if (defined(RONE_V9) || defined(RONE_V12))
/*
 * @brief Turn off the main power supply
 *
 * Shuts down everything in the Rone robot. This processor is turned off, the
 * MSP430 will be put into LPM4 and all LEDs and motors are also turned off,
 * along with the main power supply.
 * @returns void
 */
void systemShutdown(void) {
	// Turn off everything
	msp430SystemShutdownCommand();
}

#endif /* (defined(RONE_V9) || defined(RONE_V12)) */

/*
 * @brief Gets the file name from path.
 *
 * @param filepathString the file path
 * @returns a pointer that points to the file name
 */
static char* sysGetFilenameFromPath(char* filepathString) {
	return (strrchr(filepathString, '/') + 1);
}

void _systemPrintStartupPrint(char * fileName) {
	uint32 i;
	uint8 ones, tenths;
	cprintf("\nrobot id: %0d\n", roneID);

	#if defined(RONE_V6)
	cprintf("robot hardware V6, id: %d\n", roneID);
	#elif defined(RONE_V9)
	systemBatteryVoltageGet2(&ones, &tenths);
	cprintf("robot hardware V9\n");
	cprintf("roneos build %s, %s\n", __DATE__, __TIME__);
	cprintf("bottom board software version: %0d\n", systemMSPVersionGet());
	cprintf("bottom board hardware version: %0d\n", systemMSPVersionHardwareGet());
	cprintf("battery voltage: %1d.%1d\n", ones, tenths);
	#elif defined(RONE_V12)
	systemBatteryVoltageGet2(&ones, &tenths);
	cprintf("robot hardware V12\n");
	cprintf("roneos build %s, %s\n", __DATE__, __TIME__);
	cprintf("bottom board software version: %0d\n", systemMSPVersionGet());
	cprintf("bottom board hardware version: %0d\n", systemMSPVersionHardwareGet());
	cprintf("battery voltage: %1d.%1d\n", ones, tenths);
	#endif
	cprintf("%s\n\n", sysGetFilenameFromPath(fileName));
}

void _systemPrintStartup(char* fileName) {

	#if defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12)
	// Flag that the heartbeat task needs to print the info once comms are valid
	systemPrintStartupInfoFlag = TRUE;
	systemPrintStartupFilename = fileName;

	#elif defined(RONE_IRBEACON)
	cprintf("\nrobot %0d\n", roneID);
	cprintf("IR beacon running roneos build %s, %s\n", __DATE__, __TIME__);
	cprintf("%s\n\n", sysGetFilenameFromPath(fileName));
	#endif
}
//Should we do this?
/*
 * Wants to log message (list of last ten error messages).
 *
 * Keep pointer to string sent into it.
 *
 * Heartbeat threads wants to deal with printing the errors.
 *
 * Make an easy to look at data structure that is an error struct
 * that contains the
 *
 * system.h
 *
 * this becomes _error()
 *
 * copies in file, line, and a pointer to the string.
 *
 * TODO: Make the thing squawk loudly
 */


/*
 * @brief Create log of messages and make error noise.
 *
 *Takes new error message and adds it to a log of error messages.
 *Makes error noise.
 *@param filename name of file
 *@param lineNumber line number
 *@param errMsg error message that user specifies
 *@returns
 */
void _error(char *fileName, int lineNumber, char *errMsg)
{
	/*
	 * This is the number of errors that have happened this run time.
	 * If more than ERROR_LIST_SIZE errors occur, then the most recent error
	 * will be at the end and numberOfErrors - ERROR_LIST_SIZE will be at the
	 * front of the list.
	 */
	static int errorCount = 0;
	int i;
	errorMsg error;

	/* TODO: ERROR: Make it squak here. */
	//playError();

	/* Create the error message structure and fill it out. */
	error.errorMessage = errMsg;
	error.fileName = fileName;
	error.lineNumber = lineNumber;
	error.timeStamp = osTaskGetTickCount();

	/*
	 * If the list hasn't been filled, fill in the next slot.
	 *
	 * Otherwise, move all of the messages forward one slot and
	 * insert the new message at the end.
	 */
	if (errorCount < ERROR_LIST_SIZE) {
		errorMessages[errorCount] = error;
	} else {
		for (i = 0; i <= (ERROR_LIST_SIZE - 1); i++) {
			errorMessages[i] = errorMessages[i + 1];
		}
		errorMessages[ERROR_LIST_SIZE - 1] = error;
	}
	errorCount++;

	/*
	 * TODO: ERROR: Make the heartbeat thread print the errors.
	 * This lets us call the error function during ISRs.
	 */
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

#define SYSCTL_DELAY_USEC 		16
/*
 * @brief simple counting delay
 * @param delay the amount of time to delay the system
 * @returns void
 */
void systemDelayUSec(uint32 delay){
	SysCtlDelay(delay * SYSCTL_DELAY_USEC);
}

#define HEARTBEAT_10HZ_COUNTER	6

/*
 * @brief Background tasks performed during each heartbeat.
 *
 * Each heartbeat = every 16 milliseconds.
 * Updates blinky, IRBeacon, leds, accelerometer, motor velocity, pose, and motor command timer.
 * @returns void
 */
void systemHeartbeatTask(void* parameters) {
	uint8 heart_beat_10hz_timer = HEARTBEAT_10HZ_COUNTER;
	portTickType lastWakeTime;
	long i = 0;

	lastWakeTime = osTaskGetTickCount();

	for (;;) {
		// blink the blinky LED
		blinkyUpdate();

#ifdef PART_LM3S8962

		#if defined(RONE_V6)
		ledsUpdate();
		ir_beacon_update();
		accelerometerUpdate();

		#elif defined(RONE_V9) || defined(RONE_V12)
		blinkySystemUpdate();
		ledsUpdate();

		// run the velocity controller
		motorVelocityUpdate();

		/* Increase the remote control timer if remote control mode is on */
		if (rcMode == RC_MODE_ON) {
			rcTimer += 16;
			if (rcTimer >= RC_TIMEOUT) {
 			rcMode = RC_MODE_OFF;
			}
		}


		// estimate the pose
		encoderPoseUpdate();


		// Print the startup information once it arrives
		if(msp430GetCommsValid() && systemPrintStartupInfoFlag && systemMSPVersionGet() != 0){
			systemPrintStartupInfoFlag = FALSE;
			_systemPrintStartupPrint(systemPrintStartupFilename);
		}

		// Check the version number on the MSP430 and update it if it is needed
		// Always update if the version is different, even if it is lower
		// Check the hardware version of the MSP430
		// Only update if the hardware version matches the local hardware version
		if(msp430GetCommsValid() &&
			systemMSPVersionGet() != 0 &&
			systemMSPVersionGet() < msp430BSLGetLocalVersionNumber() &&
			systemMSPVersionHardwareGet() != 0 &&
			systemMSPVersionHardwareGet() == msp430BSLGetLocalVersionHardwareNumber()) {

			osTaskDelay(500);

			// Suspend all the other tasks to that nothing interferes
			// During this time interrupts will still operate and the kernel
			// tick count will be maintained.
			osTaskSuspendAll();

			ledsSetAll(0);

			// Send the re-program command. This actually starts the process
			msp430SystemReprogramCommand();

			// Set the MSP430 comms to invalid
			msp430SetCommsValid(FALSE);

			// Wait a little bit
			for(i = 0; i < 10000; i++){}

			// Wait until we are done
			while(msp430SPIOperationState != MSP430_SPI_NORMAL_OPERATION || !msp430GetCommsValid()) {
				// Do nothing!
			}

			// The operation is complete.  Restart the kernel.
			osTaskResumeAll();
		}
		// If hardware does not match, LEDs will blink angrily

		else if(systemMSPVersionHardwareGet() != 0 &&
				systemMSPVersionHardwareGet() != msp430BSLGetLocalVersionHardwareNumber() ){
			ledsSetPattern(LED_ALL, LED_PATTERN_BLINK, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
		}

		#elif defined(RONE_IRBEACON)
		blinkySystemUpdate();
		ledsUpdate();
		#endif

#endif

		// 10 hz interrupt functions
		heart_beat_10hz_timer--;
		if (heart_beat_10hz_timer == 0) {
			heart_beat_10hz_timer = HEARTBEAT_10HZ_COUNTER;
			#if (defined (RONE_V6) || defined (RONE_V9) || defined (RONE_V12))
			// enable charge mode if the motors have not been driven in a while
			motorCommandTimerUpdate();
			#endif
		}

		osTaskDelayUntil(&lastWakeTime, HEARTBEAT_PERIOD);
	}
}

/*
 * @brief Print the heap and stack usage.
 *
 * @returns void
 */
void systemPrintMemUsage(void) {
	uint8 i;

	//
	cprintf("FreeRTOS heap: %d free bytes\n", xPortGetFreeHeapSize());
	cprintf("%d tasks:\n", (int) osTaskGetNumberOfTasks());
	for (i = 0; i < sysTaskCount; ++i) {
		cprintf("  %s: stack max %d/%d\n", osTaskGetName(sysTaskHandles[i]),
				sysTaskStackSizes[i] - (uint32) osTaskGetStackHighWaterMark(
						sysTaskHandles[i]), sysTaskStackSizes[i]);
	}
}


void setMSP430SPIOperationState(uint8 state){
	msp430SPIOperationState = state;
}


/*
 * @brief Edit the bootloader state words (subnet and bootloader state)
 * @param state The value of the bootloader state
 * @return void
 */
void writeBootloaderState(unsigned long state) {
	uint32 tempBootloaderSubnet = radioCommandGetLocalSubnet();
	// address of the last word (4 bytes) of state flash = 0x0003FFFC
	// address of last 1kb block = 0x0003FC00
	FlashErase(BL_STATE_BLOCK_ADDRESS);
	FlashProgram(&state, BL_STATE_WORD_ADDRESS, sizeof(uint32));
	// address of the second to the last word in state flash
	FlashProgram(&tempBootloaderSubnet, BL_STATE_SUBNET_ADDRESS, sizeof(uint32));
}


/*
 * @brief Read the bootloader state word
 * @return the state value
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


// Interrupt handler stubs
void __attribute__ ((interrupt)) __cs3_isr_uart0(void) {
	uartIRQHandler();
}


void __attribute__ ((interrupt)) __cs3_isr_timer1a (void) {
	irCommsHandler();
}


#if defined(RONE_V6) || defined(RONE_V12)
void __attribute__ ((interrupt)) __cs3_isr_timer2a(void) {
	IRBeaconIRQHandler();
}
#endif

#if (defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12))

void __attribute__ ((interrupt)) __cs3_isr_gpio_c(void) {
	radioIntHandler();
}

void __attribute__ ((interrupt)) __cs3_isr_timer1b (void) {
	if(msp430SPIOperationState == MSP430_SPI_NORMAL_OPERATION){
		msp430InterruptHandler();
	} else if(msp430SPIOperationState == MSP430_SPI_BOOT_LOADER_MODE){
		msp430BSLHandler();
	}
}


/*
 * @brief print out the string in parameter over serial. Serial com must first be initialized.
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
 * The hex value is at most 8 characters long. The printout value is trailed by \n and \0.
 */
void faultPrintSerialHex(unsigned long num)
{
	unsigned long temp = num;
	short i, j, exp = 0;
	char h[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	char hex[2 + 8 + 2];

	hex[0] = '0';
	hex[1] = 'x';

	while( temp >= 16)
	{
		exp++;
		temp /= 16;
	}

	// For each digit in hex
	for ( i = 0; i <= exp; ++i)
	{
		temp = num;

		// Divide the right amount of time to get remainder for each digit
		for ( j = i; j <= exp; ++j)
		{
			if ( j == exp )
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
 * @brief fault handler that is called in a fault trap. It pulls the pushed register values from the stack
 * and prints them out over serial. Special fault register values are also printed.
 */
void faultHandler(unsigned int *fault_args, unsigned int exType) {
	int i;

	while (1) {
		// Print the fault message over serial (serial must be initialized first)
		faultPrintSerial("\n- Fault handler -\n\n");
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


#endif
