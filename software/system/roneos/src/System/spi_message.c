/*
 * @file spi_message.c
 * @brief SPI commands for the MSP430
 * @since Apr 2, 2012
 * @author James McLurkin
 */
#include <string.h>

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

#if defined(RONE_V6)
void msp430Init(void) {
}

void msp430InterruptEnable(void) {
}

void msp430InterruptDisable(void) {
}
#endif //#if defined(RONE_V6)


#if (defined(RONE_V9) || defined(RONE_V12))
// SPI states
#define MSPSPI_STATE_XMIT_BYTE				0
#define MSPSPI_STATE_DESELECT_SPI			1

// Timer Periods in microseconds
#define MSP430_SPI_DOUBLEBYTE_PERIOD		800
#define MSP430_SPI_DESELECT_PERIOD			400
#define MSP430_SPI_BYTE_PERIOD				20000

// Bottomboard
#define MSP430_CHECKSUM_LENGTH			1
#define MSP430_CODE_LENGTH				3
#define MSP430_MSG_LENGTH				(MSP430_CODE_LENGTH + MSP430_PAYLOAD_LENGTH + MSP430_CHECKSUM_LENGTH)
const uint8 MSP430Code[] = {'B', 'O', 'T'};

#define MSP430_MSG_BUMPER_IDX	 		0 + MSP430_CODE_LENGTH
#define MSP430_MSG_ACCEL_START_IDX		1 + MSP430_CODE_LENGTH
#define MSP430_MSG_GYRO_START_IDX		7 + MSP430_CODE_LENGTH
#define MSP430_MSG_VBAT_IDX				13 + MSP430_CODE_LENGTH
#define MSP430_MSG_VUSB_IDX				14 + MSP430_CODE_LENGTH
#define MSP430_MSG_POWER_BUTTON_IDX		15 + MSP430_CODE_LENGTH
#define MSP430_MSG_VERSION_IDX			16 + MSP430_CODE_LENGTH
#define MSP430_MSG_CHECKSUM_IDX			MSP430_MSG_LENGTH - 1
/* Commands:
 * 0x00:		Normal Operation, LED and Button Information Sent
 * 0x01:		Shutdown Command
 * 0x02-0x04:	Reserved for SPI firmware updates
 * 0xA0         Set Mode (default C Program)
 * 0xA1         Reprogram Command
 */
#define MSP430_CMD_COMMAND_IDX			0 + MSP430_CODE_LENGTH
#define MSP430_CMD_COMMAND_NORMAL		0x00
#define MSP430_CMD_COMMAND_SHUTDOWN		0x01
#define MSP430_CMD_COMMAND_RESET	    0x0A
#define MSP430_CMD_COMMAND_REPROGRAM    0xA1

// indexes for Normal Command
#define MSP430_CMD_SYSTEM_LED_IDX 		1 + MSP430_CODE_LENGTH
#define MSP430_CMD_BUTTONS_IDX 			2 + MSP430_CODE_LENGTH
#define MSP430_CMD_BEACON_IDX			3 + MSP430_CODE_LENGTH
#define MSP430_CMD_LED_IDX				4 + MSP430_CODE_LENGTH
// LEDs fill data from idx 				4-18
#define MSP430_CMD_CHECKSUM_IDX			MSP430_MSG_LENGTH - 1

#define LED_DATA_LENGTH					15

uint8 MSP430MessageIn[MSP430_MSG_LENGTH];
uint8 MSP430MessageOut[MSP430_MSG_LENGTH];
uint8 MSP430MessageIdx = 0;

// Expand0 extension board
boolean expand0En = FALSE;

#define EXPAND0_CHECKSUM_LENGTH			1
#define EXPAND0_CODE_LENGTH				3
#define EXPAND0_MSG_LENGTH				(EXPAND0_CODE_LENGTH + EXPAND0_PAYLOAD_LENGTH + EXPAND0_CHECKSUM_LENGTH)
const uint8 expand0Code[] = {'R', 'O', 'B'};

uint8 expand0MessageIn[EXPAND0_MSG_LENGTH];
uint8 expand0MessageInBuf[EXPAND0_MSG_LENGTH];
uint8 expand0MessageOut[EXPAND0_MSG_LENGTH];
uint8 expand0MessageOutBuf[EXPAND0_MSG_LENGTH];
uint8 expand0MessageIdx = 0;
uint8 expand0MessageOutBufLock = 0;
uint8 expand0MessageInBufLock = 0;

// SPI global variables
uint8 MSPSPIState = MSPSPI_STATE_XMIT_BYTE;
uint8 systemMSP430Command = MSP430_CMD_COMMAND_NORMAL;
uint8 bootloaderSet = FALSE;
uint32 checksumFailure = 0;
volatile boolean systemMSP430CommsValid = FALSE;

// Bottomboard/expansion toggle
#define MSP_BOTTOM_BOARD_SEL	0
#define MSP_EXPAND0_BOARD_SEL	1
uint8 MSPSelect = 0;

// For testing
uint8 showError = 0;

/*
 * @brief Initialize MSP430.
 *
 * use 400us interrupt for SPI comms.  This lets us xmit our 24-byte message in 9.6
 * ms.  At the end, we use a 32us interrupt for SPI comms.  This is about as fast
 * as the MSP430 can read bytes from the buffer.  This double byte signals the end of the
 * message for synchronizing the two processors
 * @returns void
*/
void msp430Init(void) {
    // Set up the message index
    MSP430MessageIdx = 0;

    // Default expand0 to off
    expand0Disable();

	// timer 1a is used for the ir interrupt.  timer 1b is used for the MSP430 message interrupt
	//ir_init initializes the timer 1, so timer1 shouldn't be enabled and configured here

	//MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	//MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_ONE_SHOT);
    // end shared timer init code

    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, MSP430_SPI_BYTE_PERIOD);
    MAP_TimerEnable(TIMER1_BASE, TIMER_B);

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
	IntPrioritySet(INT_TIMER1B, SYSTEM_INTERRUPT_PRIORITY);
    MAP_IntEnable(INT_TIMER1B);

    // Have a flag to show the first valid communication from the MSP430
    systemMSP430CommsValid = FALSE;

    // Set up normal operations between the MSP430 and the 8962
    systemMSP430Command = MSP430_CMD_COMMAND_NORMAL;

    checksumFailure = 0;
}


/*
 * @brief Enable MSP430 interrupt.
 *
 * @returns void
 */
void msp430InterruptEnable(void) {
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
}


/*
 * @brief Gets systemMSP430CommsValid
 *
 * @returns systemMSP430CommsValid
 */
boolean msp430GetCommsValid(void) {
	return systemMSP430CommsValid;
}


/*
 * @brief Sets systemMSP430CommsValid
 * @param newCommsValid - new boolean value to set systemMSP430CommsValid
 * @returns void
 */
void msp430SetCommsValid(boolean newCommsValid) {
	systemMSP430CommsValid = newCommsValid;
}


/*
 * @brief Disable MSP430 interrupt.
 *
 * @returns void
 */
void msp430InterruptDisable(void) {
	volatile uint8 i;
	if (MSPSPIState == MSPSPI_STATE_DESELECT_SPI) {
		i++;
	}
    MAP_TimerIntDisable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
}

/*
 * @brief Build a command for MSP430 message and reset command.
 *
 * Used to build the command for the actual command sent to the MSP430
 * Reset the command to normal mode
 * @returns void
 */
void msp430SystemCommandBuildMessage(uint8* msg) {
	*msg = systemMSP430Command;
	systemMSP430Command = MSP430_CMD_COMMAND_NORMAL;
}

/*
 * @brief Get the command.
 *
 * @returns systemMSP430 the command
 */
uint8 msp430SystemGetCommandMessage(void) {
	return systemMSP430Command;
}


/*
 * @brief Set the command to shutdown.
 *
 * @returns void
 */
void msp430SystemShutdownCommand(void) {
	systemMSP430Command = MSP430_CMD_COMMAND_SHUTDOWN;
}


/*
 * @brief Set the command to reset.
 *
 * @returns void
 */
void msp430SystemResetCommand(void) {
	systemMSP430Command = MSP430_CMD_COMMAND_RESET;
}


/*
 * @brief Set the next command to the MSP430 to be to enter the boot-loader.
 *
 * @returns void
 */
void msp430SystemReprogramCommand(void) {
	systemMSP430Command = MSP430_CMD_COMMAND_REPROGRAM;
}


/*
 * @brief Compute the uint8 sum for the message excluding the code and checksum
 * @param msg SPI message
 * @param codeLen Length of the code
 * @param msgLen Length of the whole message including code and checksum
 * @returns The checksum
 */
static uint8 messageChecksum(uint8* msg, uint8 codeLen, uint8 msgLen) {
	uint8 i;
	uint8 sum = 0;

	for (i = codeLen; i < msgLen - 1; i++) {
		sum += (uint8)msg[i];
	}

	return sum;
}


/*
 * @brief Shifts the buffer array toward the 0th index. The 0th index is discarded
 * @param msg Message buffer
 * @param len Length of the buffer
 * @returns void
 */
static void shiftBuffer(uint8* msg, uint8 len) {
	int i;

	for (i = 0; i < len - 1; i++) {
		msg[i] = msg[i + 1];
	}
}


/*
 * @brief Initialize expand0 SPI
 * @returns void
 */
void expand0Init(void) {
	uint8 i;
	// Pack code
	for (i = 0; i < EXPAND0_CODE_LENGTH; i++) {
		expand0MessageOutBuf[i] = expand0Code[i];
	}
	// Clear payload and checksum
	for (i = EXPAND0_CODE_LENGTH; i < EXPAND0_MSG_LENGTH; i++) {
		expand0MessageOutBuf[i] = 0;
	}

	expand0Enable();
}


/*
 * @brief Enables expand0 SPI
 * @returns void
 */
void expand0Enable(void) {
	expand0En = TRUE;
}


/*
 * @brief Disables expand0 SPI
 * @returns void
 */
void expand0Disable(void) {
	expand0En = FALSE;
}


/*
 * @brief Export incoming payload from expand0 board
 * @param msg Destination for the message export (no code and checksum, length = EXPAND0_PAYLOAD_LENGTH)
 * @returns void
 */
void expand0MsgRead(uint8* msg) {
	expand0MessageInBufLock = 1;

	memcpy(msg, &expand0MessageInBuf[EXPAND0_CODE_LENGTH], EXPAND0_PAYLOAD_LENGTH);

	expand0MessageInBufLock = 0;
}


/*
 * @brief Prepare the next outgoing message to be sent out
 * @param msg Source for the message export (no code and checksum, length = EXPAND0_PAYLOAD_LENGTH)
 * @returns void
 */
void expand0MsgWrite(uint8* msg) {
	expand0MessageOutBufLock = 1;

	uint8 checksum;
	expand0MessageOutBuf[EXPAND0_MSG_LENGTH - 1] = messageChecksum(msg, 0, EXPAND0_PAYLOAD_LENGTH);

	memcpy(&expand0MessageOutBuf[EXPAND0_CODE_LENGTH], msg, EXPAND0_PAYLOAD_LENGTH);

	expand0MessageOutBufLock = 0;
}


/*
 * @brief Bottom MSP430 board ISR
 *
 * @returns void
 */
uint8 SPIBusError_SSIDataPutNonBlocking = 0;
uint8 SPIBusError_SSIDataGetNonBlocking = 0;
uint8 bottomBoardISR() {
	uint32 data, i;
	uint32 timerLoadPeriod;
	uint8 checksum;
	long val;

	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

	// Multiplex SPI (bottomboard/radio)
	if (MSPSPIState == MSPSPI_STATE_XMIT_BYTE) {
		radioIntDisable();
		SPISelectDeviceISR(SPI_MSP430);
		timerLoadPeriod = MSP430_SPI_DESELECT_PERIOD;
	} else if (MSPSPIState == MSPSPI_STATE_DESELECT_SPI) {
		SPIDeselectISR();
		radioIntEnable();
		if (bootloaderSet && MSP430MessageIdx == 0) {
			// Deselect but then set the boot-loader to operate.
			msp430BSLInit();
			bootloaderSet = FALSE;
		} else {
			timerLoadPeriod = MSP430_SPI_BYTE_PERIOD;
		}
	}

	// Set delay
	MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, timerLoadPeriod);
	MAP_TimerEnable(TIMER1_BASE, TIMER_B);

	// Transmit message to bottomboard
	if (MSPSPIState == MSPSPI_STATE_XMIT_BYTE) {
		// Pack new message at the beginning of each cycle
		if (MSP430MessageIdx == 0) {
			// Pack code
			for (i = 0; i < MSP430_CODE_LENGTH; i++) {
				MSP430MessageOut[i] = MSP430Code[i];
			}
			// Build and pack payload
			switch (msp430SystemGetCommandMessage()) {
				case MSP430_CMD_COMMAND_NORMAL:
					blinkySystemBuildMessage(&MSP430MessageOut[MSP430_CMD_SYSTEM_LED_IDX]);
					buttonsBuildMessage(&MSP430MessageOut[MSP430_CMD_BUTTONS_IDX]);
					ledsBuildMessage(&MSP430MessageOut[MSP430_CMD_LED_IDX]);
					break;
				case MSP430_CMD_COMMAND_REPROGRAM:
					bootloaderSet = TRUE;
					break;
				default:
					// Do nothing for shutdown or reset
					break;
			}

			msp430SystemCommandBuildMessage(&MSP430MessageOut[MSP430_CMD_COMMAND_IDX]);
			MSP430MessageOut[MSP430_CMD_CHECKSUM_IDX] =  messageChecksum(MSP430MessageOut, MSP430_CODE_LENGTH, MSP430_MSG_LENGTH);
		}

		// Shift receive buffer
		shiftBuffer(MSP430MessageIn, MSP430_MSG_LENGTH);

		// Put data into the transmit buffer
		val = MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)MSP430MessageOut[MSP430MessageIdx]);
		if (val == 0) {
			SPIBusError_SSIDataPutNonBlocking = 1;
		}

		// Retrieve the received byte
		MAP_SSIDataGet(SSI0_BASE, &data);

		// Put byte the the end of the receive buffer
		MSP430MessageIn[MSP430_MSG_LENGTH - 1] = data;

		// Check to see if we have a complete message, with correct code and checksum
		if (MSP430MessageIdx == 0) {
			// Checkcode
			for (i = 0; i < MSP430_CODE_LENGTH; i++) {
				if (MSP430MessageIn[i] != MSP430Code[i])
					break;
			}

			if (i == MSP430_CODE_LENGTH) {
				// Checksum
				checksum = messageChecksum(MSP430MessageIn, MSP430_CODE_LENGTH, MSP430_MSG_LENGTH);
				if (checksum == MSP430MessageIn[MSP430_MSG_LENGTH - 1]) {
					// Process incoming message
					bumpSensorsUpdate(MSP430MessageIn[MSP430_MSG_BUMPER_IDX]);
					accelerometerUpdate(&MSP430MessageIn[MSP430_MSG_ACCEL_START_IDX]);
					gyroUpdate(&MSP430MessageIn[MSP430_MSG_GYRO_START_IDX]);
					systemBatteryVoltageUpdate(MSP430MessageIn[MSP430_MSG_VBAT_IDX]);
					systemUSBVoltageUpdate(MSP430MessageIn[MSP430_MSG_VUSB_IDX]);
					systemPowerButtonUpdate(MSP430MessageIn[MSP430_MSG_POWER_BUTTON_IDX]);
					systemMSPVersionUpdate(MSP430MessageIn[MSP430_MSG_VERSION_IDX]);

					if((MSP430MessageIn[MSP430_MSG_VERSION_IDX] != 0) && (!systemMSP430CommsValid)) {
						systemMSP430CommsValid = TRUE;
					}
				} else {
					checksumFailure++;
					// Fail checksum
					if (showError) {cprintf("Bsum ");}
				}
			} else {
				// Fail checkcode
				if (showError) {cprintf("Bcod ");}
			}
			// Toggle MSP boards, regardless of result
			if (expand0En) {
				MSPSelect = MSP_EXPAND0_BOARD_SEL;
			}
		}

		// Increment index, wrap back to 0 if it exceeds the range
		MSP430MessageIdx++;
		if (MSP430MessageIdx >= MSP430_MSG_LENGTH) {
			MSP430MessageIdx = 0;
		}

		// Toggle states
		MSPSPIState = MSPSPI_STATE_DESELECT_SPI;
	} else if (MSPSPIState == MSPSPI_STATE_DESELECT_SPI) {
		// Toggle states
		MSPSPIState = MSPSPI_STATE_XMIT_BYTE;
	}

	return 0;
}


/*
 * @brief Expansion0 MSP430 board ISR
 *
 * @returns void
 */
uint8 expand0BoardISR() {
	uint32 data, i;
	uint32 timerLoadPeriod;
	uint8 checksum;
	long val;
	static uint8 gripperData = 0;

	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

	// Multiplex outputs (gripper/radio), set delay
	if (MSPSPIState == MSPSPI_STATE_XMIT_BYTE) {
		radioIntDisable();
		SPISelectDeviceISR(SPI_EXPAND0);
		timerLoadPeriod = MSP430_SPI_DESELECT_PERIOD;
	} else if (MSPSPIState == MSPSPI_STATE_DESELECT_SPI) {
		SPIDeselectISR();
		radioIntEnable();
		timerLoadPeriod = (MSP430_SPI_BYTE_PERIOD);
	}

	MAP_TimerLoadSet(TIMER1_BASE, TIMER_B, timerLoadPeriod);
	MAP_TimerEnable(TIMER1_BASE, TIMER_B);

	// Transmit a byte or just switch state
	if (MSPSPIState == MSPSPI_STATE_XMIT_BYTE) {
		// Pack message
		if (expand0MessageIdx == 0) {
			// Grab next message from buffer
			if (!expand0MessageOutBufLock) {
				memcpy(expand0MessageOut, expand0MessageOutBuf, EXPAND0_MSG_LENGTH);
			}
		}

		// Rotate received buffer
		shiftBuffer(expand0MessageIn, EXPAND0_MSG_LENGTH);

		// Put data into the transmit buffer
		val = MAP_SSIDataPutNonBlocking(SSI0_BASE, (uint32)expand0MessageOut[expand0MessageIdx]);

		// Retrieve the received byte
		MAP_SSIDataGet(SSI0_BASE, &data);

		// Put byte the the end of the received buffer
		expand0MessageIn[EXPAND0_MSG_LENGTH - 1] = data;

		//Integrity checking
		if (expand0MessageIdx == 0) {
			// Checkcode
			for (i = 0; i < EXPAND0_CODE_LENGTH; i++) {
				if (expand0MessageIn[i] != expand0Code[i])
					break;
			}
			if (i == EXPAND0_CODE_LENGTH) {
				// Checksum
				checksum = messageChecksum(expand0MessageIn, EXPAND0_CODE_LENGTH, EXPAND0_MSG_LENGTH);
				if (checksum == expand0MessageIn[EXPAND0_MSG_LENGTH - 1]) {
					// Avoid loading from buffer if the buffer is updating
					if (!expand0MessageInBufLock) {
						memcpy(expand0MessageInBuf, expand0MessageIn, EXPAND0_MSG_LENGTH);
					}
				} else {
					// Fail checksum
					if (showError) {cprintf("Gsum ");}
				}
			} else {
				// Fail checkcode
				if (showError) {cprintf("Gcod ");}
			}
			// Toggle MSP boards, regardless of result
			MSPSelect = MSP_BOTTOM_BOARD_SEL;
		}

		// Increment index, wrap back to 0 if it exceeds the range
		expand0MessageIdx++;
		if (expand0MessageIdx >= EXPAND0_MSG_LENGTH) {
			expand0MessageIdx = 0;
		}

		// Toggle states
		MSPSPIState = MSPSPI_STATE_DESELECT_SPI;
	} else if (MSPSPIState == MSPSPI_STATE_DESELECT_SPI) {
		// Toggle states
		MSPSPIState = MSPSPI_STATE_XMIT_BYTE;
	}

	return 0;
}

/*
 * @brief Interrupt handler for MSP430.
 *
 * This runs at different periods.  We have to disable the RADIO IRQ when we
 * have the select line low. We assume the thread-based SPI calls disable both of
 * these interrupts.
 * @returns void
*/
void msp430InterruptHandler(void){
	if (MSPSelect == MSP_BOTTOM_BOARD_SEL) {
		bottomBoardISR();
	} else if (MSPSelect == MSP_EXPAND0_BOARD_SEL) {
		expand0BoardISR();
	}
}

#endif //#if (defined(RONE_V9) || defined(RONE_V12))

// There is no MSP430 on the IR beacon board, so these functions should all be left empty.
#if defined (RONE_IRBEACON)

void msp430Init(void) {
}

void msp430InterruptHandler(void) {
}

void msp430InterruptEnable(void) {
}

void msp430InterruptDisable(void){
}

void msp430SystemCommandBuildMessage(uint8* msg){
}

void msp430SystemShutdownCommand(void){
}

uint8 msp430SystemGetCommandMessage(void){
	return 0;
}
#endif
