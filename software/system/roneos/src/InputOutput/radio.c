/*
 * @file radio.c
 * @brief turns WiFi radio on or off, sends and receives radio messages, radio interrupts
 * @since Jul 9, 2010
 * @authors sjb2, edited by lyncas
 */
#include <stdio.h>
#include <string.h>

#define MSI_DEBUG

#ifdef PART_LM3S9B92
	#include "inc/lm3s9b92.h"
#endif
#ifdef PART_LM3S8962
	#include "inc/lm3s8962.h"
#endif

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_qei.h"

#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"

#include "hal_nrf_reg.h"

#include "roneos.h"


static int radioRXerror = 0; //TODO look up the queue overflow in FreeRTOS
static int radioTXerror = 0; //TODO look up the queue overflow in FreeRTOS

/* V6 Pin Definitions - Radio  */
/*  SS=PA7, SLP_TR=PA6, RST=PG0 , IRQ=PC5  */

/* V11 Pin Definitions - Radio  */
/*  SS=PA7, SLP_TR=PA6, RST=PG0 , IRQ=PC5  */


#define RADIO_IRQ_PORT 					GPIO_PORTC_BASE
#define RADIO_IRQ_SYSCTL 				SYSCTL_PERIPH_GPIOC
#define RADIO_IRQ_PIN 					GPIO_PIN_5
#define RADIO_INT 						INT_GPIOC
#define RADIO_CE_PIN            		GPIO_PIN_2
#define RADIO_CE_PORT           		GPIO_PORTB_BASE
#define RADIO_CE_SYSCTL         		SYSCTL_PERIPH_GPIOB
#define RADIO_XMIT_IRQ_TIMEOUT			10
#define RADIO_SPI_WRITE_STATUS_DELAY	20

static osQueueHandle radioCommsQueueRecv;
static osQueueHandle radioCommsQueueXmit;

uint32 radioCounterXmitQueue = 0;
uint32 radioCounterIRQStartXmit = 0;
uint32 radioCounterXmit = 0;
uint32 radioCounterReadStatusCorrect = 0;
uint32 radioCounterStopXmit = 0;
uint32 radioCounterMAXRT = 0;

uint32 radioCounterXmitLoop = 0;
uint32 radioCounterReadStatusError = 0;
uint32 radioCounterReadStatusTimeout = 0;

uint32 radioCounterReceiveStatusSuccess = 0;
uint32 radioCounterReceiveStatusError = 0;
uint32 radioCounterReceive = 0;

uint32 radioWatchdogCounter = 0;

void radioPrintCounters(void) {
	cprintf("Radio: xmit q/sta/x/stp/stat=%d/%d/%d/%d/%d/%d receive=%d s/e=%d/%d wd=%d\n",
			radioCounterXmitQueue,
			radioCounterIRQStartXmit,
			radioCounterXmit,
			radioCounterStopXmit,
			radioCounterReadStatusCorrect,
			radioCounterMAXRT,

			radioCounterReceive,
			radioCounterReceiveStatusSuccess,
			radioCounterReceiveStatusError,

			radioWatchdogCounter);
}

static uint32 radio_read_register_isr(uint32 reg) {
	uint32 chip_status;
	uint32 spi_data;

	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, reg);
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	MAP_SSIDataPut(SSI0_BASE, 0x00);
	MAP_SSIDataGet(SSI0_BASE, &spi_data);
	SPIDeselectISR();

	return spi_data;
}


static uint32 radio_write_register_isr(uint32 reg, uint32 val) {
	uint32 chip_status;
	uint32 spi_data;

	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, NRF_W_REGISTER | (reg & 0x3F));
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	MAP_SSIDataPut(SSI0_BASE, val);
	MAP_SSIDataGet(SSI0_BASE, &spi_data);
	SPIDeselectISR();

	return chip_status;
}


static uint32 radio_write_command_isr(uint32 command) {
	uint32 chip_status;
	uint32 error;

	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, command);
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	SPIDeselectISR();

	return chip_status;
}


static uint32 radio_write_command_isr_nb(uint32 command, uint32 delay, uint32* successPtr) {
	uint32 chip_status;

	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, command);
	MAP_SysCtlDelay(delay);
	*successPtr = MAP_SSIDataGetNonBlocking(SSI0_BASE, &chip_status);
	SPIDeselectISR();

	return chip_status;
}


static uint32 radio_read_register(uint32 reg) {
	uint32 chip_status;
	uint32 spi_data;

	SPISelectDevice(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, reg);
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	MAP_SSIDataPut(SSI0_BASE, 0x00);
	MAP_SSIDataGet(SSI0_BASE, &spi_data);
	SPIDeselect();

	return spi_data;
}


static uint32 radio_write_register(uint32 reg, uint32 val) {
	uint32 chip_status;
	uint32 spi_data;

	SPISelectDevice(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, NRF_W_REGISTER | (reg & 0x3F));
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	MAP_SSIDataPut(SSI0_BASE, val);
	MAP_SSIDataGet(SSI0_BASE, &spi_data);
	SPIDeselect();

	return chip_status;
}


static uint32 radio_write_command(uint32 command) {
	uint32 chip_status;

	SPISelectDevice(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, command);
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	SPIDeselect();

	return chip_status;
}


static void radio_ce_on(void) {
	MAP_GPIOPinWrite(RADIO_CE_PORT, RADIO_CE_PIN, RADIO_CE_PIN);
}


static void radio_ce_off(void) {
	MAP_GPIOPinWrite(RADIO_CE_PORT, RADIO_CE_PIN, 0);
}


//#define NRF_RADIO_CONFIG_DEFAULT	((1 << NRF_CONFIG_MASK_TX_DS) | (1 << NRF_CONFIG_MASK_MAX_RT) | (1 << NRF_CONFIG_PWR_UP) | (1 << NRF_CONFIG_EN_CRC) | (1 << NRF_CONFIG_CRCO))
#define NRF_RADIO_CONFIG_POWER_DOWN	((0 << NRF_CONFIG_PWR_UP) | (1 << NRF_CONFIG_EN_CRC) | (1 << NRF_CONFIG_CRCO))
#define NRF_RADIO_CONFIG_DEFAULT	((1 << NRF_CONFIG_PWR_UP) | (1 << NRF_CONFIG_EN_CRC) | (1 << NRF_CONFIG_CRCO))
#define NRF_STATUS_ALL				((1 << NRF_STATUS_RX_DR) | (1 << NRF_STATUS_TX_DS) | (1 << NRF_STATUS_MAX_RT))

static void radio_set_tx_mode_isr(void) {
	// disable the CE line
	radio_ce_off();

	// clear all interrupt flags
	radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);

	// power on the radio, primary tx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (0 << NRF_CONFIG_PRIM_RX));
	radio_ce_on();

	//TODO should this happen before CE goes high?
	radio_write_command_isr(NRF_FLUSH_TX);
}


static void radio_set_tx_mode(void) {
	radio_ce_off();
	// clear all interrupt flags
	radio_write_register(NRF_STATUS, NRF_STATUS_ALL);

	// power on the radio, primary tx
	radio_write_register(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (0 << NRF_CONFIG_PRIM_RX));
	systemDelayUSec(130);
	radio_ce_on();

	//TODO should this happen before CE goes high?
	radio_write_command(NRF_FLUSH_TX);
}


static void radio_set_rx_mode_isr(void) {
	radio_ce_off();

	radio_write_command_isr(NRF_FLUSH_RX);

	// power on the radio, primary rx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (1 << NRF_CONFIG_PRIM_RX));
	//systemDelayUSec(130);
	radio_ce_on();
}


/*
 * @brief Enables radio interrupt.
 *
 * @returns void
 */
void radioIntEnable(void) {
	MAP_GPIOPinIntEnable(RADIO_IRQ_PORT, RADIO_IRQ_PIN);
	//MAP_IntEnable(RADIO_INT);
}


/*
 * @brief Disables radio interrupt.
 *
 * @returns void
 */
void radioIntDisable(void) {
	//MAP_IntDisable(RADIO_INT);
	MAP_GPIOPinIntDisable(RADIO_IRQ_PORT, RADIO_IRQ_PIN);
}


boolean radioStartXmit = FALSE;

/*
 * @brief Handles radio interrupt.
 *
 * @returns void
 */
void radioIntHandler(void) {
	uint32 status, statusFIFO, chip_status, spi_data, i;
	uint32 statusWrite = 0;
	uint8 pipe, subnet, programTime;
	portBASE_TYPE val;
	portBASE_TYPE taskWoken = pdFALSE;
	RadioMessage message;

	// blink the button for timing measurements
	//MAP_GPIOPinWrite(BUTTON_RED_BASE, BUTTON_RED_PIN, BUTTON_RED_PIN);

	// the radio irq is disabled when any other SPI device is selected, so
	// we don't need to worry about being mutex on the SPI bus (can't get mutex from an interrupt anyway)
	// clear the 8962 interrupt pin flag
	MAP_GPIOPinIntClear(RADIO_IRQ_PORT, RADIO_IRQ_PIN);

	// read the radio interrupt flags
	status = radio_write_command_isr(NRF_NOP);
	statusWrite = status & ((1 << NRF_STATUS_TX_DS) | (1 << NRF_STATUS_MAX_RT) | (1 << NRF_STATUS_RX_DR));

	if ((status & (1 << NRF_STATUS_TX_DS)) || radioStartXmit) {
		// new packet to transmit or transmit finished.
		// check the xmit queue for a packet.  if there is data in the xmit queue, then send it out
		if(radioStartXmit) {
			// If this interrupt was manually triggered, reset the pin back to low-level
			MAP_GPIOIntTypeSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_LOW_LEVEL);

			// Flush the TX FIFO
			//radio_write_command_isr(NRF_FLUSH_TX);
		}

		//todo comment out all xmit
#if 1
		val = osQueueReceiveFromISR(radioCommsQueueXmit, &message, &taskWoken);
		if (val == pdPASS) {
			// disable the CE line
			radio_ce_off();

			if(radioStartXmit) {
				// new packet to xmit.  switch the radio to xmit mode
				// power on the radio, primary tx
				//systemDelayUSec(130);
				radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (0 << NRF_CONFIG_PRIM_RX));
				//systemDelayUSec(130);

				radioStartXmit = FALSE;
				radioCounterIRQStartXmit++;
			}

			SPISelectDeviceISR(SPI_RADIO);
			//MAP_SSIDataPut(SSI0_BASE, NRF_W_TX_PAYLOAD_NOACK);
			MAP_SSIDataPut(SSI0_BASE, NRF_W_TX_PAYLOAD);
			MAP_SSIDataGet(SSI0_BASE, &chip_status);
			for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; ++i) {
				MAP_SSIDataPut(SSI0_BASE, message.raw.data[i]);
				MAP_SSIDataGet(SSI0_BASE, &spi_data);
			}
			SPIDeselectISR();

			// give a > 10us pulse on CE to send the packet
			radio_ce_on();
			systemDelayUSec(15);
			radio_ce_off();
			radioCounterXmit++;
		} else {
			// nothing else to xmit.  return to rx mode
			radio_set_rx_mode_isr();
			radioCounterStopXmit++;
		}
		if (status & (1 << NRF_STATUS_TX_DS)) {
			// clear the TX_DS  interrupt flags
			radioCounterReadStatusCorrect++;
		}
#endif

	}

	if (status & (1 << NRF_STATUS_MAX_RT)) {
		// We have reached the maximum number of retransmissions. we should never get here,
		// because we don't use the rexmit feature. But, clear the status bit and put the radio in
		// receive mode anyway

		radio_write_command_isr(NRF_FLUSH_RX);
		radio_write_command_isr(NRF_FLUSH_TX);

		// turn off all auto acknowledgements
		radio_write_register_isr(NRF_EN_AA, 0x00);

		 // turn off auto rexmit
		radio_write_register_isr(NRF_SETUP_RETR, 0x00);

		 // enable only data pipe 0
		radio_write_register_isr(NRF_EN_RXADDR, 0x01);

		// set a fixed 32-byte message length
		radio_write_register_isr(NRF_RX_PW_P0, RADIO_MESSAGE_LENGTH_RAW);

		// enable the noack payload command
		radio_write_register_isr(NRF_FEATURE, (1 << NRF_FEATURE_EN_DYN_ACK));

		//radio_set_rx_mode_isr();
		radioCounterMAXRT++;
	}

	if (status & (1 << NRF_STATUS_RX_DR)) {
		boolean moreMessages = FALSE;
		do {
			// a received message has arrived.  transfer the message from the radio over the SPI bus
			SPISelectDeviceISR(SPI_RADIO);
			MAP_SSIDataPut(SSI0_BASE, NRF_R_RX_PAYLOAD);
			MAP_SSIDataGet(SSI0_BASE, &chip_status);
			for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; i++) {
				MAP_SSIDataPut(SSI0_BASE, 0x00);
				MAP_SSIDataGet(SSI0_BASE, &spi_data);
				message.raw.data[i] = (char)spi_data;
			}
			SPIDeselectISR();
			//TODO the RPD seems mostly useless.  Maybe can test later...
			//message.raw.linkQuality = radio_read_register_isr(NRF_RPD);
			message.raw.linkQuality = 0;
			message.raw.timeStamp = osTaskGetTickCountFromISR();

			radioCounterReceive++;

	#if 0
			// Receiving bootloader messages host reprogramming message
	// 		if ((message.command.type & RADIO_COMMAND_TYPE_MASK) >  RADIO_COMMAND_TYPE_REBOOT) {
	// 			// Select to right version
	// #if defined(RONE_V9)
	// 			programTime = RADIO_COMMAND_TYPE_PROGRAM_TIME_V11;
	// #elif defined(RONE_V12)
	// 			programTime = RADIO_COMMAND_TYPE_PROGRAM_TIME_V12;
	// #endif
	// 			subnet = radioCommandGetSubnet(&message);

	// //			cprintf("SUBNET = %d\n", subnet);
	// //			cprintf("programTime = %d \n", (int)(message.command.type & RADIO_COMMAND_TYPE_MASK));
	// //			cprintf("range = [%d, %d] \n",  (int)(message.raw.data[1]), (int)(message.raw.data[2]));
	// //			SysCtlDelay(500000);

	// 			// Only put robot in receive mode if: correct subnet, right hardware version, and ID range
	// 			// RADIO_COMMAND_TYPE_PROGRAM_TIME: message_header[4] = [type + subnet, id_range_min, id_range_max, sender ID]
	// 			if (((subnet == RADIO_SUBNET_ALL) || (subnet == radioCommandGetLocalSubnet())) &&
	// 					(programTime == (uint8)(message.command.type & RADIO_COMMAND_TYPE_MASK)) &&
	// 					(roneID >= (uint8)(message.raw.data[1]) && roneID <= (uint8)(message.raw.data[2]))) {
	// 				writeBootloaderState(BL_STATE_RECEIVE);
	// 			} else {
	// 				// If the robot receives non-'Program Time' bootloader commands, traps it
	// 				if ((message.command.type & RADIO_COMMAND_TYPE_MASK) < RADIO_COMMAND_TYPE_PROGRAM_TIME_V14) {
	// 					while (1) {
	// 						blinkyLedSet(1);
	// 						SysCtlDelay(5000000);
	// 						blinkyLedSet(0);
	// 						SysCtlDelay(5000000);
	// 					}
	// 				}
	// 				// Put robot in limbo state if the host program is not meant for this robot (wrong subnet or version)
	// 				writeBootloaderState(BL_STATE_LIMBO);
	// 			}
	// 			// Load bootloader
	// 			bootloading();
	// 		}
	#endif

			// put the received message on the main radio receive queue
			val = osQueueSendFromISR(radioCommsQueueRecv, (void*)(&message), &taskWoken);
			uint32 FIFOStatus = radio_read_register_isr(NRF_FIFO_STATUS);
			moreMessages = !(FIFOStatus & (1 << NRF_FIFOSTATUS_RX_EMPTY));
			if (moreMessages) {
				FIFOStatus++;
				FIFOStatus--;
			}
		} while (moreMessages);
	}

	radio_write_register_isr(NRF_STATUS, statusWrite);

//	MAP_GPIOPinWrite(BUTTON_RED_BASE, BUTTON_RED_PIN, 0);

	portEND_SWITCHING_ISR(taskWoken);
}


/*
 * @brief Initializes the radio.
 *
 * @returns void
 */
void radioInit(void) {
	radioCommsQueueRecv = osQueueCreate(RADIO_COMMS_QUEUE_RECV_SIZE, sizeof(RadioMessage));
	radioCommsQueueXmit = osQueueCreate(RADIO_COMMS_QUEUE_XMIT_SIZE, sizeof(RadioMessage));

	//Enable all peripherals for the SPI Radio Control
	MAP_SysCtlPeripheralEnable(RADIO_IRQ_SYSCTL);
	MAP_SysCtlPeripheralEnable(RADIO_CE_SYSCTL);

	//Set up interrupt on the Radio IRQ line
	MAP_GPIOPinTypeGPIOInput(RADIO_IRQ_PORT, RADIO_IRQ_PIN);
	MAP_GPIOPadConfigSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	//TODO testing level-sensitive interrupt
	//MAP_GPIOIntTypeSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_FALLING_EDGE);
	MAP_GPIOIntTypeSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_LOW_LEVEL);
	MAP_GPIOPinIntEnable(RADIO_IRQ_PORT, RADIO_IRQ_PIN);

	// we assume this is being called before threads are running, so there is no need to get the SPI mutex.
	//Configure the CE pin
	radio_ce_off();
	MAP_GPIOPinTypeGPIOOutput(RADIO_CE_PORT, RADIO_CE_PIN);

	 // turn off all auto acknowledgements
	radio_write_register_isr(NRF_EN_AA, 0x00);

	 // turn off auto rexmit
	radio_write_register_isr(NRF_SETUP_RETR, 0x00);

	 // enable only data pipe 0
	radio_write_register_isr(NRF_EN_RXADDR, 0x01);

	// set a fixed 32-byte message length
	radio_write_register_isr(NRF_RX_PW_P0, RADIO_MESSAGE_LENGTH_RAW);

	// enable the noack payload command
	radio_write_register_isr(NRF_FEATURE, (1 << NRF_FEATURE_EN_DYN_ACK));

	// clear int flags
	radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);

	// enable rx mode.  we use the isr version because radio interrupts are still disabled
	radio_set_rx_mode_isr();
	radioIntEnable();

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
	MAP_IntPrioritySet(RADIO_INT, SYSTEM_INTERRUPT_PRIORITY);
	MAP_IntEnable(RADIO_INT);
}

#define RADIO_WATCHDOG_RECEIVE_TIME	2000

void radioWatchdog(void) {
	static uint32 lastReceiveTime = 0;
	static uint32 radioCounterReceiveOld = 0;

	if (radioCounterReceiveOld != radioCounterReceive) {
		radioCounterReceiveOld = radioCounterReceive;
		lastReceiveTime = osTaskGetTickCount();
	}
	if ((osTaskGetTickCount() - lastReceiveTime) > RADIO_WATCHDOG_RECEIVE_TIME) {
		// you have received no messages for a while.  radio bug.  reset the radio

 		radio_ce_off();

//		radio_write_register(NRF_CONFIG, NRF_RADIO_CONFIG_POWER_DOWN | (1 << NRF_CONFIG_PRIM_RX));
//		systemDelayUSec(200);
//
//		radio_write_register(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (1 << NRF_CONFIG_PRIM_RX));
//		systemDelayUSec(200);

		 // turn off all auto acknowledgements
		radio_write_register(NRF_EN_AA, 0x00);

		 // turn off auto rexmit
		radio_write_register(NRF_SETUP_RETR, 0x00);

		 // enable only data pipe 0
		radio_write_register(NRF_EN_RXADDR, 0x01);

		// set a fixed 32-byte message length
		radio_write_register(NRF_RX_PW_P0, RADIO_MESSAGE_LENGTH_RAW);

		// enable the noack payload command
		radio_write_register(NRF_FEATURE, (1 << NRF_FEATURE_EN_DYN_ACK));

		// clear int flags
		radio_write_register(NRF_STATUS, NRF_STATUS_ALL);

		lastReceiveTime = osTaskGetTickCount();
		radioWatchdogCounter++;
	}
}


/*
 * @brief Sends a message through the radio.
 *
 * @param messagePtr pointer to the message to be sent
 *
 * Can't call this function from within an ISR
 */
void radioSendMessage(RadioMessage* messagePtr) {
	uint32 spi_data;
	uint8 i;
	portBASE_TYPE val;

	// queue the message in the xmit queue
	val = osQueueSend(radioCommsQueueXmit, messagePtr, 0);
	if(val == pdPASS) {
		radioCounterXmitQueue++;
		//trigger a radio interrupt to talk to the SPI device
		radioStartXmit = TRUE;
		MAP_GPIOIntTypeSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_HIGH_LEVEL);
	}
}


/*
 * \internal
 * @brief Checks whether there is a message. Does not block.
 *
 * @param message pointer to the message to be checked
 * @param message_size_ptr pointer to the size of the message
 * @param message_link_quality_ptr pointer to the link quality of the message
 * @returns TRUE if there is a message, FALSE if not
 * \endinternal
 */
static boolean radioGetMessage_internal(RadioMessage* messagePtr) {
	portBASE_TYPE val;

	val = osQueueReceive(radioCommsQueueRecv, (void *)(messagePtr), 0);
	if (val == pdPASS) {
		return TRUE;
	} else {
		return FALSE;
	}
}


/*
 * @brief Checks whether there is a message with blocking
 *
 * @param messagePtr pointer to the message to be checked
 * @returns void
 */
void radioGetMessageBlocking(RadioMessage* messagePtr) {
	portBASE_TYPE val;

	//TODO do something with this error
	val = osQueueReceive(radioCommsQueueRecv, (void *)(messagePtr), portMAX_DELAY);
}


/*
 * @brief Checks whether there is a message.  Does not block. Will return FALSE
 *        if remote control mode is enabled. Should not be called directly by
 *        user. Use the radioCommand.c method instead.
 *
 * @param messagePtr pointer to the message to be checked
 * @returns TRUE if there is a message, FALSE if not
 */
static boolean radioGetMessage(RadioMessage* messagePtr) {
/*	portBASE_TYPE val;

	val = osQueueReceive(radioCommsQueueRecv, (void *)(messagePtr), 0);
	if (val == pdPASS) {
		return TRUE;
	} else {
		return FALSE;
	}
	*/
	if (rcMode == RC_MODE_ON) {
		return (FALSE);
	} else {
		return radioGetMessage_internal(messagePtr);
	}
}
