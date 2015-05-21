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

uint32 radioCounterXmit = 0;
uint32 radioCounterXmitLoop = 0;
uint32 radioCounterReadStatusCorrect = 0;
uint32 radioCounterReadStatusError = 0;
uint32 radioCounterXmitQueue = 0;
uint32 radioCounterIRQQueueXmit = 0;
uint32 radioCounterIRQQueueEmpty = 0;

uint32 radioCounterReceiveStatusSuccess = 0;
uint32 radioCounterReceiveStatusError = 0;
uint32 radioCounterReceive = 0;

void radioPrintCounters(void) {
	//cprintf("Radio: xmitMsg %d xmitQueueMsg=%d IRQXmitMsgFromQueue=%d IRQQueueEmpty=%d receive=%d\n",
	//cprintf("Radio: xmit=%d,%d xmitQ=%d IRQXmitQ=%d IRQQEmpty=%d receive=%d\n",radioCounterXmit, radioCounterXmitLoop, radioCounterXmitQueue, radioCounterIRQQueueXmit, radioCounterIRQQueueEmpty, radioCounterReceive);
	//cprintf("Radio: xmit=%d,%d stat=%d/%d receive=%d\n", radioCounterXmit, radioCounterXmitLoop, radioCounterReadStatusCorrect, radioCounterReadStatusError, radioCounterReceive);
	cprintf("Radio: xmit=%d,%d stat=%d/%d receive=%d stat=%d/%d\n", radioCounterXmit, radioCounterXmitLoop, radioCounterReadStatusCorrect, radioCounterReadStatusError,
			radioCounterReceive, radioCounterReceiveStatusSuccess, radioCounterReceiveStatusError);
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
	MAP_SSIDataPutNonBlocking(SSI0_BASE, command);
	MAP_SSIDataGetNonBlocking(SSI0_BASE, &chip_status);
	SPIDeselectISR();

	return chip_status;
}


static uint32 radio_write_command_isr_nb(uint32 command, uint32 delay, uint32* successPtr) {
	uint32 chip_status;

	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPutNonBlocking(SSI0_BASE, command);
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


#define NRF_RADIO_CONFIG_DEFAULT	((1 << NRF_CONFIG_PWR_UP) | (1 << NRF_CONFIG_EN_CRC) | (1 << NRF_CONFIG_CRCO))
#define NRF_STATUS_RECV				(1 << NRF_STATUS_RX_DR)
#define NRF_STATUS_XMIT				((1 << NRF_STATUS_TX_DS) | (1 << NRF_STATUS_MAX_RT))
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
	radio_ce_on();

	//TODO should this happen before CE goes high?
	radio_write_command(NRF_FLUSH_TX);
}


static void radio_set_rx_mode_isr(void) {
	radio_ce_off();

	// power on the radio, primary rx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (1 << NRF_CONFIG_PRIM_RX));
	radio_ce_on();

	//TODO should this happen before CE goes high?
	radio_write_command_isr(NRF_FLUSH_RX);
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


//#define BUTTON_RED_PERIPH 			SYSCTL_PERIPH_GPIOF
//#define BUTTON_RED_BASE 			GPIO_PORTF_BASE
//#define BUTTON_RED_PIN 				GPIO_PIN_1

/*
 * @brief Handles radio interrupt.
 *
 * @returns void
 */
void radioIntHandler(void) {
	uint32 status, statusFIFO, chip_status, spi_data, i;
	uint8 pipe, subnet, programTime;
	portBASE_TYPE val;
	portBASE_TYPE taskWoken = pdFALSE;
	RadioMessage message;

//	MAP_GPIOPinWrite(BUTTON_RED_BASE, BUTTON_RED_PIN, BUTTON_RED_PIN);

	// the radio irq is disabled when any other SPI device is selected, so
	// we don't need to get the SPI mutex here (can't get mutex from an interrupt anyway)

	// clear the 8962 interrupt pin
	MAP_GPIOPinIntClear(RADIO_IRQ_PORT, RADIO_IRQ_PIN);

	// read the interrupt flags and clear the radio interrupt flag register.
	//status = radio_write_register_isr(NRF_STATUS, NRF_STATUS_RECV);
	//status = radio_write_command_isr(NRF_NOP);

	uint32 success;
	status = radio_write_command_isr_nb(NRF_NOP, RADIO_SPI_WRITE_STATUS_DELAY, &success);
	if(success) {
		radioCounterReceiveStatusSuccess++;
	} else {
		radioCounterReceiveStatusError++;
		status = 0;
	}


#ifndef MSI_DEBUG
	if (status & (1 << NRF_STATUS_TX_DS)) {
		// transmit finished. if there is data in the xmit queue, then send it out
		val = osQueueReceiveFromISR(radioCommsQueueXmit, &message, &taskWoken);
		if (val == pdPASS) {
			SPISelectDeviceISR(SPI_RADIO);
			MAP_SSIDataPut(SSI0_BASE, NRF_W_TX_PAYLOAD_NOACK);
			MAP_SSIDataGet(SSI0_BASE, &chip_status);
			for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; ++i) {
				MAP_SSIDataPut(SSI0_BASE, message.raw.data[i]);
				MAP_SSIDataGet(SSI0_BASE, &spi_data);
			}
			SPIDeselectISR();
			radioCounterIRQQueueXmit++;
		} else {
			// nothing else to xmit.  return to rx mode
			radio_set_rx_mode_isr();
			radio_xmit_irq_complete = TRUE;
			radioCounterIRQQueueEmpty++;
		}
	}
	if (status & (1 << NRF_STATUS_MAX_RT)) {
		//reach maximum retransmissions.
		//Give up. Can retry by toggling CE.
		radio_set_rx_mode_isr();
	}
#endif

	if (status & NRF_STATUS_RECV) {
		// receive the message form the SPI bus
		SPISelectDeviceISR(SPI_RADIO);
		MAP_SSIDataPut(SSI0_BASE, NRF_R_RX_PAYLOAD);
		MAP_SSIDataGet(SSI0_BASE, &chip_status);
		for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; i++) {
			MAP_SSIDataPut(SSI0_BASE, 0x00);
			MAP_SSIDataGet(SSI0_BASE, &spi_data);
			message.raw.data[i] = (char)spi_data;
		}
		SPIDeselectISR();
		//TODO the RPD seems mostly useless.  Maybe can test later today
		//message.raw.linkQuality = radio_read_register_isr(NRF_RPD);
		message.raw.linkQuality = 0;
		message.raw.timeStamp = osTaskGetTickCountFromISR();

		//TODO in the future, check to see if fifo is empty.  if not, get another packet
		//TODO for now, just flush the fifo and exit
		radio_write_command_isr(NRF_FLUSH_RX);

		radioCounterReceive++;

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

		// put the received message on the main radio receive queue
		//TODO - save this value of val , put into a global variable and look for radio recieve errors,
		val = osQueueSendFromISR(radioCommsQueueRecv, (void*)(&message), &taskWoken);
	}

	// clear all the interrupt bits
	radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);

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
	MAP_GPIOIntTypeSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_FALLING_EDGE);
	MAP_GPIOPinIntEnable(RADIO_IRQ_PORT, RADIO_IRQ_PIN);

	// we assume this is being called before threads are running, so there is no need to get the SPI mutex.
	//Configure the CE pin
	radio_ce_off();
	MAP_GPIOPinTypeGPIOOutput(RADIO_CE_PORT, RADIO_CE_PIN);

	 // turn off all auto acknowledgements
	radio_write_register_isr(NRF_EN_AA, 0x00);

	// set a fixed 32-byte message length
	radio_write_register_isr(NRF_RX_PW_P0, RADIO_MESSAGE_LENGTH_RAW);

	// clear int flags
	radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);

	// enable rx mode.  we use the isr version because radio interrupts are still disabled
	radio_set_rx_mode_isr();
	radioIntEnable();

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
	MAP_IntPrioritySet(RADIO_INT, SYSTEM_INTERRUPT_PRIORITY);
	MAP_IntEnable(RADIO_INT);
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

//	//busy wait for tx finished interrupt
//	for (i = 0; i < RADIO_XMIT_IRQ_TIMEOUT; i++) {
//		if (radio_xmit_irq_complete == TRUE) {
//			break;
//		}
//	}
//	i = i + 1;
//	radio_xmit_irq_complete = FALSE;
//
//	radio_set_tx_mode();
//
//	SPISelectDevice(SPI_RADIO);
//	MAP_SSIDataPut(SSI0_BASE, NRF_W_TX_PAYLOAD_NOACK);
//	MAP_SSIDataGet(SSI0_BASE, &chip_status);
//	while (size > 0) {
//		if (actualBytes > 0) {
//			MAP_SSIDataPut(SSI0_BASE, *char_ptr++);
//			actualBytes--;
//		} else {
//			MAP_SSIDataPut(SSI0_BASE, 0);
//		}
//		MAP_SSIDataGet(SSI0_BASE, &spi_data);
//		size--;
//	}
//	SPIDeselect();

#ifndef MSI_DEBUG
	radioIntDisable();
	if (radio_xmit_irq_complete == TRUE) {
		// the previous xmit has finished.  start a new xmit
		radio_xmit_irq_complete = FALSE;
		// switch the radio from receive to transmit mode
		radio_set_tx_mode();

		// select the SPI device and send the 32-byte message
		SPISelectDevice(SPI_RADIO);
		MAP_SSIDataPut(SSI0_BASE, NRF_W_TX_PAYLOAD_NOACK);
		MAP_SSIDataGet(SSI0_BASE, &spi_data);
		for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; ++i) {
			MAP_SSIDataPut(SSI0_BASE, messagePtr->raw.data[i]);
			MAP_SSIDataGet(SSI0_BASE, &spi_data);
		}
		SPIDeselect();
		radioCounterXmit++;
	} else {
		// buffer the message in case we have a xmit in progress.
		radioCounterXmitQueue++;
		osQueueSend(radioCommsQueueXmit, messagePtr, 1);
	}
	radioIntEnable();
#else
	// make the xmit atomic to other SPI interrupts
	// Select the radio, disable the ISRs, grab the mutex
	SPISelectDevice(SPI_RADIO);

	// switch the radio from receive to transmit mode
	// disable the CE line
	radio_ce_off();

	// clear all interrupt flags
	//radio_write_register_isr(NRF_STATUS, NRF_STATUS_XMIT);

	// power on the radio, primary tx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (0 << NRF_CONFIG_PRIM_RX));

	//TODO should this happen before CE goes high?
	//radio_write_command_isr(NRF_FLUSH_TX);

	//TODO check to see if the TX fifo is full

	// select the SPI device and send the 32-byte message
	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, NRF_W_TX_PAYLOAD_NOACK);
	MAP_SSIDataGet(SSI0_BASE, &spi_data);
	for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; ++i) {
		MAP_SSIDataPut(SSI0_BASE, messagePtr->raw.data[i]);
		MAP_SSIDataGet(SSI0_BASE, &spi_data);
	}
	SPIDeselectISR();

	// give a 15us pulse on ce to send the packet
	radio_ce_on();
	systemDelayUSec(15);
	radio_ce_off();
	//systemDelayUSec(50);

//	// Wait until TX_DS or MAX_RT to pull the IRQ line, and change back to RX mode
//	while (1) {
//		//if (!GPIOPinRead(RADIO_IRQ_PORT, RADIO_IRQ_PIN)) {
//			uint32 status;
//
//			status = radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);
//			if ((status & (1 << NRF_STATUS_TX_DS)) || (status & (1 << NRF_STATUS_MAX_RT))) {
//				radio_set_rx_mode_isr();
//				break;
//			}
//		//}
//	}

	// Wait until TX_DS or MAX_RT status bit is set, then change back to RX mode
	uint32 status;
	while (1) {
		//if (!GPIOPinRead(RADIO_IRQ_PORT, RADIO_IRQ_PIN)) {

		//status = radio_write_register_isr(NRF_STATUS, NRF_STATUS_XMIT);
		uint32 success;
		status = radio_write_command_isr_nb(NRF_NOP, RADIO_SPI_WRITE_STATUS_DELAY, &success);
		radioCounterXmitLoop++;
		if(success) {
			radioCounterReadStatusCorrect++;
			if (status & NRF_STATUS_XMIT) {
				radio_write_register_isr(NRF_STATUS, NRF_STATUS_XMIT);
				break;
			}
		} else {
			radioCounterReadStatusError++;
			break;
		}
		//}
	}

	//radio_set_rx_mode_isr();
	// power on the radio, primary rx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (1 << NRF_CONFIG_PRIM_RX));

	//TODO need delay here before we reasssert ce?
	radio_ce_on();

	//TODO should this happen before CE goes high?
	//radio_write_command_isr(NRF_FLUSH_RX);

	radioCounterXmit++;
	SPIDeselect();

#endif
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
