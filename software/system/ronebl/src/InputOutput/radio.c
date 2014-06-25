/*
 * @file radio.c
 * @brief turns WiFi radio on or off, sends and receives radio messages, radio interrupts
 * @since Jul 9, 2010
 * @authors sjb2, edited by lyncas
 */
#include <stdio.h>
#include <string.h>

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

#include "System/bootloader.h"

#define RADIO_IRQ_PORT 					GPIO_PORTC_BASE
#define RADIO_IRQ_SYSCTL 				SYSCTL_PERIPH_GPIOC
#define RADIO_IRQ_PIN 					GPIO_PIN_5
#define RADIO_INT 						INT_GPIOC
#define RADIO_CE_PIN            		GPIO_PIN_2
#define RADIO_CE_PORT           		GPIO_PORTB_BASE
#define RADIO_CE_SYSCTL         		SYSCTL_PERIPH_GPIOB
#define RADIO_XMIT_IRQ_TIMEOUT			10

#define NRF_RADIO_CONFIG_DEFAULT	((1 << NRF_CONFIG_PWR_UP) | (1 << NRF_CONFIG_EN_CRC) | (1 << NRF_CONFIG_CRCO))
#define NRF_STATUS_ALL				((1 << NRF_STATUS_RX_DR) | (1 << NRF_STATUS_TX_DS) | (1 << NRF_STATUS_MAX_RT))

uint8 bootloaderSubnet = RADIO_BOOTLOADER_DEFAULT_SUBNET;

// Radio chip read/write functions
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


static void radio_write_command_isr(uint32 command) {
	uint32 chip_status;

	SPISelectDeviceISR(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, command);
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	SPIDeselectISR();
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


static void radio_write_command(uint32 command) {
	uint32 chip_status;

	SPISelectDevice(SPI_RADIO);
	MAP_SSIDataPut(SSI0_BASE, command);
	MAP_SSIDataGet(SSI0_BASE, &chip_status);
	SPIDeselect();
}


static void radio_ce_on(void) {
	MAP_GPIOPinWrite(RADIO_CE_PORT, RADIO_CE_PIN, RADIO_CE_PIN);
}


static void radio_ce_off(void) {
	MAP_GPIOPinWrite(RADIO_CE_PORT, RADIO_CE_PIN, 0);
}


static void radio_set_tx_mode_isr(void) {
	radio_ce_off();
	// clear all interrupt flags
	radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);

	// power on the radio, primary tx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (0 << NRF_CONFIG_PRIM_RX));
	radio_ce_on();

	radio_write_command_isr(NRF_FLUSH_TX);
}


static void radio_set_tx_mode(void) {
	radio_ce_off();
	// clear all interrupt flags

	radio_write_register(NRF_STATUS, NRF_STATUS_ALL);

	// power on the radio, primary tx
	radio_write_register(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (0 << NRF_CONFIG_PRIM_RX));
	radio_ce_on();

	radio_write_command(NRF_FLUSH_TX);
}


static void radio_set_rx_mode_isr(void) {
	radio_ce_off();

	// power on the radio, primary rx
	radio_write_register_isr(NRF_CONFIG, NRF_RADIO_CONFIG_DEFAULT | (1 << NRF_CONFIG_PRIM_RX));
	radio_ce_on();

	radio_write_command_isr(NRF_FLUSH_RX);
}
// End Radio chip read/write functions

/*
 * @brief Get the subnet value from the radio message
 *
 * @returns subnet
 */
uint8 radioGetSubnet(RadioMessage* messagePtr) {
	return (messagePtr->command.type >> RADIO_COMMAND_SUBNET_SHIFTS);
}


/*
 * @brief Initializes the radio. (no interrupt)
 *
 * @returns void
 */
void radioInit(void) {
	// Enable all peripherals for the SPI Radio Control
	MAP_SysCtlPeripheralEnable(RADIO_IRQ_SYSCTL);
	MAP_SysCtlPeripheralEnable(RADIO_CE_SYSCTL);

	// Set up Radio IRQ line as input (active low)
	MAP_GPIOPinTypeGPIOInput(RADIO_IRQ_PORT, RADIO_IRQ_PIN);
	MAP_GPIOPadConfigSet(RADIO_IRQ_PORT, RADIO_IRQ_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	// Enable the CE pin
	radio_ce_off();
	MAP_GPIOPinTypeGPIOOutput(RADIO_CE_PORT, RADIO_CE_PIN);

	// Turn off all auto acknowledgements
	radio_write_register_isr(NRF_EN_AA, 0x00);

	// Set a fixed 32-byte message length
	radio_write_register_isr(NRF_RX_PW_P0, RADIO_MESSAGE_LENGTH_RAW);

	// Clear int flags
	radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);

	// Enable rx mode.
	radio_set_rx_mode_isr();

	// Flush all
	radio_write_command_isr(NRF_FLUSH_TX);
	radio_write_command_isr(NRF_FLUSH_RX);
}


/*
 * @brief Sends a message through the radio.
 *
 * @param messagePtr Pointer to the message to be sent. The message should be 32-bytes long
 *
 * @returns void
 */
void radioSendMessage(RadioMessage* messagePtr) {
	uint32 spi_data;
	uint8 i;

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

	// Wait until TX_DS or MAX_RT to pull the IRQ line, and change back to RX mode
	while (1) {
		if (!GPIOPinRead(RADIO_IRQ_PORT, RADIO_IRQ_PIN)) {
			uint32 status;

			status = radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);
			if (status & (1 << NRF_STATUS_TX_DS) || status & (1 << NRF_STATUS_MAX_RT)) {
				radio_set_rx_mode_isr();
				break;
			}
		}
	}
}


/*
 * @brief Checks for receive messages. The received message is copied into messagePtr and returns true.
 * if there is no message received, returns false.
 * @param messagePtr Pointer to the RadioMessage that stores the received message
 * @returns TRUE, if there is a new message; FALSE, if there is not.
 */
boolean radioRecvMessage(RadioMessage* messagePtr) {
	uint32 status, statusFIFO;
	uint32 chip_status;
	uint32 spi_data;
	uint8 i;
	boolean newMessage;

	newMessage = FALSE;

	// Even though we are not using interrupt, when new message is received the interrupt pin is still toggled (active low)
	if (!GPIOPinRead(RADIO_IRQ_PORT, RADIO_IRQ_PIN)) {
		// read the interrupt flags and clear the radio interrupt flag register.
		status = radio_write_register_isr(NRF_STATUS, NRF_STATUS_ALL);
		// Receive radio packet
		if (status & (1 << NRF_STATUS_RX_DR)) {
			// receive the message from the SPI bus
			SPISelectDeviceISR(SPI_RADIO);	// TODO need?
			MAP_SSIDataPut(SSI0_BASE, NRF_R_RX_PAYLOAD);
			MAP_SSIDataGet(SSI0_BASE, &chip_status);
			for (i = 0; i < RADIO_MESSAGE_LENGTH_RAW; i++) {
				MAP_SSIDataPut(SSI0_BASE, 0x00);
				MAP_SSIDataGet(SSI0_BASE, &spi_data);
				messagePtr->raw.data[i] = (char)spi_data;
			}
			SPIDeselectISR();
			radio_write_command_isr(NRF_FLUSH_RX);
			messagePtr->raw.linkQuality = radio_read_register_isr(NRF_RPD);
			messagePtr->raw.timeStamp = 0;
			if ((messagePtr->command.type & RADIO_COMMAND_TYPE_MASK) >=  RADIO_COMMAND_TYPE_REBOOT) {
				// We don't check for subnet because main program should already filter out wrong subnet robots (limbo)
				newMessage = TRUE;
			}
			// Only accept messages with the right subnet
//			if (radioGetSubnet(messagePtr) == RADIO_SUBNET_ALL) {
//				newMessage = TRUE;
//			}
//			else if (radioGetSubnet(messagePtr) == bootloaderSubnet) {
//				newMessage = TRUE;
//			}
		}
	}

	return newMessage;
}
