/*
 * @file spi.c
 * @brief SPI interface with MSP 430
 * @since Apr 2, 2012
 * @author James McLurkin
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

#include "System/bootloader.h"

uint8 SPIWordSize;
uint8 SPIMode;
uint32 SPIFrequency;

/*
 * @brief Initializes SPI unit.
 *
 * Enables peripherals
 * Configures GPIO
 * Sets radio, accelerometer, and LED as output
 * Sets word size as 0
 * @returns void
 */
void SPIInit(void) {
	volatile unsigned long ulLoop;

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// configure SSI pins for peripheral control
	MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN));
	MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	MAP_GPIOPinConfigure(GPIO_PA4_SSI0RX);
	MAP_GPIOPinConfigure(GPIO_PA5_SSI0TX);

	// Also, it may be better to just get rid of the pullups/downs entirely. -Jeremy
	MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN), GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

#if (defined(RONE_V12) || defined(RONE_V9))
    // enable output enable peripheral for SPI
	MAP_SysCtlPeripheralEnable(SPI_ENABLE_PERIPH);
	// enable peripheral for three SPI pins
	MAP_SysCtlPeripheralEnable(SPI_SELECT_PERIPH);

	// drive select pins to correct values while they are still in input mode
    SPIDeselectISR();

    // enable output enable pin for output
    MAP_GPIOPinTypeGPIOOutput(SPI_ENABLE_PORT, SPI_ENABLE_PIN);
    // enable A,B,C SPI pins for output
	MAP_GPIOPinTypeGPIOOutput(SPI_SELECT_PORT, SPI_SELECT_PINS);

#endif
	// force the port to be enabled by the first call to SPIConfigure()
	SPIWordSize = 0;
}


/*
 *	@brief Sets the SPI word size and data format
 *
 *	@param ulBase is the SSI port
 *	@param wordSize is the new word size to be set
 *	@param mode is the new transmission mode
 *	@param frequency is the frequency that the SPI clock will run at
 *	@returns void
 */
// note must be called after you have the mutex - after
// radio max clock freq = 10mhz
// MSP430 max fre clkI cycles - we might be violating this
// SD card -fast frequency ~= 2 MHz
// sound chip - 2s to be 400kHz at first, then bump up speed
void SPIConfigure(uint32 ulBase, uint8 wordSize, uint8 mode, uint32 frequency) {
	if ((wordSize != SPIWordSize) || (mode != SPIMode) || (frequency != SPIFrequency)) {
		// cache word size value
		SPIWordSize = wordSize;
		SPIMode = mode;
		SPIFrequency = frequency;

		MAP_SSIDisable(ulBase);

		// Configure the SSI0 port for master mode with all the configuration settings
		MAP_SSIConfigSetExpClk(ulBase, SYSCTL_CLOCK_FREQ, SPIMode, SSI_MODE_MASTER, SPIFrequency, SPIWordSize);

		// Enable the SSI port.
		MAP_SSIEnable(ulBase);
	}
}


/*
 * @brief Cancels all selections.
 *
 * Makes all SPI devices inactive.
 * @returns void
 */
void SPIDeselect(void) {
	// do the actual deselect
	SPIDeselectISR();
}


/*
 * @brief Cancels all selections. Then immediately sends a byte of 0xFF.
 *
 * Makes all SPI devices inactive. Then immediately sends a byte of 0xFF.
 * @returns void
 */
void SPIDeselectSynchronous(void) {
	// do the actual deselect
	SPIDeselectISR();

	//Send a 0xFF byte
	//Load up the data and send it
	uint32 rcvdat;
	MAP_SSIDataPut(SSI0_BASE, 0xFF); /* Write the data to the tx fifo */
	MAP_SSIDataGet(SSI0_BASE, &rcvdat); /* flush data read during the write */

	// wait until the last transfer is finished
	volatile uint16 i = 0;

	while (MAP_SSIBusy(SSI0_BASE)) {
		i++;
		if (i > SPI_MAX_XFER_IRQ_DELAY) {
			return;
		}
	}
}


boolean SPIBusError_SPIDeselectISR = FALSE;


/*
 * @brief Deselects all select/latch pins
 * Waits till all transfers are finished
 * @return if their is a error return FALSE else ture
 */
boolean SPIDeselectISR(void){
	// wait until the last transfer is finished
	volatile uint16 i = 0;

	while (MAP_SSIBusy(SSI0_BASE)) {
		i++;
		if (i > SPI_MAX_XFER_IRQ_DELAY) {
			SPIBusError_SPIDeselectISR = TRUE;
			return FALSE;
		}
	}

	i = i + 1;
	#if (defined(RONE_V12) || defined(RONE_V9))
	//set output enable low, which pulls all spi select lines high
	MAP_GPIOPinWrite(SPI_ENABLE_PORT, SPI_ENABLE_PIN, 0);
	MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, NULL_SELECT_PINS);
	#endif
	return TRUE;
}


/*
 * @brief Selects a SPI device and sets its wordsize.
 *
 * Does not select anything if the input parameter is not recognized.
 * @param device to be selected
 * @returns void
 */
void SPISelectDevice(uint8 device) {
	SPISelectDeviceISR(device);
}


/*
 * @brief Selects a SPI device and enable high A,B,C
 * @param device to be selected
 * @returns void
 */
void SPISelectDeviceISR(uint8 device) {
	unsigned long junk;
	volatile uint8 q;

	// deassert the previous device
	SPIDeselectISR();

	// clear the receive FIFO
	while (MAP_SSIDataGetNonBlocking(SSI0_BASE, &junk)) {q++;}

	// Configure the device to have the right settings
	SPIConfigureDevice(device);

	// Actually Select the Device
#if (defined(RONE_V9) || defined(RONE_V12))
	switch (device) {
	case SPI_RADIO: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, RADIO_SELECT_PINS);
		break;
	}
	case SPI_MSP430: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, MSP430_SELECT_PINS);
		break;
	}
	default: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, NULL_SELECT_PINS);
		break;
	}
	}
	//set output enable high since A,B,C spi pins are set
	MAP_GPIOPinWrite(SPI_ENABLE_PORT, SPI_ENABLE_PIN, SPI_ENABLE_PIN);
#endif

}


// Configures a SPI device to have the correct SPI mode, frequency, and word size
void SPIConfigureDevice(uint8 device) {
#if (defined(RONE_V9) || defined(RONE_V12))
	switch (device) {
	case SPI_RADIO: {
		SPIConfigure(SSI0_BASE, SPI_RADIO_WORDSIZE, SSI_FRF_MOTO_MODE_0, SPI_RADIO_FREQUENCY);
		break;
	}
	case SPI_MSP430: {
		SPIConfigure(SSI0_BASE, SPI_MSP430_WORDSIZE, SSI_FRF_MOTO_MODE_1, SPI_MSP430_FREQUENCY);
		break;
	}
	default: {
		break;
	}
	}
#endif
}
