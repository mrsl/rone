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

#include "roneos.h"
#include "spiInternal.h"

uint8 SPIWordSize;
uint8 SPIMode;
uint32 SPIFrequency;
static osSemaphoreHandle spiMutex;

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
	//TODO Sig, is this read important?  What is it doing?
	ulLoop = SYSCTL_RCGC2_R;

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// configure SSI pins for peripheral control
	MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN));
	MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	MAP_GPIOPinConfigure(GPIO_PA4_SSI0RX);
	MAP_GPIOPinConfigure(GPIO_PA5_SSI0TX);

	// TODO: Make sure this works and figure out a way to make it be pulldowns on V0-V11 and Pullups on V12+ -Jeremy
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

#if defined(RONE_V6)
	// enable all peripherals for the SPI Radio Control
	MAP_SysCtlPeripheralEnable(RADIO_SELECT_PERIPH);
	MAP_SysCtlPeripheralEnable(LED_LE_SYSCTL);
	MAP_SysCtlPeripheralEnable(ACCELEROMETER_SELECT_SYSCTL);

	// drive select pins to correct values while they are still in input mode
	SPIDeselectISR();

	MAP_GPIOPinTypeGPIOOutput(ACCELEROMETER_SELECT_PORT, ACCELEROMETER_SELECT_PIN);
	// enable the select pins for output
	MAP_GPIOPinTypeGPIOOutput(RADIO_SELECT_PORT, RADIO_SELECT_PIN);
	MAP_GPIOPinTypeGPIOOutput(LED_LE_PORT, LED_LE_PIN);
#endif

	// force the port to be enabled by the first call to SPIConfigure()
	SPIWordSize = 0;

	spiMutex = osSemaphoreCreateMutex();
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
		// cache argument values to compare next time
		SPIWordSize = wordSize;
		SPIMode = mode;
		SPIFrequency = frequency;

		MAP_SSIDisable(ulBase);

		// Configure the SSI0 port for master mode with all the configuration settings
		MAP_SSIConfigSetExpClk(ulBase, SYSCTL_CLOCK_FREQ, SPIMode, SSI_MODE_MASTER, SPIFrequency, SPIWordSize);

		// Enable the SSI port.
		MAP_SSIEnable(ulBase);

		// Wait 3 clock ticks.  We'll double that to be sure
		MAP_SysCtlDelay(2);
	}
}


void SPISemaphoreGiveFromISR(void) {
	portBASE_TYPE taskWoken = pdFALSE;
	osSemaphoreGiveFromISR(spiMutex, &taskWoken);
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

	// return the mutex, enable the radio IRQ
	osSemaphoreGive(spiMutex);
	radioIntEnable();
	msp430InterruptEnable();
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

	// return the mutex, enable the radio IRQ
	osSemaphoreGive(spiMutex);
	radioIntEnable();
	msp430InterruptEnable();
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
	#if defined(RONE_V6)
	//Assert all select/latch pins inactive
	MAP_GPIOPinWrite(RADIO_SELECT_PORT, RADIO_SELECT_PIN, RADIO_SELECT_PIN);
	MAP_GPIOPinWrite(LED_LE_PORT, LED_LE_PIN, 0);
	MAP_GPIOPinWrite(ACCELEROMETER_SELECT_PORT, ACCELEROMETER_SELECT_PIN, ACCELEROMETER_SELECT_PIN);
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
	// disable the radio and MSP IRQ, get the mutex
	radioIntDisable();
	msp430InterruptDisable();
	osSemaphoreTake(spiMutex, portMAX_DELAY);

	// Do the actual selection
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
#if defined(RONE_V6)
	switch (device) {
	case SPI_RADIO: {
		MAP_GPIOPinWrite(RADIO_SELECT_PORT, RADIO_SELECT_PIN, 0);
		break;
	}
	case SPI_LEDS_ONOFF: {
		// nothing to do here.  The LED controller is a shift register
		break;
	}
	case SPI_LEDS_DIMMER: {
		// nothing to do here.  The LED controller is a shift register
		break;
	}
	case SPI_ACCELEROMETER: {
		MAP_GPIOPinWrite(ACCELEROMETER_SELECT_PORT, ACCELEROMETER_SELECT_PIN, 0);
		break;
	}
	case SPI_NULL:
	default: {
		break;
	}
	}
#endif
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
	case SPI_SDCARD: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, SDCARD_SELECT_PINS);
		break;
	}
	case SPI_AUDIOSDI: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, AUDIOSDI_SELECT_PINS);
		break;
	}
	case SPI_AUDIOSCI: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, AUDIOSCI_SELECT_PINS);
		break;
	}
	case SPI_EXPAND0: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, EXPAND0_SELECT_PINS);
		break;
	}
	case SPI_EXPAND1: {
		MAP_GPIOPinWrite(SPI_SELECT_PORT, SPI_SELECT_PINS, EXPAND1_SELECT_PINS);
		break;
	}
	case SPI_NULL:
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
#if defined(RONE_V6)
	switch (device) {
	case SPI_RADIO: {
		SPIConfigure(SSI0_BASE, SPI_RADIO_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_RADIO_FREQUENCY);
		break;
	}
	case SPI_LEDS_ONOFF: {
		SPIConfigure(SSI0_BASE, SPI_LEDS_ONOFF_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_NULL_FREQUENCY);
		break;
	}
	case SPI_LEDS_DIMMER: {
		SPIConfigure(SSI0_BASE, SPI_LEDS_DIMMER_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_NULL_FREQUENCY);
		break;
	}
	case SPI_ACCELEROMETER: {
		SPIConfigure(SSI0_BASE, SPI_ACCELEROMETER_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_NULL_FREQUENCY);
		break;
	}
	case SPI_NULL:
	default: {
		break;
	}
	}
#endif
#if (defined(RONE_V9) || defined(RONE_V12))
	switch (device) {
	case SPI_RADIO: {
		SPIConfigure(SSI0_BASE, SPI_RADIO_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_RADIO_FREQUENCY);
		break;
	}
	case SPI_MSP430: {
		SPIConfigure(SSI0_BASE, SPI_MSP430_WORDSIZE, SSI_FRF_MOTO_MODE_1,SPI_MSP430_FREQUENCY);
		break;
	}
	case SPI_SDCARD: {
		SPIConfigure(SSI0_BASE, SPI_SDCARD_WORDSIZE, SSI_FRF_MOTO_MODE_0,SDCardGetFrequency());
		break;
	}
	case SPI_AUDIOSDI: {
		SPIConfigure(SSI0_BASE, SPI_AUDIO_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_AUDIOSDI_FREQUENCY);
		break;
	}
	case SPI_AUDIOSCI: {
		SPIConfigure(SSI0_BASE, SPI_AUDIO_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_AUDIOSCI_FREQUENCY);
		break;
	}
	case SPI_EXPAND0: {
		SPIConfigure(SSI0_BASE, SPI_EXPAND_WORDSIZE, SSI_FRF_MOTO_MODE_1,SPI_EXPAND0_FREQUENCY);
		break;
	}
	case SPI_EXPAND1: {
		SPIConfigure(SSI0_BASE, SPI_EXPAND_WORDSIZE, SSI_FRF_MOTO_MODE_0,SPI_EXPAND1_FREQUENCY);
		break;
	}
	case SPI_NULL:
	default: {
		break;
	}
	}
#endif
}


/*
 * Transmits a byte to nothing via SPI. You must already have the mutex and
 * config set up.
 * Does not set the Slave Select line on anything, so you can clock to nothing.
 * Allows for delays such as to the SD Card.
 *
 * @param dat The byte that will be sent to nothing.
 *
 * @returns boolean TRUE if it was sent correctly, FALSE otherwise.
 */
boolean SPIXmitNullByte(uint8 dat) {
	//Load up the data and send it
	uint32 rcvdat;
	MAP_SSIDataPut(SSI0_BASE, dat); /* Write the data to the tx fifo */
	MAP_SSIDataGet(SSI0_BASE, &rcvdat); /* flush data read during the write */

	// wait until the last transfer is finished
	volatile uint16 i = 0;

	while (MAP_SSIBusy(SSI0_BASE)) {
		i++;
		if (i > SPI_MAX_XFER_IRQ_DELAY) {
			return FALSE;
		}
	}

	return TRUE;
}


/*
 * Transmits a variable number of 0xFF bytes to nothing via SPI. Does not need
 * the mutex to be gotten or the bus to be configured. Essentially talks a
 * set number of null bytes to nothing. Required to send null bytes to set up
 * an SD card like device. Sets the transmit line to high to ensure that the
 * line stays high (required for SD cards).
 *
 * @param device The device that the fake bytes will be configured to send to.
 * @param numBytes The number of bytes that will be sent..
 *
 * @returns boolean TRUE if it was sent correctly, FALSE otherwise.
 */
void SPIXmitNullBytes(uint8 device, uint16 numBytes) {
	unsigned int i;

	// Ensure that the CS line is de-selected
	SPIDeselectISR();

	// Get the SPI Mutex to so nothing else can talk
	radioIntDisable();
	msp430InterruptDisable();
	osSemaphoreTake(spiMutex, portMAX_DELAY);

	// Ensure that the MOSI line to the device is high by making it a GPIO
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, SPI_MOSI_PIN);
	MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_MOSI_PIN, SPI_MOSI_PIN);

	// Configure the device
	SPIConfigureDevice(device);

	// Clock out the fake bytes with the device not selected
	for(i = 0; i < numBytes; i++) {
		SPIXmitNullByte(0xFF);
	}

	// Revert to hardware control of the SSI MOSI line. and re-enable the pull-up
	MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, SPI_MOSI_PIN);
	MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, SPI_MOSI_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	//re-enable the IRQ's and release the Mutex
	osSemaphoreGive(spiMutex);
	radioIntEnable();
	msp430InterruptEnable();
}
