/**
 * @file spi.h
 *
 * @brief SPI interface with MSP 430
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef SPI_H_
#define SPI_H_

/******** Defines ********/

#define SPI_NULL                               	0
#define SPI_RADIO                               1
#define SPI_ACCELEROMETER                       2
#define SPI_LEDS_ONOFF                          3
#define SPI_LEDS_DIMMER                         4
#define SPI_MSP430								5
#define SPI_SDCARD								7
#define SPI_AUDIOSDI			                8
#define SPI_AUDIOSCI							9
#define SPI_EXPAND0								10
#define SPI_EXPAND1								11

#define SPI_MAX_XFER_IRQ_DELAY 		32

/******** Functions ********/


/**
 * @brief Initializes SPI unit.
 *
 * Enables peripherals
 * Configures GPIO
 * Sets radio, accelerometer, and LED as output
 * Sets word size as 0
 * @returns void
 */
void SPIInit(void);


/**
 * @brief Selects a SPI device and sets its wordsize.
 *
 * Does not select anything if the input parameter is not recognized.
 * @param device to be selected
 * @returns void
 */
void SPISelectDevice(uint8 device);


/**
 * @brief Selects a SPI device and enable high A,B,C
 * @param device to be selected
 * @returns void
 */
void SPISelectDeviceISR(uint8 device);


/**
 * @brief Cancels all selections.
 *
 * Makes all SPI devices inactive.
 * @returns void
 */
void SPIDeselect(void);


/**
 * @brief Cancels all selections. Then immediately sends a byte of 0xFF.
 *
 * Makes all SPI devices inactive. Then immediately sends a byte of 0xFF.
 * @returns void
 */
void SPIDeselectSynchronous(void);


/**
 * @brief Deselects all select/latch pins
 * Waits till all transfers are finished
 * @return if their is a error return FALSE else ture
 */
boolean SPIDeselectISR(void);


/**
 * @brief
 * \internal
 * TODO Needs explaining
 * \endinternal
 * @return void
 */
void SPISemaphoreGiveFromISR(void);


/**
 * @brief Transmits a byte to nothing via SPI.
 * You must already have the mutex and config set up.
 * Does not set the Slave Select line on anything, so you can clock to nothing.
 * Allows for delays such as to the SD Card.
 *
 * @param dat The byte that will be sent to nothing.
 *
 * @returns boolean TRUE if it was sent correctly, FALSE otherwise.
 */
boolean SPIXmitNullByte(uint8 dat);


/**
 *
 * @brief Transmits a variable number of 0xFF bytes to nothing via SPI.
 * Does not need the mutex to be gotten or the bus to be configured. Essentially talks a
 * set number of null bytes to nothing. Required to send null bytes to set up
 * an SD card like device. Sets the transmit line to high to ensure that the
 * line stays high (required for SD cards).
 *
 * @param device The device that the fake bytes will be configured to send to.
 * @param numBytes The number of bytes that will be sent..
 *
 * @returns boolean TRUE if it was sent correctly, FALSE otherwise.
 */
void SPIXmitNullBytes(uint8 device, uint16 numBytes);


/**
 *	@brief Sets the SPI word size and data format
 *  This function must be called after you have the mutex - after
 *  radio max clock freq = 10mhz
 *  MSP430 max fre clkI cycles - we might be violating this
 *  SD card -fast frequency ~= 2 MHz
 *  sound chip - 2s to be 400kHz at first, then bump up speed
 *
 *	@param ulBase is the SSI port
 *	@param wordSize is the new word size to be set
 *	@param mode is the new transmission mode
 *	@param frequency is the frequency that the SPI clock will run at
 *	@returns void
 */
void SPIConfigure(uint32 ulBase, uint8 wordSize, uint8 mode, uint32 frequency);


/**
 * @brief Configures a SPI device to have the correct SPI mode, frequency, and word size
 *
 * @ param device The device that needs to be configured.
 * @return void
 */
void SPIConfigureDevice(uint8 device);


#endif /* SPI_H_ */
