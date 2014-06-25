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
#define SPI_MSP430								5
#define SPI_LEDS_DIMMER                         4
#define SPI_SDCARD								7
#define SPI_AUDIOSDI			                8
#define SPI_AUDIOSCI							9
#define SPI_EXPAND0								10
#define SPI_EXPAND1								11

//TODO Determine what speed we set the SPI clock to, it might be a bottleneck.
#define SPI_RADIO_FREQUENCY						5000000
#define SPI_MSP430_FREQUENCY					2000000
#define SPI_SDCARD_FREQUENCY					350000
#define SPI_AUDIOSDI_FREQUENCY					2000000
#define SPI_AUDIOSCI_FREQUENCY					2000000
#define SPI_EXPAND0_FREQUENCY					2000000
#define SPI_EXPAND1_FREQUENCY					1000000
#define SPI_NULL_FREQUENCY						1000000

#define SPI_RADIO_WORDSIZE                      8
#define SPI_LEDS_ONOFF_WORDSIZE                 16
#define SPI_LEDS_DIMMER_WORDSIZE                7
#define SPI_MSP430_WORDSIZE						8
#define SPI_SDCARD_WORDSIZE						8
#define SPI_AUDIO_WORDSIZE						16
//TODO: maj5 - added the define because when I tried to define RONE_V6 it wasn't define, so the number is random
#define SPI_ACCELEROMETER_WORDSIZE				8
#define SPI_EXPAND_WORDSIZE						8

#if defined(RONE_V6)

#define SPI_MOSI_PIN							GPIO_PIN_5
#define SPI_MISO_PIN							GPIO_PIN_4
#define SPI_CLK_PIN								GPIO_PIN_2

#define RADIO_SELECT_PORT                       GPIO_PORTA_BASE
#define RADIO_SELECT_PERIPH                     SYSCTL_PERIPH_GPIOA
#define RADIO_SELECT_PIN                        GPIO_PIN_7

#define LED_LE_SYSCTL                           SYSCTL_PERIPH_GPIOA
#define LED_LE_PORT                             GPIO_PORTA_BASE
#define LED_LE_PIN                              GPIO_PIN_6

#define ACCELEROMETER_SELECT_SYSCTL             SYSCTL_PERIPH_GPIOA
#define ACCELEROMETER_SELECT_PORT               GPIO_PORTA_BASE
#define ACCELEROMETER_SELECT_PIN                GPIO_PIN_3
#elif defined(RONE_V9)
	#define SPI_ENABLE_PERIPH						SYSCTL_PERIPH_GPIOG
	#define SPI_ENABLE_PORT							GPIO_PORTG_BASE
	#define SPI_ENABLE_PIN							GPIO_PIN_0

	#define SPI_MOSI_PIN							GPIO_PIN_5
	#define SPI_MISO_PIN							GPIO_PIN_4
	#define SPI_CLK_PIN								GPIO_PIN_2

	#define SPI_SELECT_PERIPH                  		SYSCTL_PERIPH_GPIOA
	#define SPI_SELECT_PORT							GPIO_PORTA_BASE
	#define SPI_SELECT_PINS							(GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7)
	#define NULL_SELECT_PINS						(0          | 0          | 0         )
	#define MSP430_SELECT_PINS						(GPIO_PIN_3 | 0          | 0         )
	#define RADIO_SELECT_PINS						(0          | GPIO_PIN_6 | 0         )
	#define AUDIOSCI_SELECT_PINS                    (GPIO_PIN_3 | GPIO_PIN_6 | 0         )
	#define AUDIOSDI_SELECT_PINS                    (0          | 0          | GPIO_PIN_7)
	#define SDCARD_SELECT_PINS                    	(GPIO_PIN_3 | 0          | GPIO_PIN_7)
	#define EXPAND0_SELECT_PINS                    	(0			| GPIO_PIN_6 | GPIO_PIN_7)
	#define EXPAND1_SELECT_PINS                    	(GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7)
#elif defined(RONE_V12)
	#define SPI_ENABLE_PERIPH						SYSCTL_PERIPH_GPIOG
	#define SPI_ENABLE_PORT							GPIO_PORTG_BASE
	#define SPI_ENABLE_PIN							GPIO_PIN_0

	#define SPI_MOSI_PIN							GPIO_PIN_5
	#define SPI_MISO_PIN							GPIO_PIN_4
	#define SPI_CLK_PIN								GPIO_PIN_2

	#define SPI_SELECT_PERIPH                  		SYSCTL_PERIPH_GPIOA
	#define SPI_SELECT_PORT							GPIO_PORTA_BASE
	#define SPI_SELECT_PINS							(GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7)
	#define NULL_SELECT_PINS						(0          | 0          | 0         )
	#define MSP430_SELECT_PINS						(GPIO_PIN_3 | 0          | 0         )
	#define RADIO_SELECT_PINS						(0          | GPIO_PIN_6 | 0         )
	#define AUDIOSCI_SELECT_PINS                    (GPIO_PIN_3 | GPIO_PIN_6 | 0         )
	#define AUDIOSDI_SELECT_PINS                    (0          | 0          | GPIO_PIN_7)
	#define SDCARD_SELECT_PINS                    	(GPIO_PIN_3 | 0          | GPIO_PIN_7)
	#define EXPAND0_SELECT_PINS                    	(0			| GPIO_PIN_6 | GPIO_PIN_7)
	#define EXPAND1_SELECT_PINS                    	(GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7)
#else
	//#error "Robot type must be defined: V6, V9, V12, or IR Beacon"
#endif

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
