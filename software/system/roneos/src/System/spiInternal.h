/**
 * @file spi.h
 *
 * @brief SPI interface with MSP 430
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef SPI_INTERNAL_H_
#define SPI_INTERNAL_H_


/* THese defines are here to keep them out of the the public roneos.h */

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
#elif defined(RONE_IRBEACON)
#else
	#error "Robot type must be defined: V6, V9, V12, or IR Beacon"
#endif

#define NULL ((void *)0)

#endif /* SPI_INTERNAL_H_ */
