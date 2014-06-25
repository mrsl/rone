/*
 * @file blinky_led.c
 * @brief Functions that control the blinky_led {heartbeat, System, Charge, Power}
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"


#include "System/bootloader.h"

#ifdef PART_LM3S8962
	#include "inc/lm3s8962.h"
	#if defined(RONE_V9)
		#define BLINKY_LED_PERIPH 		SYSCTL_PERIPH_GPIOE
		#define BLINKY_LED_BASE 		GPIO_PORTE_BASE
		#define BLINKY_LED_PIN 			GPIO_PIN_1
	#elif defined(RONE_V12)
		#define BLINKY_LED_PERIPH 		SYSCTL_PERIPH_GPIOB
		#define BLINKY_LED_BASE 		GPIO_PORTB_BASE
		#define BLINKY_LED_PIN 			GPIO_PIN_6
	#elif defined(RONE_IRBEACON)
		#define BLINKY_LED_PERIPH 		SYSCTL_PERIPH_GPIOE
		#define BLINKY_LED_BASE 		GPIO_PORTE_BASE
		#define BLINKY_LED_PIN 			GPIO_PIN_1
	#else
		#error "Robot type must be defined: V9, V12, or beacon"
	#endif
#endif

/*
 *	@brief Initializes blinky.
 *
 * 	Initializes blinky with port B, pin 6 as output. Blinky is turned off with initialization.
 * 	@returns void
 */
void blinkyLedInit(void) {
	MAP_SysCtlPeripheralEnable(BLINKY_LED_PERIPH);
	MAP_GPIOPinTypeGPIOOutput(BLINKY_LED_BASE, BLINKY_LED_PIN);
	blinkyLedSet(FALSE);
}


/*
 * @brief Sets the blinky on or off.
 * @param state	determines whether the pin should be on or off (set 1 to turn on, 0 to turn off)
 * @returns void
 */
void blinkyLedSet(uint32 state) {
	/* The blinky led is active low. */
	if (state == 0) {
		MAP_GPIOPinWrite(BLINKY_LED_BASE, BLINKY_LED_PIN, BLINKY_LED_PIN);
	} else {
		MAP_GPIOPinWrite(BLINKY_LED_BASE, BLINKY_LED_PIN, 0);
	}
}


/*
 * @brief Toggles the blinky on and off
 * @returns void
 */
void blinkyLedToggle(void) {
	static uint8 state = 0;
	if (state == 0) {
		blinkyLedSet(1);
		state = 1;
	} else {
		blinkyLedSet(0);
		state = 0;
	}
}
