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

#include "roneos.h"

#ifdef PART_LM3S8962
	#include "inc/lm3s8962.h"
	#if defined(RONE_V6)
		#define BLINKY_LED_PERIPH 		SYSCTL_PERIPH_GPIOB
		#define BLINKY_LED_BASE 		GPIO_PORTB_BASE
		#define BLINKY_LED_PIN 			GPIO_PIN_4
	#elif defined(RONE_V9)
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
		#error "Robot type must be defined: V6, V9, V12, or beacon"
	#endif
#endif

#ifdef PART_LM3S9B92
	#include "inc/lm3s9b92.h"
	#define BLINKY_LED_PERIPH 		SYSCTL_PERIPH_GPIOD
	//For our boards
	#define BLINKY_LED_BASE 		GPIO_PORTD_BASE
	#define BLINKY_LED_PIN 			GPIO_PIN_4 | GPIO_PIN_0
	//0 is for eval board.
	//for 9b96 eval board
//	#define BLINKY_LED_PERIPH 		SYSCTL_PERIPH_GPIOF
//	#define BLINKY_LED_BASE 		GPIO_PORTF_BASE
//	#define BLINKY_LED_PIN 			GPIO_PIN_3
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

 * @param state	determines whether the pin should be on or off (send 1 to turn on, 0 to turn off)
 * @returns void
 */
void blinkyLedSet(uint32 state) {
	/* The blinky led is active low. */
	taskENTER_CRITICAL();
	if (state == 0) {
		MAP_GPIOPinWrite(BLINKY_LED_BASE, BLINKY_LED_PIN, BLINKY_LED_PIN);
	} else {
		MAP_GPIOPinWrite(BLINKY_LED_BASE, BLINKY_LED_PIN, 0);
	}
	taskEXIT_CRITICAL();
}

/*
 * @brief Flashes the blinky once with delay.
 *
 * Flashes the blinky once (turns it on and then off) with a specified delay in between).
 * @param delay	determines how long the delay is
 * @returns void
 */
void blinky_led_flash(uint32 delay) {
	blinkyLedSet(1);
	SysCtlDelay(delay);
	blinkyLedSet(0);
}


#define BLINKY_TIMER 		60
#define BLINKY_TIMER_FAST	20
#define BLINKY_TIMER_ON 	4
#define BLINKY_SYSTEM_LED_BRIGHTNESS		15

uint32 blinkyTimer = 0;
uint32 blinkySystemTimer = BLINKY_TIMER / 2;
uint8 blinkySystemBrightness = 0;

/*
 * @brief Updates the blinky.
 *
 * Updates the blinky timer and turns blinky on or off depending on the timer.
 * @returns void
 */
void blinkyUpdate(void) {
	if (blinkyTimer > 0) {
		blinkyTimer--;
	}
	if (blinkyTimer == 0) {
		blinkyTimer = BLINKY_TIMER;
	}
	if (blinkyTimer < BLINKY_TIMER_ON) {
		blinkyLedSet(1);
	} else {
		blinkyLedSet(0);
	}
}

/*
 * @brief Updates the blinky with a fast rate.
 *
 * Updates the blinky timer and turns blinky on or off depending on the timer.
 * @returns void
 */
void blinkyUpdateFast(void) {
	if (blinkyTimer > 0) {
		blinkyTimer--;
	}
	if (blinkyTimer == 0) {
		blinkyTimer = BLINKY_TIMER_FAST;
	}
	if (blinkyTimer < BLINKY_TIMER_ON) {
		blinkyLedSet(1);
	} else {
		blinkyLedSet(0);
	}
}


/*
 * @brief Build message for blinky.
 *
 * Initializes message to specific brightness.
 * @param msg pointer for message
 * @returns void
 */
void blinkySystemBuildMessage(uint8* msg) {
	*msg = blinkySystemBrightness;
}


/*
 * @brief Update blinky system.
 *
 * Update both the blinky system timer and the brightness.
 * @returns void
 */
void blinkySystemUpdate(void) {
	if (blinkySystemTimer > 0) {
		blinkySystemTimer--;
	}
	if (blinkySystemTimer == 0) {
		blinkySystemTimer = BLINKY_TIMER;
	}
	if (blinkySystemTimer < BLINKY_TIMER_ON) {
		blinkySystemBrightness = BLINKY_SYSTEM_LED_BRIGHTNESS;
	} else {
		blinkySystemBrightness = 0;
	}
}
