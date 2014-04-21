/**
 * @file blinky_led.h
 *
 * @brief Functions that control the blinky_led {heartbeat, System, Charge, Power}.
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef BLINKY_LED_H_
#define BLINKY_LED_H_

/******** Functions ********/

/**
 * @brief Initializes blinky.
 *
 * Initializes blinky with port B, pin 6 as output. Blinky is turned off with initialization.
 * @returns void
 */
void blinkyLedInit(void);


/**
 * @brief Sets the blinky on or off.

 * @param state	determines whether the pin should be on or off (send 1 to turn on, 0 to turn off)
 * @returns void
 */
void blinkyLedSet(uint32 state); // python rone


/**
 * @brief Flashes the blinky once with delay.
 *
 * Flashes the blinky once (turns it on and then off) with a specified delay in between).
 * @param delay	determines how long the delay is
 * @returns void
 */
void blinky_led_flash(uint32 delay);


/**
 * @brief Updates the blinky with a fast rate.
 *
 * Updates the blinky timer and turns blinky on or off depending on the timer.
 * @returns void
 */
void blinkyUpdateFast(void);


/**
 * @brief Updates the blinky.
 *
 * Updates the blinky timer and turns blinky on or off depending on the timer.
 * @returns void
 */
void blinkyUpdate(void);


/**@brief Update blinky system.
 *
 * Update both the blinky system timer and the brightness.
 * @returns void
 */
void blinkySystemUpdate(void);


/**
 * @brief Build message for blinky.
 *
 * Initializes message to specific brightness.
 * @param msg pointer for message
 * @returns void
 */
void blinkySystemBuildMessage(uint8* msg);

#endif /* BLINKY_LED_H_ */
