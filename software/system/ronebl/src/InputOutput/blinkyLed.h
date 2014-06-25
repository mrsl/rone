/**
 * @file blinky_led.h
 *
 * @brief Functions that control the blinky_led {heartbeat, System, Charge, Power}.
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef BLINKYLED_H_
#define BLINKYLED_H_


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
void blinkyLedSet(uint32 state);


/**
 * @brief Toggles the blinky on and off
 * @returns void
 */
void blinkyLedToggle();


#endif /* BLINKYLED_H_ */
