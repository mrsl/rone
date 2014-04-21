/**
 * @file leds.h
 * @brief Interface function for LEDs on the robot.
 *
 * @since Mar 22, 2011
 * @author James McLurkin
 */

#ifndef LEDS_H_
#define LEDS_H_

/******** Defines ********/

#define LED_NUM_ELEMENTS                    16
#define LED_NUM_ELEMENTS_PER_COLOR          5

#define LED_RED_START_IDX                   10
#define LED_GREEN_START_IDX                 5
#define LED_BLUE_START_IDX                  0

// Keep these defines in order: red, green, blue, for indexing into arrays
#define LED_RED                             0
#define LED_GREEN                           1
#define LED_BLUE                            2
#define LED_ALL                            	3


#define LED_PATTERN_OFF						0
#define LED_PATTERN_ON						1
#define LED_PATTERN_BLINK					2
#define LED_PATTERN_PULSE					3
#define LED_PATTERN_CIRCLE					4
#define LED_PATTERN_COUNT					5
#define LED_PATTERN_CLAW					6
#define LED_PATTERN_MANUAL					7


#define LED_BRIGHTNESS_LOWEST				2
#define LED_BRIGHTNESS_LOW					5
#define LED_BRIGHTNESS_MED					10
#define LED_BRIGHTNESS_HIGH					15
#define LED_BRIGHTNESS_HIGHER				40

// old defines
//#define LED_RATE_SNAIL						64
//#define LED_RATE_TURTLE						32
//#define LED_RATE_SLOW						16
//#define LED_RATE_MED						8
//#define LED_RATE_FAST						4
//#define LED_RATE_TURBO						2

#define LED_RATE_SNAIL						32
#define LED_RATE_TURTLE						16
#define LED_RATE_SLOW						8
#define LED_RATE_MED						4
#define LED_RATE_FAST						2
#define LED_RATE_TURBO						1

/******** Functions ********/

/**
 * @brief Sets the brightness of a specified dimmer.
 *
 * If the index exceeds the number of elements in the LED (16), does nothing.
 * If the value exceeds the maximum brightness(7F), uses the maximum as the value instead.
 * @param led_idx indicates which dimmer's brightness is to be set
 * @param dimmer brightness to be set
 * @returns void
 */
void ledsSetSingle(uint32 led_idx, uint32 dimmer); // python rone


/**
 * @brief Turns LEDS on or off depending on the input values for each grouping.
 *
 * For example, a 1 for the red leds indicates the first led will be on.
 * A 3 for the blue leds will turn on the second led.
 * A 5 for the green will turn on the first and third leds.
 * @param r binary pattern for red leds
 * @param g binary pattern for green leds
 * @param b binary pattern for blue leds
 * @returns void
 */
void ledsSetBinary(uint8 r, uint8 g, uint8 b);


/**
 * @brief Sets the brightness of all dimmers on the robot.
 *
 * @param dimmer the brightness to be set
 * @returns void
 */
void ledsSetAll(uint32 dimmer);


/*
 * @brief Sets the properties of the LED animation if remote control mode is off.
 *
 * Sets the color, patter, brightness, and rate of the LED animation.
 * Should be called to set all LEDs
 * @param color which color group to set
 * @param pattern lights can run in patterns such as circle/all/etc.
 * @param brightness the brightness to set the pattern
 * @param rate the speed at which the pattern cycles
 * @returns void
 */
void ledsSetPattern(uint8 color, uint8 pattern, uint8 brightness, uint8 rate);


/*
 * @brief Sets the properties of the LED animation if remote control mode is off.
 *
 * Sets the color, patter, brightness, and rate of the LED animation.
 * Should be called to individually set LEDs
 * @param color which color group to set
 * @param pattern lights can run in patterns such as circle/all/etc.
 * @param brightness the brightness to set the pattern
 * @param rate the speed at which the pattern cycles
 * @returns void
 */
void ledsSetPatternSingle(uint8 color, uint8 pattern, uint8 brightness, uint8 rate);


/**
 * @brief Build LED message.
 *
 * Blue, green, red.
 * @param msg pointer for the message
 * @returns void
 */
void ledsBuildMessage(uint8* msg);


/*
 * @brief clear one LED group.
 *
 * Sets the designated led color to a brightness of 0
 * @returns void
 */
void ledsClear(uint8 color);


#endif /* LEDS_H_ */
