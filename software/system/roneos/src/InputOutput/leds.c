/*
 * @file leds.c
 *
 * @brief interface functions for LEDs on robot
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"

#include "roneos.h"



#if defined(RONE_V6)

#define LED_MODE_SYSCTL			SYSCTL_PERIPH_GPIOF
#define LED_MODE_PORT			GPIO_PORTF_BASE
#define LED_MODE_PIN 			GPIO_PIN_3

#define LED_DEMO_BLINK_DELAY  	80000
#define LED_DEMO_BLINK_COUNT  	2

#define LED_ONOFF_ALL_MASK 0xFFFF

#define LED_MAX_DIMMER                      0x7F

void ledLatch(void);
void ledLatchOnOffData(uint32 code);
void ledLatchDimmerData(void);

uint8 ledDimmers[LED_NUM_ELEMENTS] = { 0 };

/*
 * @brief Latches the LED.
 *
 * Turns the LED on and then off with a delay of 1.
 * @returns void
 */
void ledLatch(void) {
	MAP_GPIOPinWrite(LED_LE_PORT, LED_LE_PIN, LED_LE_PIN);
	MAP_SysCtlDelay(1);
	MAP_GPIOPinWrite(LED_LE_PORT, LED_LE_PIN, 0);
}

/*
 * @brief Latches the first 16 bits from code into the binary on/off state of the controller.
 *
 * @param code data to be latched
 * @returns void
 */
void ledLatchOnOffData(uint32 code) {
	volatile uint8 q;
	SPISelectDevice(SPI_LEDS_ONOFF);

	// Enable on/off writes
	MAP_GPIOPinWrite(LED_MODE_PORT, LED_MODE_PIN, 0);
	MAP_SSIDataPut(SSI0_BASE, code);
	while (MAP_SSIBusy(SSI0_BASE)) {q++;}
	ledLatch();
	SPIDeselect();
}

/*
 * @brief Latches the stored LED dimmer data (16 x 8 bytes) into the adjustable 7-bit part.
 * of the LED controller.
 *
 * This is the primary low-level interface to the LEDs.
 * @returns void
 */
void ledLatchDimmerData(void) {
	uint8 i, q;
	uint8 ledDimmersShifted[LED_NUM_ELEMENTS];
	static uint8 ledDimmerOold[LED_NUM_ELEMENTS] = {0xFF};
	boolean updateData = false;

	for (i = 0; i < LED_NUM_ELEMENTS; i++) {
		if (ledDimmersOld[i] != ledDimmers[i]) {
			ledDimmersOld[i] = ledDimmers[i];
			updateData = true;
		}
	}

	if(updateData) {
		i = 0;
		ledDimmersShifted[i++] = ledDimmers[15];

		ledDimmersShifted[i++] = ledDimmers[LED_BLUE_START_IDX + 4];
		ledDimmersShifted[i++] = ledDimmers[LED_BLUE_START_IDX + 3];
		ledDimmersShifted[i++] = ledDimmers[LED_BLUE_START_IDX + 2];
		ledDimmersShifted[i++] = ledDimmers[LED_BLUE_START_IDX + 1];
		ledDimmersShifted[i++] = ledDimmers[LED_BLUE_START_IDX + 0];

		ledDimmersShifted[i++] = ledDimmers[LED_GREEN_START_IDX + 4];
		ledDimmersShifted[i++] = ledDimmers[LED_GREEN_START_IDX + 3];
		ledDimmersShifted[i++] = ledDimmers[LED_GREEN_START_IDX + 2];
		ledDimmersShifted[i++] = ledDimmers[LED_GREEN_START_IDX + 1];
		ledDimmersShifted[i++] = ledDimmers[LED_GREEN_START_IDX + 0];

		ledDimmersShifted[i++] = ledDimmers[LED_RED_START_IDX + 4];
		ledDimmersShifted[i++] = ledDimmers[LED_RED_START_IDX + 3];
		ledDimmersShifted[i++] = ledDimmers[LED_RED_START_IDX + 2];
		ledDimmersShifted[i++] = ledDimmers[LED_RED_START_IDX + 1];
		ledDimmersShifted[i++] = ledDimmers[LED_RED_START_IDX + 0];

		SPISelectDevice(SPI_LEDS_DIMMER);
		// Enable dimmer writes
		MAP_GPIOPinWrite(LED_MODE_PORT, LED_MODE_PIN, LED_MODE_PIN);
		for (i = 0; i < LED_NUM_ELEMENTS; i++) {
			MAP_SSIDataPut(SSI0_BASE, ledDimmersShifted[i]);
			while (MAP_SSIBusy(SSI0_BASE)) {q++;}
		}
		ledLatch();
		SPIDeselect();
	}
}

/*
 * @brief Initializes the LEDs.
 *
 * Configure the LED_MODE pin as output.
 * Turns off all dimmer.
 * Latches dimmer data.
 * Latches on/off data with a mask (FFFF).
 * Sets the LED properties - color red, pattern circle, brightness high, and rate fast.
 * @returns void
 */
void ledsInit(void) {
	MAP_SysCtlPeripheralEnable(LED_MODE_SYSCTL);
	MAP_GPIOPinTypeGPIOOutput(LED_MODE_PORT, LED_MODE_PIN);
	MAP_GPIOPinWrite(LED_MODE_PORT, LED_MODE_PIN, 0);
	ledsSetAll(0);
	ledLatchDimmerData();
	ledLatchOnOffData(LED_ONOFF_ALL_MASK);

	ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
}

/*
 * @brief Sets the brightness of a specified dimmer.
 *
 * If the index exceeds the number of elements in the LED (16), does nothing.
 * If the value exceeds the maximum brightness(7F), uses the maximum as the value instead.
 * @param ledIdx indicates which dimmer's brightness is to be set
 * @param val brightness to be set
 * @returns void
 */
void ledsSetSingle(uint32 ledIdx, uint32 val) {
	if (ledIdx >= LED_NUM_ELEMENTS) {
		// bad LED number.  exit
		return;
	}
	if (val > LED_MAX_DIMMER) {
		val = LED_MAX_DIMMER;
	}
	ledDimmers[ledIdx] = val;
}

/*
 * @brief Sets the brightness of all dimmers of a specified color.
 *
 * If the specified color does not match the existing colors, does nothing.
 * @param ledColor specifies the dimmer color to be set
 * @param dimmer brightness to be set
 * @returns void
 */
void ledSetGroup(uint32 color, uint32 brightness) {
	uint8 idx, i;

//	switch (ledColor) {
//	case LED_RED:
//		startIdx = LED_RED_START_IDX;
//		break;
//	case LED_GREEN:
//		startIdx = LED_GREEN_START_IDX;
//		break;
//	case LED_BLUE:
//		startIdx = LED_BLUE_START_IDX;
//		break;
//	default:
//		return;
//	}

	//	for (i = startIdx; i < (startIdx + LED_NUM_ELEMENTS_PER_COLOR); i++) {
	//		ledsSetSingle(i,(uint8)dimmer);
	//	}
	if((color == LED_RED) || (color == LED_GREEN) || (color == LED_BLUE)) {
		idx = color * LED_NUM_ELEMENTS_PER_COLOR;
		for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; ++i) {
			ledsSetSingle(idx++, (uint8)brightness);
		}
	}
}

/*
 * @brief Sets the brightness of all dimmers on the robot.
 *
 * @param dimmer the brightness to be set
 * @returns void
 */
void ledsSetAll(uint32 brightness) {
	ledSetGroup(LED_RED, brightness);
	ledSetGroup(LED_GREEN, brightness);
	ledSetGroup(LED_BLUE, brightness);
}

/******** LED animations ********/
uint8 ledAnimationColor;
uint8 ledAnimationPattern;
uint8 ledAnimationBrightness;
uint8 ledAnimationRate;
uint32 ledAnimationCounter = 0;

#define LED_PATTERN_SHIFTS	4
#define LED_PATTERN_LENGH	10

//										   00,01,02,03,04,05,06,07,08,09
uint8 circlePattern[LED_PATTERN_LENGH] = { 8,16,12, 7, 5, 3, 1, 0, 0, 0};
uint8 circleCounters[LED_NUM_ELEMENTS_PER_COLOR] = {0,2,4,6,8};
uint8 circleState[LED_NUM_ELEMENTS_PER_COLOR] = {0};

uint8 patternCounter = 0;
//										  00,01,02,03,04,05,06,07,08,09
uint8 pulsePattern[LED_PATTERN_LENGH] = { 1, 3, 8,12,16,12, 8, 3, 1, 0};

//										  00,01,02,03,04,05,06,07,08,09
uint8 -[LED_PATTERN_LENGH] = { 0, 0,16,16,16,16,16, 0, 0, 0};



/*
 * @brief Sets the properties of the LED animation.
 *
 * Sets the color, patter, brightness, and rate of the LED animation.
 * @returns void
 */
void ledsSetPattern(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	ledAnimationColor = color;
	ledAnimationPattern = pattern;
	ledAnimationBrightness = brightness;
	ledAnimationRate = rate;
}

/*
 * @brief Updates LED animation.
 *
 * Updates LED animation according to the color, pattern, brightness, and rate previously set.
 * Sets properties using the ledsSetPattern function.
 * @returns void
 */
void ledsUpdate(void) {
	uint8 i, c, val;
	uint8 colorStartIdx, colorEndIdx;

	if (ledAnimationColor == LED_ALL) {
		colorStartIdx = LED_RED;
		colorEndIdx = LED_BLUE;
	} else {
		colorStartIdx = ledAnimationColor;
		colorEndIdx = ledAnimationColor;
	}

	if (ledAnimationPattern == LED_PATTERN_MANUAL) {
		// do nothing
	} else {
		ledsSetAll(0);
		switch (ledAnimationPattern) {
		case LED_PATTERN_ON: {
			for (c = colorStartIdx; c <= colorEndIdx; c++) {
				ledSetGroup(c, ledAnimationBrightness);
			}
			break;
		}
		case LED_PATTERN_BLINK:
		case LED_PATTERN_PULSE: {
			uint8 patternCounterNext;
			ledAnimationCounter++;
			if (ledAnimationCounter > (ledAnimationRate >> 1)) {
				ledAnimationCounter = 0;
				patternCounter++;
				if (patternCounter >= LED_PATTERN_LENGH) {
					patternCounter = 0;
				}
			}
			patternCounterNext = patternCounter + 1;
			if (patternCounternext >= LED_PATTERN_LENGH) {
				patternCounterNext = 0;
			}

			for (c = colorStartIdx; c <= colorEndIdx; c++) {
				if (ledAnimationPattern == LED_PATTERN_BLINK) {
					ledSetGroup(c, (blinkPattern[patternCounter] * ledAnimationBrightness) >> LED_PATTERN_SHIFTS);
				} else {
					uint16 v0 = pulsePattern[patternCounter];
					uint16 v1 = pulsePattern[patternCounterNext];
					uint16 val = (v1 - v0) * ledAnimationCounter / ledAnimationRate + v0;
					//ledSetGroup(c, (pulsePattern[patternCounter] * ledAnimationBrightness) >> LED_PATTERN_SHIFTS);
					ledSetGroup(c, (val * ledAnimationBrightness) >> LED_PATTERN_SHIFTS);
				}
			}
			break;
		}
		case LED_PATTERN_CIRCLE: {
			uint8 idx;
			ledAnimationCounter++;
			if (ledAnimationCounter > (ledAnimationRate >> 1)) {
				// circle animations run twice as fast to counteract the interpolation
				ledAnimationCounter = 0;
				val = circleState[0];
				for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
					circleCounters[i]++;
					if (circleCounters[i] >= LED_PATTERN_LENGH) {
						circleCounters[i] = 0;
					}
				}
				for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
					circleState[i] = circlePattern[circleCounters[i]];
				}
			}
			for (c = colorStartIdx; c <= colorEndIdx; c++) {
				idx = c * LED_NUM_ELEMENTS_PER_COLOR;
				for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; ++i) {
					ledsSetSingle(idx++, ((uint32)circleState[i] * (uint32)ledAnimationBrightness) >> LED_PATTERN_SHIFTS);
				}
			}
			break;
		}
		case LED_PATTERN_COUNT: {
			// ledAnimationRate contains the count
			uint8 idx = colorStartIdx * LED_NUM_ELEMENTS_PER_COLOR;
			uint8 val;
			ledAnimationCounter++;
			if (ledAnimationCounter > LED_RATE_FAST) {
				ledAnimationCounter = 0;
				patternCounter++;
				if (patternCounter >= LED_PATTERN_LENGH) {
					patternCounter = 0;
				}
			}

			if(ledAnimationRate > LED_NUM_ELEMENTS_PER_COLOR) {
				ledAnimationRate = LED_NUM_ELEMENTS_PER_COLOR;
			}
			val = ((8 + (pulsePattern[patternCounter] >> 1)) * ledAnimationBrightness)  >> LED_PATTERN_SHIFTS;
			for (i = 0; i < ledAnimationRate; i++) {
				ledsSetSingle(idx++, val);
			}
			break;
		}
		default:
			break;
		}
	}
	ledLatchDimmerData();
}

#endif


#if defined(RONE_V9) || defined(RONE_V12)
#define LED_DEMO_BLINK_DELAY  	80000
#define LED_DEMO_BLINK_COUNT  	2

#define LED_MAX_DIMMER          0xFF

uint8 ledDimmers[LED_NUM_ELEMENTS] = { 0 };

/*
 * @brief Initializes the LEDs.
 *
 * Configure the LED_MODE pin as output.
 * Turns off all dimmer.
 * Latches dimmer data.
 * Latches on/off data with a mask (FFFF).
 * Sets the LED properties - color red, pattern circle, brightness high, and rate fast.
 * @returns void
 */
void ledsInit(void) {
	ledsSetAll(0);
	ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
}

/*
 * @brief Sets the brightness of a specified dimmer.
 *
 * If the index exceeds the number of elements in the LED (16), does nothing.
 * If the value exceeds the maximum brightness(7F), uses the maximum as the value instead.
 * @param ledIdx indicates which dimmer's brightness is to be set
 * @param val brightness to be set
 * @returns void
 */
void ledsSetSingle(uint32 ledIdx, uint32 val) {
	if (ledIdx >= LED_NUM_ELEMENTS) {
		// bad LED number.  exit
		return;
	}
	if (val > LED_MAX_DIMMER) {
		val = LED_MAX_DIMMER;
	}
	ledDimmers[ledIdx] = val;
}



/*
 * @brief Gets the start index of the LEDs for the input color group
 *
 * Default will return the red LED start index
 * @param color to find the start index of
 * @returns the start index of the LEDs
 */
uint8 ledStartIdx(uint8 color) {
	switch (color) {
	case LED_RED:
		return LED_RED_START_IDX;
	case LED_GREEN:
		return LED_GREEN_START_IDX;
	case LED_BLUE:
		return LED_BLUE_START_IDX;
	default:
		return LED_RED_START_IDX;
	}
}


/*
 * @brief Sets the brightness of all dimmers of a specified color.
 *
 * If the specified color does not match the existing colors, does nothing.
 * @param ledColor specifies the dimmer color to be set
 * @param dimmer brightness to be set
 * @returns void
 */
static void ledSetGroup(uint32 ledColor, uint32 dimmer) {
	int i;
	uint8 startIdx = ledStartIdx(ledColor);

	for (i = startIdx; i < (startIdx + LED_NUM_ELEMENTS_PER_COLOR); i++) {
		ledsSetSingle(i,(uint8)dimmer);
	}
}

/*
 * @brief Sets the brightness of all dimmers on the robot.
 *
 * @param dimmer the brightness to be set
 * @returns void
 */
void ledsSetAll(uint32 dimmer) {
	int i;
	for (i = 0; i < LED_NUM_ELEMENTS; i++) {
		ledDimmers[i] = (uint8)dimmer;
	}
}


/******** LED animations ********/
typedef struct LEDSettings {
     uint8 pattern;
     uint8 brightness;
     uint8 rate;
     uint8 animationCounter;
     uint8 patternCounter;
} LEDSettings;

LEDSettings ledAnimation[3] = {{0}, {0}, {0}};


#define LED_PATTERN_SHIFTS	4
#define LED_PATTERN_LENGH	10


//										   00,01,02,03,04,05,06,07,08,09
static const uint8 circlePattern[LED_PATTERN_LENGH] = { 8,16,12, 7, 5, 3, 1, 0, 0, 0};
static uint8 circleCounters[LED_NUM_ELEMENTS_PER_COLOR] = {0,2,4,6,8};
static uint8 circleState[LED_NUM_ELEMENTS_PER_COLOR] = {0};

//										  00,01,02,03,04,05,06,07,08,09
static const uint8 pulsePattern[LED_PATTERN_LENGH] = { 1, 3, 8,12,16,12, 8, 3, 1, 0};

//										  00,01,02,03,04,05,06,07,08,09
static const uint8 blinkPattern[LED_PATTERN_LENGH] = { 0, 0,16,16,16,16,16, 0, 0, 0};

//										 00,01,02,03,04,05,06,07,08,09
static const uint8 clawPattern0[LED_PATTERN_LENGH] = { 8,12,16, 16, 16, 16, 16, 12, 8, 0};
static const uint8 clawPattern1[LED_PATTERN_LENGH] = { 12,8,4, 0, 4, 8, 12, 16, 16, 16};
static const uint8 clawPattern2[LED_PATTERN_LENGH] = { 8,12,16, 12, 8, 4, 0, 0, 0, 4};
static uint8 clawCounters[LED_NUM_ELEMENTS_PER_COLOR] = {8,4,0,0,4};
static uint8 clawState[LED_NUM_ELEMENTS_PER_COLOR] = {0};

/*
 * @brief clear one LED group.
 *
 * Sets the designated led color to a brightness of 0
 * Should not be called by user
 * @returns void
 */
void ledsClear_rc(uint8 color) {
	int c;
	uint8 colorStart = LED_RED;
	uint8 colorEnd = LED_BLUE;

	if ((color >= LED_RED) && (color <= LED_ALL)) {
		if (color != LED_ALL) {
			// only update one group
			colorStart = color;
			colorEnd = color;
		}

		for (c = colorStart; c <= colorEnd; c++) {
			ledAnimation[c].pattern = LED_PATTERN_OFF;
		}
	}
}

/*
 * @brief clear one LED group.
 *
 * Sets the designated led color to a brightness of 0
 * @returns void
 */
void ledsClear(uint8 color) {
	if (rcMode == RC_MODE_OFF) {
		ledsClear_rc(color);
	}
}


/*
 * @brief Turns LEDS on or off depending on the input values for each grouping.
 * Should not be called by user. Call ledsSetBinary() instead.
 *
 * For example, a 1 for the red leds indicates the first led will be on.
 * A 3 for the blue leds will turn on the first and second led.
 * A 5 for the green will turn on the first and third leds.
 * @param r binary pattern for red leds
 * @param g binary pattern for green leds
 * @param b binary pattern for blue leds
 * @returns void
 */
void ledsSetBinary_rc(uint8 r, uint8 g, uint8 b) {	//### use led to display 5-bit binary numbers
	int i;
	ledAnimation[LED_RED].pattern = LED_PATTERN_MANUAL;
	ledAnimation[LED_GREEN].pattern = LED_PATTERN_MANUAL;
	ledAnimation[LED_BLUE].pattern = LED_PATTERN_MANUAL;

	uint8 startIdx = ledStartIdx(LED_RED);
	for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
		ledsSetSingle(i+startIdx, (r & (1<<i)) ? LED_BRIGHTNESS_HIGHER : 0);
	}

	startIdx = ledStartIdx(LED_GREEN);
	for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
		ledsSetSingle(i+startIdx, (g & (1<<i)) ? LED_BRIGHTNESS_HIGHER : 0);
	}

	startIdx = ledStartIdx(LED_BLUE);
	for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
		ledsSetSingle(i+startIdx, (b & (1<<i)) ? LED_BRIGHTNESS_HIGHER : 0);
	}
}


/*
 * @brief Turns LEDS on or off depending on the input values for each grouping.
 *
 * For example, a 1 for the red leds indicates the first led will be on.
 * A 3 for the blue leds will turn on the first and second led.
 * A 5 for the green will turn on the first and third leds.
 * @param r binary pattern for red leds
 * @param g binary pattern for green leds
 * @param b binary pattern for blue leds
 * @returns void
 */
void ledsSetBinary(uint8 r, uint8 g, uint8 b) { //### use led to display 5-bit binary numbers
	if (rcMode == RC_MODE_OFF) {
		ledsSetBinary_rc(r, g, b);
	}
}



/*
 * \internal
 * @brief Sets the properties of the LED animation. Should not be called by user. Call
 *        ledsSetPattern() or ledsSetPatternSingle() instead.
 *
 * Sets the color, patter, brightness, and rate of the LED animation.
 * @param color       which colour group to set
 * @param brightness  the brightness to set the pattern
 * @param rate        the speed at which the pattern cycles
 * @returns void
 * \endinternal
 */
void ledsSetPatternSingle_rc(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	uint8 c;
	uint8 colorStart = LED_RED;
	uint8 colorEnd = LED_BLUE;

	if (color != LED_ALL) {
		// only update one group
		colorStart = color;
		colorEnd = color;
	}

	for (c = colorStart; c <= colorEnd; c++) {
		ledAnimation[c].pattern = pattern;
		//TODO hack to keep brightness values consistent line with v6 code
		ledAnimation[c].brightness = (uint16)brightness * 200 / 15;
		ledAnimation[c].rate = rate;
	}
}


/*
 * @brief Sets the properties of the LED animation. Should not be called by user. Call
 *        ledsSetPattern() or ledsSetPatternSingle() instead.
 *
 * Sets the color, patter, brightness, and rate of the LED animation.
 * @param color which color group to set
 * @param pattern lights can run in patterns such as circle/all/etc.
 * @param brightness the brightness to set the pattern
 * @param rate the speed at which the pattern cycles
 * @returns void
 */
void ledsSetPattern_rc(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	ledsClear_rc(LED_ALL);
	ledsSetPatternSingle_rc(color, pattern, brightness, rate);
}


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
void ledsSetPattern(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	ledsClear(LED_ALL);
	ledsSetPatternSingle(color, pattern, brightness, rate);
}


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
void ledsSetPatternSingle(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	if (rcMode == RC_MODE_OFF) {
		ledsSetPatternSingle_rc(color, pattern, brightness, rate);
	}
}

#define RATE_SHIFT	4

/*
 * @brief Updates LED animation.
 *
 * Updates LED animation according to the color, pattern, brightness, and rate previously set.
 * Sets properties using the ledsSetPattern function.
 * @returns void
 */
void ledsUpdate(void) {
	uint8 c;
	uint8 i, val;
	uint32 t = osTaskGetTickCount();
	uint32 patternCounter;

	for (c = LED_RED; c <= LED_BLUE; c++) {
		switch (ledAnimation[c].pattern) {
		case LED_PATTERN_MANUAL: {
			// do nothing
			break;
		}
		case LED_PATTERN_ON: {
			ledSetGroup(c, ledAnimation[c].brightness);
			break;
		}
		case LED_PATTERN_OFF: {
			ledSetGroup(c, 0);
			break;
		}
		case LED_PATTERN_BLINK:
		case LED_PATTERN_PULSE: {
			uint8 patternCounterNext;
			patternCounter = ((t / ((uint32)ledAnimation[c].rate << RATE_SHIFT)) % LED_PATTERN_LENGH);
			patternCounterNext = ((patternCounter + 1) % LED_PATTERN_LENGH);

			if (ledAnimation[c].pattern == LED_PATTERN_BLINK) {
				ledSetGroup(c, (blinkPattern[patternCounter] * ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS);
			} else {
				// code for interpolation. put this back after we debug the individyal animations
	//			uint16 v0 = pulsePattern[patternCounter];
	//			uint16 v1 = pulsePattern[patternCounterNext];
	//			uint16 val = (v1 - v0) * ledAnimationCounter / ledAnimationRate + v0;
	//			ledSetGroup(c, (val * ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS);

				ledSetGroup(c, (pulsePattern[patternCounter] * ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS);
			}
			break;
		}
		case LED_PATTERN_CIRCLE: {
			uint8 idx;
	//		ledAnimationCounter++;
	//		if (ledAnimationCounter > (ledAnimationRate)) {
	//			// circle animations run twice as fast to counteract the interpolation
	//			ledAnimationCounter = 0;
	//			val = circleState[0];
	//			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
	//				circleCounters[i]++;
	//				if (circleCounters[i] >= LED_PATTERN_LENGH) {
	//					circleCounters[i] = 0;
	//				}
	//			}
	//			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
	//				circleState[i] = circlePattern[circleCounters[i]];
	//			}
	//		}
	//		for (c = colorStart; c <= colorEnd; c++) {
	//			idx = ledStartIdx(c);
	//			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; ++i) {
	//				ledsSetSingle(idx++, ((uint32) circleState[i] * (uint32) ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS);
	//			}
	//		}

			patternCounter = ((t / ((uint32)(ledAnimation[c].rate) << RATE_SHIFT)) % LED_PATTERN_LENGH);
			idx = ledStartIdx(c);
			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; ++i) {
				ledsSetSingle(idx++, (circlePattern[(patternCounter + i * 2) % LED_PATTERN_LENGH] * (uint32) ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS);
			}

			break;
		}
		case LED_PATTERN_COUNT: {
			// ledAnimationRate contains the count
			uint8 idx = ledStartIdx(c);
			uint8 val;
	//		ledAnimationCounter++;
	//		if (ledAnimationCounter > LED_RATE_FAST) {
	//			ledAnimationCounter = 0;
	//			patternCounter++;
	//			if (patternCounter >= LED_PATTERN_LENGH) {
	//				patternCounter = 0;
	//			}
	//		}
	//
	//		if (ledAnimationRate > LED_NUM_ELEMENTS_PER_COLOR) {
	//			ledAnimationRate = LED_NUM_ELEMENTS_PER_COLOR;
	//		}
	//		val = ((8 + (pulsePattern[patternCounter] >> 1))
	//				* ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS;
	//		for (i = 0; i < ledAnimationRate; i++) {
	//			ledsSetSingle(idx++, val);
	//		}

			patternCounter = ((t / LED_RATE_FAST) % LED_PATTERN_LENGH);
			val = ((8 + (pulsePattern[patternCounter] >> 1))
					* ledAnimation[c].brightness) >> LED_PATTERN_SHIFTS;
			for (i = 0; i < ledAnimation[c].rate; i++) {
				ledsSetSingle(idx++, val);
			}

			break;
		}
		case LED_PATTERN_CLAW: {
	//		uint8 idx;
	//		uint8 patternType, patternIndex;
	//		ledAnimationCounter++;
	//		if (ledAnimationCounter > (ledAnimationRate >> 1)) {
	//			// circle animations run twice as fast to counteract the interpolation
	//			ledAnimationCounter = 0;
	//			val = clawState[0];
	//			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
	//				clawCounters[i]++;
	//				if (clawCounters[i] >= LED_PATTERN_LENGH) {
	//					clawCounters[i] = 0;
	//				}
	//			}
	//			//set different light patterns for indices 0 / 1&4 / 2&3
	//			// TODO port this to the new counter system.  I don't understand this code...
	//			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; i++) {
	//				patternType = min(i, LED_NUM_ELEMENTS_PER_COLOR - i);
	////						if (clawCounters[i] == LED_PATTERN_LENGH - 1 ||
	////								(clawCounters[i] > LED_PATTERN_LENGH / 2 - 1 - (LED_NUM_ELEMENTS_PER_COLOR - patternType)
	////									&& clawCounters[i] < LED_PATTERN_LENGH / 2 - 1 + (LED_NUM_ELEMENTS_PER_COLOR - patternType))) {
	////							clawState[i] = clawPattern[(clawCounters[i] + LED_PATTERN_LENGH - 4 * patternType) % LED_PATTERN_LENGH];
	////						}
	////						else {
	////							clawState[i] = clawPattern[(clawCounters[i] + LED_PATTERN_LENGH - 4 * patternType) % LED_PATTERN_LENGH] - 4*patternType;
	////						}
	//				if (patternType == 0) {
	//					clawState[i] = clawPattern0[clawCounters[i]];
	//				} else if (patternType == 1) {
	//					clawState[i] = clawPattern1[clawCounters[i]];
	//				} else {
	//					clawState[i] = clawPattern2[clawCounters[i]];
	//				}
	//			}
	//		}
	//		for (c = colorStart; c <= colorEnd; c++) {
	//			idx = ledStartIdx(c);
	//			for (i = 0; i < LED_NUM_ELEMENTS_PER_COLOR; ++i) {
	//				ledsSetSingle(
	//						idx++,
	//						((uint32) clawState[i]
	//								* (uint32) ledAnimation[c].brightness)
	//								>> LED_PATTERN_SHIFTS);
	//			}
	//		}
			break;
		}
		default:
			break;
		}
	}
}



/*
 * @brief Build LED message.
 *
 * Blue, green, red.
 * @param msg pointer for the message
 * @returns void
 */
void ledsBuildMessage(uint8* msg) {
	uint8 i = 0;

	msg[i++] = ledDimmers[LED_BLUE_START_IDX + 0];
	msg[i++] = ledDimmers[LED_BLUE_START_IDX + 1];
	msg[i++] = ledDimmers[LED_BLUE_START_IDX + 2];
	msg[i++] = ledDimmers[LED_BLUE_START_IDX + 3];
	msg[i++] = ledDimmers[LED_BLUE_START_IDX + 4];

	msg[i++] = ledDimmers[LED_GREEN_START_IDX + 0];
	msg[i++] = ledDimmers[LED_GREEN_START_IDX + 1];
	msg[i++] = ledDimmers[LED_GREEN_START_IDX + 2];
	msg[i++] = ledDimmers[LED_GREEN_START_IDX + 3];
	msg[i++] = ledDimmers[LED_GREEN_START_IDX + 4];

	msg[i++] = ledDimmers[LED_RED_START_IDX + 0];
	msg[i++] = ledDimmers[LED_RED_START_IDX + 1];
	msg[i++] = ledDimmers[LED_RED_START_IDX + 2];
	msg[i++] = ledDimmers[LED_RED_START_IDX + 3];
	msg[i++] = ledDimmers[LED_RED_START_IDX + 4];
}


#endif


#if defined (RONE_IRBEACON)

#define RED_LED_PERIPH 		SYSCTL_PERIPH_GPIOG
#define RED_LED_BASE 		GPIO_PORTG_BASE
#define RED_LED_PIN 		GPIO_PIN_1
#define RED_LED_PWM_BIT		0x0002

#define GREEN_LED_PERIPH 	SYSCTL_PERIPH_GPIOB
#define GREEN_LED_BASE 		GPIO_PORTB_BASE
#define GREEN_LED_PIN 		GPIO_PIN_0
#define GREEN_LED_PWM_BIT	0x0004

#define BLUE_LED_PERIPH 	SYSCTL_PERIPH_GPIOB
#define BLUE_LED_BASE 		GPIO_PORTB_BASE
#define BLUE_LED_PIN 		GPIO_PIN_1
#define BLUE_LED_PWM_BIT	0x0008

/******** LED animations ********/
uint8 ledAnimationColor;
uint8 ledAnimationPattern;
uint8 ledAnimationBrightness;
uint8 ledAnimationRate;
uint32 ledAnimationCounter = 0;

#define LED_BRIGHTNESS_LOWEST_BEACON 			10
#define LED_BRIGHTNESS_LOW_BEACON				30
#define LED_BRIGHTNESS_MED_BEACON				50
#define LED_BRIGHTNESS_HIGH_BEACON				80

/* Blinks one of the indicator LEDs.
 * @param: color selects which LED will be flashed
 * @param: delay is the length of time the LED will be on
 */
void ledFlash(uint8 color, uint32 delay) {
	ledsSetSingle(color, 1);
	SysCtlDelay(delay);
	ledsSetSingle(color, 0);
}

/* Initializes all three of the indicator LEDs and their pwm functionality.
   Red is Port G, pin 1; green is port B, pin 0; blue is port B, pin 1.
   They are all initialized to be turned off. */
void ledsInit(void) {
	MAP_SysCtlPeripheralEnable(RED_LED_PERIPH);
	MAP_GPIOPinTypePWM(RED_LED_BASE, RED_LED_PIN);

	MAP_SysCtlPeripheralEnable(GREEN_LED_PERIPH);
	MAP_GPIOPinTypePWM(GREEN_LED_BASE, GREEN_LED_PIN);

	MAP_SysCtlPeripheralEnable(BLUE_LED_PERIPH);
	MAP_GPIOPinTypePWM(BLUE_LED_BASE, BLUE_LED_PIN);

	pwmInitializeLed();

	ledFlash(LED_RED, 30000);
	ledFlash(LED_GREEN, 30000);
	ledFlash(LED_BLUE, 30000);
}

/* Sets one of the indicator LEDs
    They are active high.
    @param: ledColor indicates which LED is being set
    @param: dimmer indicates the brightness and should be between 0 and 100
*/

void ledsSetSingle (uint32 ledColor, uint32 dimmer) {
	pwmSetDuty(ledColor, dimmer);
	//pwmEnableSingle(ledColor);
}

/*
 * The beacon only has a single LED of each color, so setting a color group simply sets the single LED
 * corresponding to that color.
*/
void ledsSetGroup (uint32 ledColor, uint32 state) {
	ledsSetSingle(ledColor, state);
}

/*
 * Sets all of the LEDs.
 * TODO - make brightness mean something
 */
void ledsSetAll (uint32 brightness) {
	ledsSetSingle(LED_RED, brightness);
	ledsSetSingle(LED_GREEN, brightness);
	ledsSetSingle(LED_BLUE, brightness);
}

/*
 * This sets an LED pattern.
 *
 * color - any one of RED, GREEN, BLUE, or ALL
 * pattern - the name of the pattern
 * brightness - any number from 0-100, determines the duty cycle of the PWM
 * rate - determines the speed of the animation
 *
 */
void ledsSetPattern(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	ledAnimationColor = color;
	ledAnimationPattern = pattern;
	ledAnimationBrightness = brightness;
	ledAnimationRate = rate;

}

void ledsSetPattern_rc(uint8 color, uint8 pattern, uint8 brightness, uint8 rate){

}

// This is an rone function. Not necessary for IR beacon because there's only one LED of each color.
uint8 ledStartIdx(uint8 color) {
	return 0; // It needs some kind of return.
}

// This will need to be populated. Placeholder for now.
void ledsUpdate(void) {

}

// This may or may not be necessary. Need to look at rone code.
void ledsBuildMessage(uint8* msg) {

}
#endif
