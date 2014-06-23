#ifndef LEDS_H_
#define LEDS_H_

#define LED_BRIGHTNESS_LOW			10
#define LED_BRIGHTNESS_MEDIUM		15
#define LED_BRIGHTNESS_HIGH			20
#define LED_NUM_ELEMENTS            16
#define LED_NUM_ELEMENTS_SPI_MSG    15
#define LED_NUM_ELEMENTS_PER_COLOR  5

void ledInit(void);
void ledSet(uint8 index, uint8 brightness);
void ledSetAll(uint8 brightness);
void ledUpdate(uint8* data);
void blinkyLEDSet(uint8 brightness);
void ledTimeoutReset();
void ledTimeoutUpdate();

#ifdef RONE_V12
	void ledResetSet(boolean val);
	void ledResetInit(void);
#endif

#endif /*LEDS_H_*/

