#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"
#include "I2C.h"
#include "leds.h"
#include "SPI_8962.h"
#include "System.h"


// 0x30 | low bits from pins
#define I2C_LED_ADDRESS			0x60		


// register addresses
#define LED_MODE1				0x00
#define LED_MODE2				0x01
#define LED_PWM0				0x02
#define LED_PWM1				0x03
#define LED_PWM2				0x04
#define LED_PWM3				0x05
#define LED_PWM4				0x06
#define LED_PWM5				0x07
#define LED_PWM6				0x08
#define LED_PWM7				0x09
#define LED_PWM8				0x0A
#define LED_PWM9				0x0B
#define LED_PWM10				0x0C
#define LED_PWM11				0x0D
#define LED_PWM12				0x0E
#define LED_PWM13				0x0F
#define LED_PWM14				0x10
#define BLINKY_LED				0x11
#define LED_GRPPWM				0x12
#define LED_GRPFERQ				0x13
#define LED_LEDOUT0				0x14
#define LED_LEDOUT1				0x15
#define LED_LEDOUT2				0x16
#define LED_LEDOUT3				0x17
#define LED_SUBADR1				0x18
#define LED_SUBADR2				0x19
#define LED_SUBADR3				0x1A
#define LED_ALLCALLADR			0x1B
#define LED_IREF				0x1C
#define LED_EFLAG1				0x1D
#define LED_EFLAG2				0x1F

#define LED_RESET				0xD6
#define LED_SLEEP				0x10					

#define LED_SLAVE_READ
#define LED_SLA	VE_WRITE	

#define BLINKY_LED_IDX			15
#define LED_MAX_BRIGHTNESS		250

#if defined(RONE_V11)
#define LED_SCALE 	100
#endif

#if defined(RONE_V12)
#define LED_SCALE	30
#endif

#define LED_BRIGHTNESS_K_RED		(LED_SCALE * 128 / 100)
#define LED_BRIGHTNESS_K_GREEN		(LED_SCALE * 100 / 100)
#define LED_BRIGHTNESS_K_BLUE		(LED_SCALE * 115 / 100)
#define LED_BRIGHTNESS_K_BLINKY		(LED_SCALE * 128 / 100)

#ifdef RONE_V12
	#define LED_RESET_PORT_SEL			P1SEL
	#define LED_RESET_PORT_IN			P1IN
	#define LED_RESET_PORT_OUT			P1OUT
	#define LED_RESET_PORT_DIR			P1DIR
	#define LED_RESET_BIT				BIT1
#endif

#define LED_TIMEOUT_TIME		20 //This is in tenths of seconds

uint8 brightCoeff[LED_NUM_ELEMENTS] = {LED_BRIGHTNESS_K_BLUE, LED_BRIGHTNESS_K_BLUE, LED_BRIGHTNESS_K_BLUE, LED_BRIGHTNESS_K_BLUE, LED_BRIGHTNESS_K_BLUE,
						LED_BRIGHTNESS_K_GREEN, LED_BRIGHTNESS_K_GREEN, LED_BRIGHTNESS_K_GREEN, LED_BRIGHTNESS_K_GREEN, LED_BRIGHTNESS_K_GREEN, 
						LED_BRIGHTNESS_K_RED, LED_BRIGHTNESS_K_RED, LED_BRIGHTNESS_K_RED, LED_BRIGHTNESS_K_RED, LED_BRIGHTNESS_K_RED, 
						LED_BRIGHTNESS_K_BLINKY};
						
static uint8 LEDTimeoutTimer; //Used to timeout LED commands. Updated at 10 Hz

void ledTimeoutReset(){
	LEDTimeoutTimer = LED_TIMEOUT_TIME;
}

void ledTimeoutUpdate(){
	/* 
	 * This function should be called at 10 Hz. If the ledTimeoutReset function
	 * has not been called in LED_TIMEOUT_TIME deciseconds then the Red, Green,
	 * and Blue LEDs on the rone are turned off.
	 */
	if(LEDTimeoutTimer > 0){
		LEDTimeoutTimer--;
	}
	if(LEDTimeoutTimer == 0){
		//No reset function called in a while, timeout and turn the LEDs off
		uint8 i;
		uint8 dataLocal[LED_NUM_ELEMENTS_SPI_MSG];
		
		for (i = 0; i < LED_NUM_ELEMENTS_SPI_MSG; i++) {
			dataLocal[i] = 0;
		}
		I2CSendArray(I2C_LED_ADDRESS, 0x80 | LED_PWM0, dataLocal, LED_NUM_ELEMENTS_SPI_MSG);
	}
}
			
		
void ledInit(void) {
	I2CSend(I2C_LED_ADDRESS, LED_MODE1, 0x80);
	I2CSend(I2C_LED_ADDRESS, LED_IREF, 0x7F);
	I2CSend(I2C_LED_ADDRESS, LED_GRPPWM, 0xFF);
	I2CSend(I2C_LED_ADDRESS, LED_LEDOUT0, 0xFF);
	I2CSend(I2C_LED_ADDRESS, LED_LEDOUT1, 0xFF);
	I2CSend(I2C_LED_ADDRESS, LED_LEDOUT2, 0xFF);
	I2CSend(I2C_LED_ADDRESS, LED_LEDOUT3, 0xFF);
	ledSetAll(0);
	blinkyLEDSet(0);
	ledTimeoutReset();
}


uint8 brightAdj(uint8 brightness, uint8 index) {
	uint16 brightTemp = brightness;
	if (index < LED_NUM_ELEMENTS) {
		brightTemp = (brightTemp * LED_MAX_BRIGHTNESS) >> 8;
		brightTemp = ((uint16)brightCoeff[index] * brightTemp) >> 7;
		if (brightTemp > LED_MAX_BRIGHTNESS) {
			brightTemp = LED_MAX_BRIGHTNESS;
		}
	} else {
		brightTemp = 0;
	}
	return (uint8)brightTemp;
}


void ledSet(uint8 index, uint8 brightness) {
	if (index < LED_NUM_ELEMENTS) {
		brightness = brightAdj(brightness, index);
		I2CSendArray(I2C_LED_ADDRESS, LED_PWM0 + (index & 0x0F), &brightness, 1);
	}
}


void ledSetAll(uint8 brightness) {
	uint8 i;
	uint8 dataLocal[LED_NUM_ELEMENTS_SPI_MSG];
	
	for (i = 0; i < LED_NUM_ELEMENTS_SPI_MSG; i++) {
		dataLocal[i] = brightAdj(brightness, i);
	}
	I2CSendArray(I2C_LED_ADDRESS, 0x80 | LED_PWM0, dataLocal, LED_NUM_ELEMENTS_SPI_MSG);
}


void ledUpdate(uint8* data) {
	uint8 i;
	uint8 dataLocal[LED_NUM_ELEMENTS_SPI_MSG];
	
//	for (i = 0; i < LED_NUM_ELEMENTS_SPI_MSG; i++) {
//		//ledSet(i, data[MSP430_CMD_LED_IDX + i]);
//		ledSet(i, data[i]);
//	}
	for (i = 0; i < LED_NUM_ELEMENTS_SPI_MSG; i++) {
		dataLocal[i] = brightAdj(data[i], i);
	}
	I2CSendArray(I2C_LED_ADDRESS, 0x80 | LED_PWM0, dataLocal, LED_NUM_ELEMENTS_SPI_MSG);
}


void blinkyLEDSet(uint8 brightness) {
	ledSet(BLINKY_LED_IDX, brightness);
}

#ifdef RONE_V12
	void ledResetSet(boolean val) {
		if(val) {
			LED_RESET_PORT_DIR |= LED_RESET_BIT; //Force the output to a 0
			LED_RESET_PORT_OUT &= ~LED_RESET_BIT;
		} else {
			LED_RESET_PORT_DIR &= ~LED_RESET_BIT; //Let the output float to a 1
		}
	}
	
	void ledResetInit(void) {
		LED_RESET_PORT_SEL &= ~LED_RESET_BIT; 
		ledResetSet(TRUE);
	}
#endif
