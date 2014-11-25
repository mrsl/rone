/*
 * blinkTest.c
 *
 *  Created on: May 22, 2012
 *      Author: Lindsay
 *
 *		This is a test of the LEDs, both the heartbeat and the RGB indicators.
 */

#include "roneos.h"
#include "ronelib.h"

void blink(void);
void buttonBlink(void);
void buttonColors(void);
void testLedBrightness(void);

int main (void){
	systemPreInit();
	systemInit();
	//blinky_led_init();
	//buttons_init();
	buttonBlink();
	//buttonColors();
	//testLedBrightness();
	//Program should never get here
	return 0;

}

//This is a basic Hello World test of the blinky Heartbeat LED.
void blink (void){
	blinky_led_flash(10000);

}

// This tests the buttons. Any time one of the buttons is pressed the heartbeat LED should flash
void buttonBlink (void){
	uint8 buttonRedOld = 0;
	uint8 buttonGreenOld = 0;
	uint8 buttonBlueOld = 0;
	uint8 buttonRed, buttonGreen, buttonBlue;
	while(1){
		buttonRed = buttons_get(BUTTON_RED);
		buttonGreen = buttons_get(BUTTON_GREEN);
		buttonBlue = buttons_get(BUTTON_BLUE);
		if ((buttonRed & !buttonRedOld) | (buttonGreen & !buttonGreenOld) | (buttonBlue & !buttonBlueOld)){
			blinky_led_flash(30000);
		}
		buttonRedOld = buttonRed;
		buttonGreenOld = buttonGreen;
		buttonBlueOld = buttonBlue;
	}
}

// Testing the colored LEDs.
void buttonColors(void){
	uint8 buttonRedOld = 0;
	uint8 buttonGreenOld = 0;
	uint8 buttonBlueOld = 0;
	uint8 buttonRed, buttonGreen, buttonBlue;
	while(1){
		buttonRed = buttons_get(BUTTON_RED);
		buttonGreen = buttons_get(BUTTON_GREEN);
		buttonBlue = buttons_get(BUTTON_BLUE);
		if (buttonRed & !buttonRedOld){
			ledFlash(LED_RED, 30000);
		}
		if (buttonGreen & !buttonGreenOld){
			ledFlash(LED_GREEN, 30000);
		}
		if (buttonBlue & !buttonBlueOld){
			ledFlash(LED_BLUE, 30000);
		}
		buttonRedOld = buttonRed;
		buttonGreenOld = buttonGreen;
		buttonBlueOld = buttonBlue;
	}
}

// This was a way to see if the resistor values on the board were correct.
void testLedBrightness(void){
	ledsSetAll(1);
	while(1){				// Trap to keep LEDs on
	}
}

