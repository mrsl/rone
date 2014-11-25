/*
 * pwmTest.c
 *
 *  Created on: Jun 3, 2012
 *      Author: Lindsay
 *
 *      This tests the pwm module in roneos and the parts of leds which use it.
 */

#include "roneos.h"
#include "ronelib.h"

#define RED 		0
#define GREEN 		1
#define BLUE 		2

#define INITIAL_DUTY		0
#define DELAY				0xFFFF

void ledOn(uint32 color);
void pwmRampSingle(uint32 color, uint32 period);
void pwmRampAll(uint32 period);
void testDuty(uint32 color, uint32 period, uint32 duty);



int main(void) {
	uint16 count = 0;
	systemPreInit();
	systemInit();
	systemPrintStartup();
	cprintf("I am an IR beacon\r\n");
	//cprintf("\n");

	while(1){
		//pwmRampSingle(GREEN);
		//pwmRampAll(PWM_FREQUENCY);
		testDuty(GREEN, PWM_FREQUENCY, 50);
		testDuty(GREEN, PWM_FREQUENCY, 5);
		pwmDisableAllLeds();
		//systemDelay(DELAY);
/*		if (count == 99){
			cprintf("I am an IR beacon \n");
			count = 0;
		}
		count ++;*/
	}

	while(1){
	}

	// Should never get here
	return 0;
}

// Checks to see if an LED can be turned on.
void ledOn(uint32 color) {
	pwmSetDuty(color, 100);
	while (1){ //TRAP

	}

}

// This tests the PWM functions for a single LED to make sure the pinouts and general code are correct.
void pwmRampSingle(uint32 color, uint32 period) {
	uint32 duty;
	uint8 delay_count;
	duty = INITIAL_DUTY;
	pwmEnableSingle(color);
	while (duty <= 100){
		ledsSetSingle(color, duty);
		systemDelay(DELAY);
		for (delay_count = 0; delay_count < 3; delay_count++){
			systemDelay(DELAY);
		}
		duty += 2;
	}

}

//This tests the pwm functions for all of the LEDs at once to see if they are synchronizing well
void pwmRampAll(uint32 period){
	uint32 duty;
	uint8 delay_count;
	duty = INITIAL_DUTY;
	pwmEnableAllLeds();
	while (duty <= 100){

		pwmSetDutyAllLeds(duty);
		for (delay_count = 0; delay_count < 3; delay_count++){
			systemDelay(DELAY);
		}
		duty += 2;
	}

}

void testDuty(uint32 color, uint32 period, uint32 duty){
	pwmEnableSingle(color);
	pwmSetDuty(color, duty);

}
