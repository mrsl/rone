/*
 * @file pwm.c
 *
 * @since Jun 1, 2012
 * @author Lindsay
 *
 * @brief This is a PWM module which was originally created for the IR beacon. It is meant to control some PWM
 *       setup, but mostly for setting and changing PWM on the 8962 pins. PWM outputs are used for things like single LEDs and the power
 *      adjustment on the IR beacons.
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

#include "roneos.h"

// The PWM module is not used in the RONE builds, so all of the functions have been left empty
#if (defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12))

void pwmInitializeLed(void) {
}

void pwmInitializeBeacon(void) {
}

void pwmSetDuty (uint32 pwm_target, uint16 duty){
}

void pwmSetDutyAllLeds(uint16 duty){
}

void pwmEnableSingle (uint32 pin_name){
}

void pwmEnableAllLeds(void) {
}

void pwmDisableSingle(uint32 pin_name) {
}

void pwmDisableAllLeds(void) {
}

#elif defined(RONE_V9)

void pwmInitializeLed(void){
}

void pwmInitializeBeacon(void) {
}

void pwmSetDuty (uint32 pwm_target, uint16 duty) {
}

void pwmSetDutyAllLeds(uint16 duty) {
}

void pwmEnableSingle (uint32 pin_name){
}

void pwmEnableAllLeds(void) {
}

void pwmDisableSingle(uint32 pin_name) {
}

void pwmDisableAllLeds(void) {
}

#elif defined(RONE_V12)

void pwmInitializeLed(void){
}

void pwmInitializeBeacon(void) {
}

void pwmSetDuty (uint32 pwm_target, uint16 duty) {
}

void pwmSetDutyAllLeds(uint16 duty) {
}

void pwmEnableSingle (uint32 pin_name){
}

void pwmEnableAllLeds(void) {
}

void pwmDisableSingle(uint32 pin_name) {
}

void pwmDisableAllLeds(void) {
}


#elif defined(RONE_IRBEACON)

// PWM base - this is from the memory map
#define PWM_BASE			0x40028000

// PWM generators
#define GEN_RED_BEACON		PWM_GEN_0
#define GEN_GREEN_BLUE		PWM_GEN_1

#define RED_LED				0
#define RED_LED_PERIPH 		SYSCTL_PERIPH_GPIOG
#define RED_LED_BASE 		GPIO_PORTG_BASE
#define RED_LED_PWM			PWM_OUT_1
#define RED_LED_PWM_BIT		PWM_OUT_1_BIT

#define GREEN_LED			1
#define GREEN_LED_PERIPH 	SYSCTL_PERIPH_GPIOB
#define GREEN_LED_BASE 		GPIO_PORTB_BASE
#define GREEN_LED_PWM		PWM_OUT_2
#define GREEN_LED_PWM_BIT	PWM_OUT_2_BIT

#define BLUE_LED			2
#define BLUE_LED_PERIPH 	SYSCTL_PERIPH_GPIOB
#define BLUE_LED_BASE 		GPIO_PORTB_BASE
#define BLUE_LED_PWM		PWM_OUT_3
#define BLUE_LED_PWM_BIT	PWM_OUT_3_BIT

#define IR_BEACON_PERIPH	SYSCTL_PERIPH_GPIOF
#define IR_BEACON_BASE		GPIO_PORTF_BASE
#define	IR_BEACON_PWM		PWM_OUT_0
#define IR_BEACON_PWM_BIT	PWM_OUT_0_BIT

//To be used until the PWM is reliable enough at high frequencies.
#define IR_BEACON_GPIO_BIT	GPIO_PIN_0

/*
 * Brief: Initializes the PWM pins.
 *
 *  Configures the RGB LEDs to be countdown PWM pins. They are flashed once.
 */
void pwmInitializeLed(void) {


	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	// Configures the red LED, which is on the same PWM generator as the beacons
	MAP_PWMGenEnable(PWM_BASE, GEN_RED_BEACON);
	MAP_PWMGenConfigure(PWM_BASE, GEN_RED_BEACON, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_DBG_RUN);
	MAP_PWMGenPeriodSet(PWM_BASE, GEN_RED_BEACON, PWM_FREQUENCY);
	MAP_PWMIntEnable(PWM_BASE, GEN_RED_BEACON);

	// Configures the green and blue LEDs
	MAP_PWMGenEnable(PWM_BASE, GEN_GREEN_BLUE);
	MAP_PWMGenConfigure(PWM_BASE, GEN_GREEN_BLUE, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_DBG_RUN);
	MAP_PWMGenPeriodSet(PWM_BASE, GEN_GREEN_BLUE, PWM_FREQUENCY);
	MAP_PWMIntEnable(PWM_BASE, GEN_GREEN_BLUE);

}

/*
 * Sets up the PWM pins for the IR beacon PWM control.
 */
void pwmInitializeBeacon(void) {
	MAP_SysCtlPeripheralEnable(IR_BEACON_PERIPH);
	//MAP_GPIOPinWrite(IR_BEACON_BASE, IR_BEACON_GPIO_BIT, 0);
	MAP_GPIOPinTypeGPIOOutput(IR_BEACON_BASE, IR_BEACON_GPIO_BIT);
	MAP_GPIOPinWrite(IR_BEACON_BASE, IR_BEACON_GPIO_BIT, IR_BEACON_GPIO_BIT);
}

/*
 * Brief: Sets the duty cycle length for one pin
 *
 * pwm_target is the pin which needs to have its duty cycle changed. In the IR beacon it is colors
 * period is the current length of the entire PWM period
 * duty is the duty cycle for when the pin is ON, and can range from 0 to 100
 */
void pwmSetDuty (uint32 pwm_target, uint16 duty) {

	// Declarations
	uint8 MAX_DUTY_CYCLE = 100;
	uint32 duty_clock_cycles;
	uint32 target_base = 0;
	uint32 target_pin = 0;
	uint32 current_period;
	uint32 current_pulse_width;
	// Checking boundary conditions
	if (duty > MAX_DUTY_CYCLE){
		duty = MAX_DUTY_CYCLE;
	}
	// For some reason 0 doesn't actually work and it acts like 100 in driverlib code.
	if (duty == 0){
		pwmDisableSingle(pwm_target);
		return;
	}

	// Picking which pin to change
	switch (pwm_target){
	case RED_LED:
		target_pin = RED_LED_PWM;
		break;
	case GREEN_LED:
		target_pin = GREEN_LED_PWM;
		break;
	case BLUE_LED:
		target_pin = BLUE_LED_PWM;
		break;
	}

	// Finds the duty cycle as a fraction of the period's clock cycles
	duty_clock_cycles = PWM_FREQUENCY * duty / MAX_DUTY_CYCLE;

	// Sets the target pin's new duty cycle
	MAP_PWMPulseWidthSet(PWM_BASE, target_pin, duty_clock_cycles);
	pwmEnableSingle(pwm_target);
}

/*
 * Changes the duty cycle for all of the LED PWM pins
 * period is the current length of the entire PWM period
 * duty is the length of the new duty cycle and should be 0-100
 */
void pwmSetDutyAllLeds(uint16 duty) {
	pwmSetDuty(RED_LED, duty);
	pwmSetDuty(GREEN_LED, duty);
	pwmSetDuty(BLUE_LED, duty);
}

/*
 * This enables the output of the given pwm pin.
 * The input argument should be the name of the function of a given PWM pin, i.e. RED_LED
 */
void pwmEnableSingle(uint32 pin_name){
	uint32 pin;
	switch (pin_name){
	case RED_LED:
		pin = RED_LED_PWM_BIT;
		break;
	case GREEN_LED:
		pin = GREEN_LED_PWM_BIT;
		break;
	case BLUE_LED:
		pin = BLUE_LED_PWM_BIT;
		break;
	}
	MAP_PWMOutputState(PWM_BASE, pin, true);
}

// Enables the pwm output for all 3 LEDs
void pwmEnableAllLeds(void) {
	MAP_PWMOutputState(PWM_BASE, RED_LED_PWM_BIT| GREEN_LED_PWM_BIT | BLUE_LED_PWM_BIT, true);
}
/*
 * This disables the output of the given pwm pins.
 * The input argument should be any logical OR of valid PWM BIT pin names, e.g. RED_LED_PWM_BIT
 */
void pwmDisableSingle(uint32 pin_name){
	uint32 pin;
	switch (pin_name){
	case RED_LED:
		pin = RED_LED_PWM_BIT;
		break;
	case GREEN_LED:
		pin = GREEN_LED_PWM_BIT;
		break;
	case BLUE_LED:
		pin = BLUE_LED_PWM_BIT;
		break;
	}
	MAP_PWMOutputState(PWM_BASE, pin, false);
}

void pwmDisableAllLeds(void) {
	MAP_PWMOutputState(PWM_BASE, RED_LED_PWM_BIT| GREEN_LED_PWM_BIT | BLUE_LED_PWM_BIT, false);
}

#endif

