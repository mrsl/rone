/**
 * @file pwm.h
 *
 *  @since Jun 1, 2012
 *  @author Lindsay
 *  @brief This is a PWM module which was originally created for the IR beacon. It is meant to control some PWM
 *       setup, but mostly for setting and changing PWM on the 8962 pins. PWM outputs are used for things like single LEDs and the power
 *      adjustment on the IR beacons.
 */

#ifndef PWM_H_
#define PWM_H_

/* All periods are in 8962 clock cycles
 *  The short period should always look continuous and varying the duty cycle changes the perceived
 *  brightness of one of the RGB LEDs. The long period should result in a visible on/off when
 *  the duty cycle is varied and enable the light to blink.
 *  The super short period is for controlling the power of the IR beacons.
 *  Note that the PWM generator counter is 16 bits.
 */


//#define PERIOD_IRBEACON TODO Need actual number for this. Talk to professor, use math.
#define IR_FREQUENCY			38000
#define	IR_PWM_LEVEL			31
#define PWM_FREQUENCY			(SYSCTL_CLOCK_FREQ / IR_FREQUENCY * IR_PWM_LEVEL)


/**
 * @brief Initializes the PWM pins.
 *  Configures the RGB LEDs to be countdown PWM pins. They are flashed once.
 *  @returns void
 */
void pwmInitializeLed(void);


/**
 * @brief Sets up the PWM pins for the IR beacon PWM control.
 * @returns voids
 */
void pwmInitializeBeacon(void);

/**
* Brief: Sets the duty cycle length for one pin
* period is the current length of the entire PWM period
*
* @param pwm_target is the pin which needs to have its duty cycle changed. In the IR beacon it is colors
* @param duty is the duty cycle for when the pin is ON, and can range from 0 to 100
*
* @returns void
*/
void pwmSetDuty (uint32 pwm_target, uint16 duty);


/**
 * @brief Changes the duty cycle for all of the LED PWM pins
 * period is the current length of the entire PWM period
 * @param duty is the length of the new duty cycle and should be 0-100
 * @returns void
 */
void pwmSetDutyAllLeds(uint16 duty);


/**
 * @brief This enables the output of the given pwm pin.
 * The input argument should be the name of the function of a given PWM pin, i.e. RED_LED
 * @param pin_name - name of pins wanted to change
 * @returns void
 */
void pwmEnableSingle (uint32 pin_name);


/**
 * @brief Enables the pwm output for all 3 LEDs
 * @returns void
 *
 */
void pwmEnableAllLeds(void);


/**
 * @brief This disables the output of the given pwm pins.
 * The input argument should be any logical OR of valid PWM BIT pin names, e.g. RED_LED_PWM_BIT
 * @param pin_name - name of pins wanted to change
 * @returns void
 */
void pwmDisableSingle(uint32 pin_name);


/**
 * @brief This disables the output of the all pwm pins.
 *
 * @returns void
 */
void pwmDisableAllLeds(void);

#endif /* PWM_H_ */
