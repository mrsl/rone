/*
 * @file ir_beacon.c
 *
 * @brief This code controls the 4 IR LEDS on the top center of the rone robot (these are the IR_beacons, and an IR sensitive camera can use these to track the robots).
 *
 *  The init and preinit functions are typically called by functions in system.c
 *
 *  TODO:  list the functions (and the calling order) that must be included in any main file in order to use the IRbeacons.
 *
 * @since Jul 22, 2010
 * @author jamesm
 */
#include <string.h>

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "driverlib/debug.h"
#include "driverlib/pwm.h"

#include "roneos.h"
#include "swarmCamLookupTable.h"

#if defined(RONE_V6)
#define IR_BEACON_PERIPH 		SYSCTL_PERIPH_GPIOB
#define IR_BEACON_BASE 			GPIO_PORTB_BASE
#define IR_BEACON_PIN 			GPIO_PIN_5

#elif defined(RONE_V12)
// TODO - The V12 IR beacon stuff may eventually want to use PWM functionality
#define IR_BEACON_PERIPH 		SYSCTL_PERIPH_GPIOE
#define IR_BEACON_BASE 			GPIO_PORTE_BASE
#define IR_BEACON_PIN 			GPIO_PIN_1
#endif

// The beacon needs to blink at exactly 60 Hz
#define IR_BEACON_CLOCK_SPEED		60
#define IR_BEACON_COMMAND_TIMEOUT	60

uint32 ir_beacon_data = 0;
uint32 ir_beacon_command_timer = 0;

uint8 ir_beacon_transmit_bit = 19;
uint8 ir_beacon_transmit_phase = 2;
uint32 ir_beacon_transmit_bits = 0;


/*
 * 	@brief Turns the IRBeacon LED on.
 *
 * 	@returns void
 */
static void IRBeaconLEDOn(void) {
#if defined(RONE_V6)
	GPIOPinWrite(IR_BEACON_BASE, IR_BEACON_PIN, IR_BEACON_PIN);
#elif defined(RONE_V12)
	taskENTER_CRITICAL();
	MAP_GPIOPinWrite(IR_BEACON_BASE, IR_BEACON_PIN, IR_BEACON_PIN);
	taskEXIT_CRITICAL();
#endif
}

/*
 * 	@brief Turns the IRbeacon LED off.
 *
 * 	@returns void
 */
static void IRBeaconLEDOff(void) {
#if defined(RONE_V6)
	GPIOPinWrite(IR_BEACON_BASE, IR_BEACON_PIN, 0);
#elif defined(RONE_V12)
	taskENTER_CRITICAL();
	MAP_GPIOPinWrite(IR_BEACON_BASE, IR_BEACON_PIN, 0);
	taskEXIT_CRITICAL();
#endif
}


/*
 * 	@brief Initializes IRBeacon.
 *
 * 	Enables the IRBeacon pin as an output. Turns IRBeacon off in the process.
 * 	@returns void
 */
void IRBeaconPreinit(void) {
	#if defined(RONE_V6) || defined(RONE_V12)
	MAP_SysCtlPeripheralEnable(IR_BEACON_PERIPH);
	MAP_GPIOPinTypeGPIOOutput(IR_BEACON_BASE, IR_BEACON_PIN);
	IRBeaconLEDOff();
	#endif
}


/*
 * 	@brief Initializes IRBeacon interrupt.
 *
 * 	Enables the 60hz IRBeacon interrupt.
 * 	@returns void
 */
void IRBeaconInit(void) {
	#if defined(RONE_V6) || defined(RONE_V12)
	// Configure and enable a 60Hz clock on timer 2 for use with the IR Beacon
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_32_BIT_PER);
	//	MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()
	//			/ IR_BEACON_CLOCK_INTERVAL * 4 /*Hz*/);
	MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, SYSCTL_CLOCK_FREQ / IR_BEACON_CLOCK_SPEED);
	MAP_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	MAP_TimerEnable(TIMER2_BASE, TIMER_A);

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
	MAP_IntPrioritySet(INT_TIMER2A, SYSTEM_INTERRUPT_PRIORITY);
	IRBeaconDisable();
	#endif
}


/*
 * @brief Enables ir_beacon clock.
 *
 * @returns void
 *
 * Currently unused
 */
void IRBeaconIntEnable() {
	#if defined(RONE_V6) || defined(RONE_V12)
	MAP_IntEnable(INT_TIMER2A);
	#endif
}

/*
 * @brief Disables ir_beacon clock.
 *
 * @returns void
 *
 * Currently unused
 */
void IRBeaconIntDisable() {
	#if defined(RONE_V6) || defined(RONE_V12)
	MAP_IntDisable(INT_TIMER2A);
	#endif
}


/*
 * 	@brief Sets the data in IRBeacon.
 *
 * 	Sets what the IRBeacon is going to output; also sets the timer for IRBeacon to 60.
 *
 * 	This function, when called "IRBeaconSetData(roneID);"  gives each robot a unique ID.
 * 	TODO: this function should be called every second to get continuous localization?
 *
 * 	@param data the output data (32 bit unsigned int)
 * 	@returns void
 */
void IRBeaconSetData(uint32 data) {
	ir_beacon_data = data;
	#if defined(RONE_V6) || defined(RONE_V12)
	ir_beacon_command_timer = IR_BEACON_COMMAND_TIMEOUT;
	IRBeaconIntEnable();
	#endif
}


/*
 *	@brief Disables IRBeacon.
 *
 *	Turns off IRBeacon LED and sets the timer to 0.
 *	@returns void
 */
void IRBeaconDisable(void) {
	IRBeaconIntDisable();
	#if defined(RONE_V6) || defined(RONE_V12)
	ir_beacon_command_timer = 0;
	IRBeaconLEDOff();
	#endif
}


#if defined(RONE_V6) || defined (RONE_V12)
/*
 * @brief Updates IRBeacon.
 *
 * This function needs to be called at 60hz.
 * Updates command timer, transmits phase, and transmits bit/bits.
 * Turns IRBeacon LED on or off accordingly.
 *
 * NOTE: this code was provided by NewtonLabs
 * @returns void
 */
static void IRBeaconUpdate(void) {
	if (ir_beacon_command_timer > 0) {
		ir_beacon_command_timer--; //updates the timer
		if (ir_beacon_transmit_phase < 2) {
			ir_beacon_transmit_phase++; //updates the phase
		} else {
			ir_beacon_transmit_phase = 0;
			if (ir_beacon_transmit_bit < 19) {
				ir_beacon_transmit_bit++; //update which
			} else {
				ir_beacon_transmit_bit = 0;
				ir_beacon_transmit_bits = ircodes[ir_beacon_data]; //gets the right code..?
			}
		}
		/* Framing = 110000 */
		if (ir_beacon_transmit_bit == 0 && ir_beacon_transmit_phase <= 1) {
			IRBeaconLEDOn();
			//return;
		} else if (ir_beacon_transmit_bit <= 1) {
			IRBeaconLEDOff();
			//return;
		} else if (ir_beacon_transmit_phase == 0) {
			/* Bits are transmitted 0=100, 1=111 */
			IRBeaconLEDOn();
			//return;
		} else if (((ir_beacon_transmit_bits >> (19 - ir_beacon_transmit_bit))
				& 1) == 1) {
			IRBeaconLEDOn();
		} else {
			IRBeaconLEDOff();
		}
	} else {
		IRBeaconLEDOff();
	}
}


/*
 * @brief Interrupt function
 *
 * Called when Timer2 goes off (@ 60hz (16.6667 ms))
 *
 *
 */
void IRBeaconIRQHandler(void) {
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IRBeaconUpdate();
}
#endif





/*
 * @brief This temporary ir_beacon_update method was used to test the frequency with which
 * it was called (Easily scopeable).
 *
 * Originally, it was being called at 16ms intervals, which is not
 * the 60Hz that is needed to sync with the Newton Labs tracking system setup.
 */

//void ir_beacon_update(void) {
//	// Used to test frequency of this function's calling (confirmed to be ~16ms
//	static int irState = 1;
//	if (irState == 1) {
//		ir_beacon_LED_on();
//		irState = 1 - irState;
//	} else {
//		ir_beacon_LED_off();
//		irState = 1 - irState;
//	}
//}
