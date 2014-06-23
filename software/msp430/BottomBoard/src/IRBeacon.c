#include <msp430f2132.h>

#include "typedefs.h"
#include "IRBeacon.h"

#ifdef RONE_V11
	#define IR_BEACON_PORT_IN			P1IN
	#define IR_BEACON_PORT_OUT			P1OUT
	#define IR_BEACON_PORT_DIR			P1DIR
	#define IR_BEACON_PORT_SEL			P1SEL
	#define IR_BEACON_BIT				BIT0

const uint32 ircodes[60] = { /* ECC 14->10 code table */
0x00000000, 0x00000197, 0x000002B9, 0x0000032E, 0x000004E5, /* 0-4 */
0x00000572, 0x0000065C, 0x000007CB, 0x00000874, 0x000009E3, /* 5-9 */
0x00000ACD, 0x00000B5A, 0x00000C91, 0x00000D06, 0x00000E28, /* 10-14 */
0x00000FBF, 0x000010E8, 0x0000117F, 0x00001251, 0x000013C6, /* 15-19 */
0x0000140D, 0x0000159A, 0x000016B4, 0x00001723, 0x0000189C, /* 20-24 */
0x0000190B, 0x00001A25, 0x00001BB2, 0x00001C79, 0x00001DEE, /* 25-29 */
0x00001EC0, 0x00001F57, 0x00002047, 0x000021D0, 0x000022FE, /* 30-34 */
0x00002369, 0x000024A2, 0x00002535, 0x0000261B, 0x0000278C, /* 35-39 */
0x00002833, 0x000029A4, 0x00002A8A, 0x00002B1D, 0x00002CD6, /* 40-44 */
0x00002D41, 0x00002E6F, 0x00002FF8, 0x000030AF, 0x00003138, /* 45-49 */
0x00003216, 0x00003381, 0x0000344A, 0x000035DD, 0x000036F3, /* 50-54 */
0x00003764, 0x000038DB, 0x0000394C, 0x00003A62, 0x00003BF5};//, /* 60-64 */


// For use by ir_beacon_update()
uint32 ir_beacon_data = 0;
uint32 ir_beacon_command_timer = 0;

#define IR_BEACON_COMMAND_TIMEOUT	60
uint8 ir_beacon_transmit_bit = 19;
uint8 ir_beacon_transmit_phase = 2;
uint32 ir_beacon_transmit_bits = 0;

// ir_beacon flag
uint8 irBeaconState;

// Function declarations (these are for use only be this file. Do not call externally.
void irBeaconUpdate();
void ir_beacon_LED_on();
void ir_beacon_LED_off();

// Used with the two commented-out lines in ir_beacon_update. Was used to easily scope to check that it was running at 60Hz
//int pulseFlag = 0;


// must enable timer for ir_beacon to work
void ir_beacon_init() {
	// Init with a starting value of 22 for testing.
	ir_beacon_set_data(23);
	// Init with LED off
	IR_BEACON_PORT_SEL &= ~IR_BEACON_BIT; 
	IR_BEACON_PORT_DIR |= IR_BEACON_BIT;
	ir_beacon_LED_off();
}

uint8 irBeaconGetEnable(void) {
	return irBeaconState;
}

void irBeaconEnable(void) {
	irBeaconState = TRUE;
}

void irBeaconDisable(void) {
	irBeaconState = FALSE;
}

void ir_beacon_set_data(uint32 data) {
	ir_beacon_data = data;
	ir_beacon_command_timer = IR_BEACON_COMMAND_TIMEOUT;
}

void ir_beacon_LED_on() {
	IR_BEACON_PORT_OUT |= IR_BEACON_BIT;
}

void ir_beacon_LED_off() {
	IR_BEACON_PORT_OUT &= ~IR_BEACON_BIT;
}

/*
 * This function needs to be called at 60hz.
 * Updates command timer, transmit phase, and transmit bit/bits.
 * Turns IRBeacon LED on or off accordingly
 */

void irBeaconUpdate(void) {
	//#if 0 // comented out to save space - getting odd complier bugs
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
			ir_beacon_LED_on();
			//return;
		} else if (ir_beacon_transmit_bit <= 1) {
			ir_beacon_LED_off();
			//return;
		} else if (ir_beacon_transmit_phase == 0) {
			/* Bits are transmitted 0=100, 1=111 */
			ir_beacon_LED_on();
			//return;
		} else if (((ir_beacon_transmit_bits >> (19 - ir_beacon_transmit_bit))
				& 1) == 1) {
			ir_beacon_LED_on();
		} else {
			ir_beacon_LED_off();
		}
	} else {
		ir_beacon_LED_off();
	}
	//#endif
}

#endif

