 /*
 * The IR Beacon is used to transmit data via a pulsing infrared LED.
 * 
 * To use this code, all that needs to be done is call ir_beacon_init() once.
 * Whenever data needs to be sent, call ir_beacon_set_data(uint32) with the data
 * to be encoded (between 0 and 255).
 * If transmission needs to be stopped, call ir_beacon_disable(). This will stop
 * the timer (Timer A) from running as well, conserving power. To re-enable the 
 * transmittion system, call ir_beacon_enable().
 * 
 * -Josh
 * 
 * NOTE: When the bottom board is properly connected to the top board, and a line is
 * available to wire the 430 to the IR beacon itself, you need only modify the functions
 * ir_beacon_LED_on() and ir_beacon_LED_off() to power on whichever pin happens to have 
 * been used to connect to the beacon.
 */

#include <stdio.h>
#include <msp430f2132.h>

#include "typeDefs.h"

#ifndef IRBEACON_H_
#define IRBEACON_H_

// Initializer function
void ir_beacon_init();

// Re-enables the timer. Need only be called after ir_beacon_disable() has been called.
void irBeaconEnable(void);

// Used to stop the timer to conserve power. No transmission is possible after caling this
//   function, until ir_beacon_enable() is called.
void irBeaconDisable(void);

// This transmits a passed integer between 0 and 255 through the beacon. When this function is
//   called, the data is sent once, then the transmission stops. For continuous data transmission,
//   repeatedly call this function.
void ir_beacon_set_data(uint32 data);
uint8 irBeaconGetEnable(void);
void irBeaconUpdate(void);

#endif /*IRBEACON_H_*/




