/**
 * @file ir_beacon.h
 *
 * @brief This code controls the 4 IR LEDS on the top center of the rone robot
 * These are the IR_beacons, and an IR sensitive camera can use these to track the robots.
 *
 * The init and preinit functions are typically called by functions in system.c.
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef IR_BEACON_H_
#define IR_BEACON_H_

/******** Defines ********/

#define IR_BEACON_MAX_ID                    1023

/******** Functions ********/

/**
 * 	@brief Initializes IRBeacon.
 *
 * 	Enables the IRBeacon pin as an output. Turns IRBeacon off in the process.
 * 	@returns void
 */
void IRBeaconPreinit(void);


/**
 * 	@brief Initializes IRBeacon interrupt.
 *
 * 	Enables the 60hz IRBeacon interrupt.
 * 	@returns void
 */
void IRBeaconInit(void);


/**
 *	@brief Disables IRBeacon.
 *
 *	Turns off IRBeacon LED and sets the timer to 0.
 *	@returns void
 */
void IRBeaconDisable(void); // python rone

//TODO: this function should be called every second to get continuous localization?
/**
 * 	@brief Sets the data in IRBeacon.
 *
 * 	Sets what the IRBeacon is going to output; also sets the timer for IRBeacon to 60.
 *
 * 	This function, when called "IRBeaconSetData(roneID);"  gives each robot a unique ID.
 * 	@param data the output data (32 bit unsigned int)
 * 	@returns void
 */
void IRBeaconSetData(uint32 data); // python rone

#endif /* IR_BEACON_H_ */
