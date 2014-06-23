#include <stdio.h>
#include <msp430f2132.h>

#ifndef RONE_V12_TILETRACK

#include "typedefs.h"
#include "BumpSensors.h"
//#define BUMP_DEBUG_PINS

uint8 bumpSensorValue;

//#define BUMP_POWERON_DELAY		10
// hack for bump = 1 bug
#if defined(RONE_V11)
#define BUMP_POWERON_DELAY		100

// The v12 robots have a Miller cap feedback loop which requires a longer pulse 
// to turn on the bump sensors
#elif defined (RONE_V12)
#define BUMP_POWERON_DELAY		1200

#endif



void bumpSensorPowerEnable(boolean val) {
	if(val) {
		BUMP_PWR_PORT_DIR |= BUMP_PWR_BIT;		//Set the bump sensor power control to output
		BUMP_PWR_PORT_OUT &= ~BUMP_PWR_BIT;		//Set the output to a 0 (on for PFETs)
	} else {
		#ifdef RONE_V11
			BUMP_PWR_PORT_DIR &= ~BUMP_PWR_BIT;
			// no need to drive the pin high, let the external pull up handle that
			// this prevents a brown-on condition when the 3.3v is turned off 
			//BUMP_PWR_PORT_OUT |= BUMP_PWR_BIT;
		#endif
		#ifdef RONE_V12
			//The V12 version should drive the pin high. Do not rely on the pullup
			BUMP_PWR_PORT_DIR |= BUMP_PWR_BIT;
			BUMP_PWR_PORT_OUT |= BUMP_PWR_BIT;
		#endif
			
	}
}


void bumpSensorInit() {
	BUMPA_PORT_DIR &= ~BUMPA_BITS;		//configure pins to be inputs
	BUMPA_PORT_SEL &= ~BUMPA_BITS;		//configure pins to be GPIO
	BUMPB_PORT_DIR &= ~BUMPB_BITS;		//configure pins to be inputs
	BUMPB_PORT_SEL &= ~BUMPB_BITS;		//configure pins to be GPIO
	
	bumpSensorPowerEnable(FALSE);
	BUMP_PWR_PORT_SEL	&= ~BUMP_PWR_BIT; 
	bumpSensorUpdate();
} 


//TODO this seems to fix the bump sensor always = 1 bug, but I think this is a message bug, not a i/o. bug
void bumpSensorHackFix() {
	BUMPA_PORT_DIR &= ~BUMPA_BITS;		//configure pins to be inputs
	BUMPA_PORT_SEL &= ~BUMPA_BITS;		//configure pins to be GPIO
	BUMPB_PORT_DIR &= ~BUMPB_BITS;		//configure pins to be inputs
	BUMPB_PORT_SEL &= ~BUMPB_BITS;		//configure pins to be GPIO
} 


void bumpSensorUpdate() {
	uint16 i;
	
	bumpSensorHackFix();
	bumpSensorPowerEnable(TRUE);
	for (i = 0; i < BUMP_POWERON_DELAY; ++i) {}
	uint8 bumpA = ~(BUMPA_PORT_IN);
	uint8 bumpB = ~(BUMPB_PORT_IN);
	bumpSensorPowerEnable(FALSE);
	bumpSensorValue = (bumpA & BUMPA_BITS) | ((bumpB & BUMPB_BITS) >> 2);
}


uint8 bumpSensorGet() {
	return bumpSensorValue;
}


void bumpDebugInit() {
	#ifdef BUMP_DEBUG_PINS
	BUMPA_PORT_DIR = 0x03;				//configure pins to be outputs
	BUMPA_PORT_SEL &~ BUMPA_BITS;		//configure pins to be GPIO
	#endif //BUMP_DEBUG_PINS
} 

void bumpDebugSet(uint8 val) {
	#ifdef BUMP_DEBUG_PINS
	BUMPA_PORT_OUT |= val;				//configure pins to be outputs
	#endif //BUMP_DEBUG_PINS
} 

void bumpDebugClear(uint8 val) {
	#ifdef BUMP_DEBUG_PINS
	BUMPA_PORT_OUT &= ~val;				//configure pins to be outputs
	#endif //BUMP_DEBUG_PINS
} 


#endif //#ifndef RONE_V12_TILETRACK

