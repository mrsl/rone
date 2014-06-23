#include <stdio.h>
#include <msp430f2132.h>

#ifndef RONE_V12_TILETRACK

#include "typedefs.h"
#include "I2C.h"
#include "Accelerometer.h"

//7-bit addresses, no R/W bits
#define I2C_ACCEL_ADDRESS   0x1C	/*0x38*/

#define ACCEL_XOUTH    		0x01
#define ACCEL_XOUTL    		0x02
#define ACCEL_YOUTH    		0x03
#define ACCEL_YOUTL    		0x04
#define ACCEL_ZOUTH    		0x05
#define ACCEL_ZOUTL    		0x06
#define ACCEL_CTRL_REG1    	0x2A


uint8 accelData[ACCEL_DATA_LENGTH];


void accelUpdate(void){
    I2CReceiveArray(I2C_ACCEL_ADDRESS, ACCEL_XOUTH, accelData, ACCEL_DATA_LENGTH);
}


//Initilizes the Accerometer
void accelInit(void){
    I2CSend(I2C_ACCEL_ADDRESS, ACCEL_CTRL_REG1, 0x01);		//switch from STANDBY to ACTIVE mode
    
    // Update to give initial data to accelData
    accelUpdate();
}


uint16 accelGetData(uint8 addr) {
	if (addr < ACCEL_DATA_LENGTH) {
		return accelData[addr];
	} else {
		return 0;
	}
}


#endif //#ifndef RONE_V12_TILETRACK
