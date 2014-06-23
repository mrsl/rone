#include <stdio.h>
#include <msp430f2132.h>

#ifndef RONE_V12_TILETRACK

#include "typedefs.h"
#include "I2C.h"
#include "Gyroscope.h"

#define I2C_GYRO_ADDRESS		0x68	/*0xD0*/

#define GYRO_CTRL_REG1			0x20
#define GYRO_CTRL_REG2			0x21
#define GYRO_CTRL_REG3			0x22
#define GYRO_CTRL_REG4			0x23
#define GYRO_CTRL_REG5			0x24
#define GYRO_REFERENCE			0x25
#define GYRO_OUT_TEMP			0x26
#define GYRO_STATUS_REG			0x27
#define GYRO_OUT_X_L			0x28
#define GYRO_OUT_X_H			0x29
#define GYRO_OUT_Y_L			0x2A
#define GYRO_OUT_Y_H			0x2B
#define GYRO_OUT_Z_L			0x2C
#define GYRO_OUT_Z_H			0x2D
#define GYRO_FIFO_CTRL_REG		0x2E
#define GYRO_FIFO_SRC_REG		0x2F
#define GYRO_INT1_CFG			0x30
#define GYRO_INT1_SRC			0x31
#define GYRO_INT1_TSH_XH		0x32
#define GYRO_INT1_TSH_XL		0x33
#define GYRO_INT1_TSH_YH		0x34
#define GYRO_INT1_TSH_YL		0x35
#define GYRO_INT1_TSH_ZH		0x36
#define GYRO_INT1_TSH_ZL		0x37
#define GYRO_INT1_DURATION		0x38

#define GYRO_AUTOINC_ADDR_FLAG  0x80

uint8 gyroData[GYRO_DATA_LENGTH];


//Read 6 bytes from the gyro using multiple byte auto-increment
void gyroUpdate(void){
	I2CReceiveArray(I2C_GYRO_ADDRESS, (GYRO_OUT_X_L | GYRO_AUTOINC_ADDR_FLAG), gyroData, GYRO_DATA_LENGTH);
	// swarm the little-endian gyro data for a big-endian world
	endianSwap(&gyroData[0]);
	endianSwap(&gyroData[2]);
	endianSwap(&gyroData[4]);
}


uint16 gyroGetData(uint8 addr) {
	if (addr < GYRO_DATA_LENGTH) {
		return gyroData[addr];
	} else {
		return 0;
	}
}


//Initilizes the Gyroscope
void gyroInit(void){
	// enable sensor readings
	I2CSend(I2C_GYRO_ADDRESS, GYRO_CTRL_REG1, 0x0F);
	
	// Update to give initial values
	gyroUpdate();
}

#endif // RONE_V12_TILETRACK
