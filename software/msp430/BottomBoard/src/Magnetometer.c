
#include <stdio.h>
#include <msp430f2132.h>

#ifdef RONE_V12_TILETRACK

#include "typedefs.h"
#include "I2C.h"
#include "Magnetometer.h"

#define I2C_MAG_LEFT_ADDRESS		0x1D
#define I2C_MAG_RIGHT_ADDRESS		0x1E

#define MAG_TEMP_OUT_L		0x05
#define MAG_TEMP_OUT_H		0x06
#define MAG_STATUS_M		0x07
#define MAG_OUT_X_L_M		0x08
#define MAG_OUT_X_H_M		0x09
#define MAG_OUT_Y_L_M		0x0A
#define MAG_OUT_Y_H_M		0x0B
#define MAG_OUT_Z_L_M		0x0C
#define MAG_OUT_Z_H_M		0x0D
#define MAG_WHO_AM_I		0x0F
#define MAG_INT_CTRL_M		0x12
#define MAG_INT_SRC_M		0x13
#define MAG_INT_THS_L_M		0x14
#define MAG_INT_THS_H_M		0x15
#define MAG_OFFSET_X_L_M	0x16
#define MAG_OFFSET_X_H_M	0x17
#define MAG_OFFSET_Y_L_M	0x18
#define MAG_OFFSET_Y_H_M	0x19
#define MAG_OFFSET_Z_L_M	0x1A
#define MAG_OFFSET_Z_H_M	0x1B
#define MAG_REFERENCE_X		0x1C
#define MAG_REFERENCE_Y		0x1D
#define MAG_REFERENCE_Z		0x1E
#define MAG_CTRL0			0x1F
#define MAG_CTRL1			0x20
#define MAG_CTRL2			0x21
#define MAG_CTRL3			0x22
#define MAG_CTRL4			0x23
#define MAG_CTRL5			0x24
#define MAG_CTRL6			0x25
#define MAG_CTRL7			0x26
#define MAG_STATUS_A		0x27
#define MAG_OUT_X_L_A		0x28
#define MAG_OUT_X_H_A		0x29
#define MAG_OUT_Y_L_A		0x2A
#define MAG_OUT_Y_H_A		0x2B
#define MAG_OUT_Z_L_A		0x2C
#define MAG_OUT_Z_H_A		0x2D
#define MAG_FIFO_CTRL		0x2E
#define MAG_FIFO_SRC		0x2F
#define MAG_IG_CFG1			0x30
#define MAG_IG_SRC1			0x31
#define MAG_IG_THS1			0x32
#define MAG_IG_DUR1			0x33
#define MAG_IG_CFG2			0x34
#define MAG_IG_SRC2			0x35
#define MAG_IG_THS2			0x36
#define MAG_IG_DUR2			0x37
#define MAG_CLICK_CFG		0x38
#define MAG_CLICK_SRC		0x39
#define MAG_CLICK_THS		0x3A
#define MAG_TIME_LIMIT		0x3B
#define MAG_TIME_LATENCY	0x3C
#define MAG_TIME_WINDOW		0x3D
#define MAG_ACT_THS			0x3E
#define MAG_ACT_DUR			0x3f

#define MAG_AUTOINC_ADDR_FLAG  0x80

static uint8 magDataLeft[MAG_DATA_LENGTH];
static uint8 magDataRight[MAG_DATA_LENGTH];

//Read 6 bytes from the gyro using multiple byte auto-increment
void magUpdate(void){
	// removed code to save space
	//if (magPresent) {
		I2CReceiveArray(I2C_MAG_LEFT_ADDRESS, (MAG_OUT_X_L_M | MAG_AUTOINC_ADDR_FLAG), magDataLeft, MAG_DATA_LENGTH);
		// swarm the little-endian gyro data for a big-endian world
		endianSwap(&magDataLeft[0]);
		endianSwap(&magDataLeft[2]);
		endianSwap(&magDataLeft[4]);

		I2CReceiveArray(I2C_MAG_RIGHT_ADDRESS, (MAG_OUT_X_L_M | MAG_AUTOINC_ADDR_FLAG), magDataRight, MAG_DATA_LENGTH);
		// swarm the little-endian gyro data for a big-endian world
		endianSwap(&magDataRight[0]);
		endianSwap(&magDataRight[2]);
		endianSwap(&magDataRight[4]);
	//}
}


uint8 magGetDataLeft(uint8 addr) {
	// removed code to save space
	//if (addr < MAG_DATA_LENGTH) {
		return magDataLeft[addr];
	//} else {
	//	return 0;
	//}
}


uint8 magGetDataRight(uint8 addr) {
	// removed code to save space
	//if (addr < MAG_DATA_LENGTH) {
		return magDataRight[addr];
	//} else {
	//	return 0;
	//}
}


void magInitOne(uint8 address) {
	// removed code to save space
	//check to see if the magnetometer is present
	I2CReceiveArray(address, MAG_WHO_AM_I, magDataLeft, 2);
	if (magDataLeft[0] == 0x49) {
		//magPresent = TRUE;

		// high res, 25hz updates, no int latch
		I2CSend(address, MAG_CTRL5, 0x68);

		// +/- 12 gauss full scale
		I2CSend(address, MAG_CTRL6, 0x60);

		// normal HPF, continuous conversion
		I2CSend(address, MAG_CTRL7, 0x00);
	}
}


//Initializes the Magnetometer
void magInit(void){
	magInitOne(I2C_MAG_LEFT_ADDRESS);
	magInitOne(I2C_MAG_RIGHT_ADDRESS);

	// Update to give initial values
	magUpdate();
}

#endif // #ifdef RONE_V12_TILETRACK
