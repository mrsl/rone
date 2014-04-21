/*
 * @file accelerometer.c
 * @brief interface functions for 3D accelerometer in the robot
 * @since July 22, 2010
 * @author James McLurkin
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"


#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"


#include "roneos.h"


#define ACCEL_READ_BIT			0x80
#define ACCEL_WRITE_BIT			0x00
#define ACCEL_ADDR_INC_BIT		0x40
#define ACCEL_ADDR_MASK			0xFC

#define ACCEL_CTRL_REG1			0x20
#define ACCEL_CTRL_REG2			0x21
#define ACCEL_CTRL_REG3			0x23
#define ACCEL_STATUS_REG		0x27

#define ACCEL_OUTX_L			0x28
#define ACCEL_OUTX_H			0x29
#define ACCEL_OUTY_L			0x2A
#define ACCEL_OUTY_H			0x2B
#define ACCEL_OUTZ_L			0x2C
#define ACCEL_OUTZ_H			0x2D

#if defined(RONE_V6)
int32 accelerometer_x = 0;
int32 accelerometer_y = 0;
int32 accelerometer_z = 0;


/*
 * @brief Reads the accelerometer register from the specified address.
 *
 * @param addr specifies where the register is
 * @returns the data contained in the register
 */
uint8 accelerometer_read_register(uint8 addr) {
	uint32 buffer;
	SPISelectDevice(SPI_ACCELEROMETER);
	MAP_SSIDataPut(SSI0_BASE, ACCEL_READ_BIT | (addr & 0x3F));
	MAP_SSIDataGet(SSI0_BASE, &buffer);
	MAP_SSIDataPut(SSI0_BASE, 0x00);//dummy read
	MAP_SSIDataGet(SSI0_BASE, &buffer);
	SPIDeselect();
	return (uint8) buffer;
}

/*
 * @brief Writes data into the specified accelerometer register.
 *
 * @param addr specifies where the register is
 * @param data to be written into the register
 * @returns void
 */
void accelerometer_write_register(uint8 addr, uint8 data) {
	uint32 buffer;
	SPISelectDevice(SPI_ACCELEROMETER);
	MAP_SSIDataPut(SSI0_BASE, ACCEL_WRITE_BIT | (addr & 0x3F));
	MAP_SSIDataGet(SSI0_BASE, &buffer);
	MAP_SSIDataPut(SSI0_BASE, data);
	MAP_SSIDataGet(SSI0_BASE, &buffer);
	SPIDeselect();
}

/*
 * private function
 */
int32 accelerometer_read_register_16(uint8 addr) {
	uint32 data_low;
	uint32 data_high;
	int32 value;

	SPISelectDevice(SPI_ACCELEROMETER);
	MAP_SSIDataPut(SSI0_BASE, ACCEL_READ_BIT | ACCEL_ADDR_INC_BIT | (addr & 0x3F));
	MAP_SSIDataGet(SSI0_BASE, &data_low);
	MAP_SSIDataPut(SSI0_BASE, 0x00);
	MAP_SSIDataGet(SSI0_BASE, &data_low);
	MAP_SSIDataPut(SSI0_BASE, 0x00);
	MAP_SSIDataGet(SSI0_BASE, &data_high);
	SPIDeselect();

	value = (int32)((int16)((uint16)data_high << 8) | ((uint16)data_low));
	// need to manually sign extend because this stupid complier does not
	if (value & 0x8000) {
		value |= 0xFFFF0000;
	}
	return value;
}

/*
 * @brief Initializes the accelerometer.
 *
 * Enable power-on, X, Y, Z, channels.
 * @returns void
 */
void accelerometer_init(void) {

	accelerometer_write_register(ACCEL_CTRL_REG1, 0xC7);
}

/*
 * @brief Updates the x, y, and z axes of the accelerometer.
 *
 * @returns void
 */
void accelerometerUpdate(void) {
	accelerometer_x = -accelerometer_read_register_16(ACCEL_OUTX_L);
	accelerometer_y = -accelerometer_read_register_16(ACCEL_OUTY_L);
	accelerometer_z = -accelerometer_read_register_16(ACCEL_OUTZ_L);
	return;
}

/*
 * @brief Gets the value of the specified axis
 *
 * @param axis specifies which axis to look up
 * @returns the value of the axis; 0 if the input parameter is not recognized
 */
int16 accelerometerGetValue(uint32 axis) {
	switch (axis) {
	case ACCELEROMETER_X:
		return accelerometer_x;
	case ACCELEROMETER_Y:
		return accelerometer_y;
	case ACCELEROMETER_Z:
		return accelerometer_z;
	default:
		return 0;
	}
}

#elif defined(RONE_V9)

#define MSP430_MSG_ACCELXLSB_IDX		0
#define MSP430_MSG_ACCELXMSB_IDX		1
#define MSP430_MSG_ACCELYLSB_IDX		2
#define MSP430_MSG_ACCELYMSB_IDX		3
#define MSP430_MSG_ACCELZLSB_IDX		4
#define MSP430_MSG_ACCELZMSB_IDX		5

#define ACCEL_XMSB_IDX					0
#define ACCEL_XLSB_IDX					1
#define ACCEL_YMSB_IDX					2
#define ACCEL_YLSB_IDX					3
#define ACCEL_ZMSB_IDX					4
#define ACCEL_ZLSB_IDX					5

int16 accelX;
int16 accelY;
int16 accelZ;

/*
 * @brief Initialize accelerometer.
 *
 * Set (accelX, accelY, accelZ) to 0.
 * @returns void
 */
void accelerometer_init(void) {
	accelX = 0;
	accelY = 0;
	accelZ = 0;
}


/*
 * @brief Updates the x, y, and z axes of the accelerometer.
 *
 * @returns void
 */
void accelerometerUpdate(uint8 msgIn[]) {
	// deals with accelerometer 12 bit nonsense
	accelX = msgIn[ACCEL_XMSB_IDX];
	accelX <<= 8;
	accelX |= msgIn[ACCEL_XLSB_IDX];
	accelX = accelX >> 4;		// this is a sign extended shift

	accelY = msgIn[ACCEL_YMSB_IDX];
	accelY <<= 8;
	accelY |= msgIn[ACCEL_YLSB_IDX];
	accelY = accelY >> 4;

	accelZ = msgIn[ACCEL_ZMSB_IDX];
	accelZ <<= 8;
	accelZ |= msgIn[ACCEL_ZLSB_IDX];
	accelZ = accelZ >> 4;
}

/*
 * @brief Get accelerometer value on axis.
 *
 * @param axis specifies ACCELOMETER_X, ACCELEROMETER_Y, or ACCELEROMETER_Z.
 * @returns if argument is valid axis, accelerometer value on specified axis; else, 0
 */
int16 accelerometerGetValue(uint32 axis) {
	switch (axis) {
	case ACCELEROMETER_X:
		return accelX;
	case ACCELEROMETER_Y:
		return accelY;
	case ACCELEROMETER_Z:
		return accelZ;
	default:
		return 0;
	}
}
#elif defined(RONE_V12)

#define MSP430_MSG_ACCELXLSB_IDX		0
#define MSP430_MSG_ACCELXMSB_IDX		1
#define MSP430_MSG_ACCELYLSB_IDX		2
#define MSP430_MSG_ACCELYMSB_IDX		3
#define MSP430_MSG_ACCELZLSB_IDX		4
#define MSP430_MSG_ACCELZMSB_IDX		5

#define ACCEL_XMSB_IDX					0
#define ACCEL_XLSB_IDX					1
#define ACCEL_YMSB_IDX					2
#define ACCEL_YLSB_IDX					3
#define ACCEL_ZMSB_IDX					4
#define ACCEL_ZLSB_IDX					5

int16 accelX;
int16 accelY;
int16 accelZ;

/*
 * @brief Initialize accelerometer.
 *
 * Set (accelX, accelY, accelZ) to 0.
 * @returns void
 */
void accelerometer_init(void) {
	accelX = 0;
	accelY = 0;
	accelZ = 0;
}


/*
 * @brief Updates the x, y, and z axes of the accelerometer.
 *
 * @returns void
 */
void accelerometerUpdate(uint8 msgIn[]) {
	// deals with accelerometer 12 bit nonsense
	accelX = msgIn[ACCEL_XMSB_IDX];
	accelX <<= 8;
	accelX |= msgIn[ACCEL_XLSB_IDX];
	accelX = accelX >> 4;		// this is a sign extended shift

	accelY = msgIn[ACCEL_YMSB_IDX];
	accelY <<= 8;
	accelY |= msgIn[ACCEL_YLSB_IDX];
	accelY = accelY >> 4;

	accelZ = msgIn[ACCEL_ZMSB_IDX];
	accelZ <<= 8;
	accelZ |= msgIn[ACCEL_ZLSB_IDX];
	accelZ = accelZ >> 4;
}

/*
 * @brief Get accelerometer value on axis.
 *
 * @param axis specifies ACCELOMETER_X, ACCELEROMETER_Y, or ACCELEROMETER_Z.
 * @returns if argument is valid axis, accelerometer value on specified axis; else, 0
 */
int16 accelerometerGetValue(uint32 axis) {
	switch (axis) {
	case ACCELEROMETER_X:
		return accelX;
	case ACCELEROMETER_Y:
		return accelY;
	case ACCELEROMETER_Z:
		return accelZ;
	default:
		return 0;
	}
}
#endif
