/*
 * @file systemIO.c
 *
 * @since Mar 29, 2012
 * @author jamesm
 */

#include "roneos.h"
#include <stdio.h>

uint8 batteryVoltage, usbVoltage, powerButton, mspVersion, mspVersionHardware, prevVal, prevValHardware;

/*
 * @brief Check USBlevel.
 *
 * Unfinished, and returns FALSE unconditionally.
 * @returns FALSE
 */
uint32 systemUSBConnected(void) {
	//TODO put actual code here to check USBlevel
	return FALSE;
}


#if defined(RONE_V6)

void systemIOInit() {
	batteryVoltage = 0;
	usbVoltage = 0;
	powerButton = 0;
	mspVersion = 0;
	prevVal = 0;
}

void systemBatteryVoltageUpdate(uint8 val) {
}

uint8 systemBatteryVoltageGet(void) {
	return 0;
}


void systemUSBVoltageUpdate(uint8 val) {
}

uint8 systemUSBVoltageGet(void) {
	return 0;
}


void systemPowerButtonUpdate(uint8 val) {
}

uint8 systemPowerButtonGet(void) {
	return 0;
}

#endif

#if (defined(RONE_V9) || defined(RONE_V12))

/*
 * @brief Initialize the system IO
 *
 */
void systemIOInit() {
	batteryVoltage = 0;
	usbVoltage = 0;
	powerButton = 0;
	mspVersion = 0;
	prevVal = 0;
}

/*
 * Update the battery voltage.
 *
 * @param val the new voltage value
 * @returns void
 */
void systemBatteryVoltageUpdate(uint8 val) {
	batteryVoltage = val;
}

/*
 * @brief Retrieve the battery voltage.
 *
 * @returns batteryVoltage the battery voltage
 */
float systemBatteryVoltageGet(void) {
	float bv = ((float)batteryVoltage) / 10.0;
	return bv;
}

/*
 * @brief Retrieve the battery voltage and place ones and tenths values in appropriate spaces.
 *
 * @param onesPtr the pointer to where the ones value of the voltage will be stored
 * @param tenthsPtr the pointer to where the tenths value of the voltage will be stored
 * @returns void
 */
void systemBatteryVoltageGet2(uint8* onesPtr, uint8* tenthsPtr) {
	float bv = ((float)batteryVoltage) / 10.0;
	float ones, tenths;

	ones = (float)(int)(bv);
	tenths = (bv - ones) * 10.0;
	*onesPtr = (uint8)ones;
	*tenthsPtr = (uint8)tenths;
}

/*
 * @brief Update the USB voltage.
 *
 * @param val the new USB voltage value
 * @returns void
 */
void systemUSBVoltageUpdate(uint8 val) {
	usbVoltage = val;
}

/*
 * @brief Get the USB voltage.
 *
 * @returns usbVoltage the USB voltage
 */
uint8 systemUSBVoltageGet(void) {
	return usbVoltage;
}

/*
 * @brief Update the power button.
 *
 * @param val the new power button value
 * @returns void
 */
void systemPowerButtonUpdate(uint8 val) {
	powerButton = val;
}

/*
 * @brief Get the power button value.
 *
 * @returns the power button value
 */
uint8 systemPowerButtonGet(void) {
	return powerButton;
}


/*
 *@brief Update the MSP version.
 *
 *@returns void
 */
void systemMSPVersionUpdate(uint8 val) {
	// Attempt to filter out erroneous data
	if((val & 63) == prevVal && (val & 63) != 0){
		mspVersion = (val & 63);
	}
	prevVal = (val  & 63);

	if( (val >> 6) == prevValHardware && (val >> 6) != 0){
		mspVersionHardware = (val >> 6);
	}
	prevValHardware = (val >> 6);
}

/*
 *@brief Get the MSP Hardware version.
 *
 *@returns mspHardwareVersion the MSP version
 */

uint8 systemMSPVersionHardwareGet(void){
	return mspVersionHardware;

}

/*
 *@brief Get the MSP version.
 *
 *@returns mspVersion the MSP version
 */
uint8 systemMSPVersionGet(void) {
	return mspVersion;
}

#endif
