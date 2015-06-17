/*
 * @file systemIO.c
 *
 * @since Mar 29, 2012
 * @author jamesm
 */

#include "roneos.h"
#include <stdio.h>

uint16 voltageBattery, voltageUSB;
uint8 powerButton, mspVersion, mspVersionHardware, prevVal, prevValHardware;
boolean batteryCharging, batteryFastCharging;

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
	voltageBattery = 0;
	voltageUSB = 0;
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
	voltageBattery = 0;
	voltageUSB = 0;
	batteryCharging = FALSE;
	batteryFastCharging = FALSE;
	powerButton = 0;
	mspVersion = 0;
	prevVal = 0;
}

#define SPI_MESSAGE_VOLTAGE_OFFSET	300
#define SPI_MESSAGE_VOLTAGE_SCALER	2

uint16 voltageMessageConvert(uint8 voltageMsg) {
	//TODO check for bounds
	uint16 voltage = (uint16)voltageMsg * SPI_MESSAGE_VOLTAGE_SCALER + SPI_MESSAGE_VOLTAGE_OFFSET;
	return voltage;
}

/*
 * Update the battery voltage.
 *
 * @param val the new voltage value
 * @returns void
 */
void systemBatteryVoltageUpdate(uint8 val) {
	voltageBattery = voltageMessageConvert(val);
}

/*
 * @brief Retrieve the battery voltage.
 *
 * @returns batteryVoltage the battery voltage
 */
float systemBatteryVoltageGet(void) {
	float bv = ((float)voltageBattery) / 100.0;
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
	float bv = ((float)voltageBattery) / 100.0;
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
#define VOLTAGE_USB_CONV_OFFSET		30
#define MSP430_MSG_VUSB_FASTCHARGE_BIT	(1<<7)
#define MSP430_MSG_VUSB_CHARGE_BIT		(1<<6)

void systemUSBVoltageUpdate(uint8 val) {
	// pull ou the charger status bits
	batteryCharging = ((val & MSP430_MSG_VUSB_CHARGE_BIT) ? TRUE : FALSE);
	batteryFastCharging = ((val & MSP430_MSG_VUSB_CHARGE_BIT) ? TRUE : FALSE);

	//mask the charging bits from the battery value
	val &= ~(MSP430_MSG_VUSB_FASTCHARGE_BIT | MSP430_MSG_VUSB_CHARGE_BIT);
	voltageUSB = val + VOLTAGE_USB_CONV_OFFSET;
}

/*
 * @brief Get the USB voltage.
 *
 * @returns usbVoltage the USB voltage
 */
float systemUSBVoltageGet(void) {
	float bv = ((float)voltageUSB) / 10.0;
	return bv;
}

/*
 * @brief Get the USB voltage.
 *
 * @returns boolean of the charge status
 */
boolean systemBatteryChargingGet(void) {
	return batteryCharging;
}
boolean systemBatteryFastChargingGet(void) {
	return batteryFastCharging;
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
