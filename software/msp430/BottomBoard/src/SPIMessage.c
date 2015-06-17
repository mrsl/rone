#include "typedefs.h"
#include "SPI_8962.h"
#include "SPIMessage.h"
#include "Gyroscope.h"
#include "Accelerometer.h"
#include "RFIDReader.h"
#include "ReflectiveSensors.h"
#include "BumpSensors.h"
#include "IRBeacon.h"
#include "I2C.h"
#include "leds.h"
#include "PowerController.h"
#include "Buttons.h"
#include "msp430.h"
#include "bootStrapLoader.h"
#include "System.h"
#include "VersionNumber.h"

const uint8 MSP430Code[] = {'B', 'O', 'T'};

#define SPI_MESSAGE_VOLTAGE_OFFSET	300
#define SPI_MESSAGE_VOLTAGE_SCALER	2

uint8 voltageMessageConvert(uint16 voltage) {
	//TODO check for bounds
	voltage = (voltage - SPI_MESSAGE_VOLTAGE_OFFSET) / SPI_MESSAGE_VOLTAGE_SCALER;
	return (uint8)voltage;
}

// process messageIn and messageOut from CPU board
void msp430CheckAndUpdate(void) {
	uint8 i;
	uint8 SPIMessageOut[MSP430_MSG_LENGTH];
	uint8 SPIMessageIn[MSP430_MSG_LENGTH];

	if (SPI8962GetMessage(SPIMessageIn)) {
		// Build the next outgoing message as soon as possibe so that
		// there will be data available for the interrupt to send
		for (i = 0; i < MSP430_MSG_LENGTH; i++) {
			SPIMessageOut[i] = 0;
		}

		// Pack code
		for (i = 0; i < MSP430_CODE_LENGTH; i++) {
			SPIMessageOut[i] = MSP430Code[i];
		}

		// Pack payload
#ifdef RONE_V12_TILETRACK
		SPIMessageOut[MSP430_MSG_BUMPER_IDX] = RFIDReaderGet();
		for (i = 0; i < ACCEL_DATA_LENGTH; i++) {
			SPIMessageOut[MSP430_MSG_ACCEL_START_IDX + i] = accelGetData(i);
		}
		// 6 bytes free from gyro
		for (i = 0; i < GYRO_DATA_LENGTH; i++) {
			SPIMessageOut[MSP430_MSG_GYRO_START_IDX + i] = 0;
		}
		for (i = 0; i < NUM_REFLECTIVE_PORTS; i++) {
			SPIMessageOut[MSP430_MSG_REFLECT_START_IDX + i] = reflectiveGetData(i);

		}
#else
		SPIMessageOut[MSP430_MSG_BUMPER_IDX] = bumpSensorGet();
		for (i = 0; i < ACCEL_DATA_LENGTH; i++) {
			SPIMessageOut[MSP430_MSG_ACCEL_START_IDX + i] = accelGetData(i);
		}
		for (i = 0; i < GYRO_DATA_LENGTH; i++) {
			SPIMessageOut[MSP430_MSG_GYRO_START_IDX + i] = gyroGetData(i);
		}
#endif //RONE_V12_TILETRACK

		SPIMessageOut[MSP430_MSG_VBAT_IDX] = voltageMessageConvert(voltageBatGet());
		// convert and pack the USB voltage.  this int will range from 30-65ish.
		// we subtract 30 to free up some room at the top for the charge bits
		uint8 vUSB = voltageUSBGet() * VOLTAGE_USB_CONV_NUMER / VOLTAGE_USB_CONV_DENOM - VOLTAGE_USB_CONV_OFFSET;
		if (voltageBatGet() < VOLTAGE_BAT_FULLY_CHARGED) {
			vUSB |= (voltageUSBGet() > VOLTAGE_USB_PLUGGED_IN_THRESHOLD ? MSP430_MSG_VUSB_CHARGE_BIT : 0);
			vUSB |= (voltageUSBGet() > VOLTAGE_USB_FAST_CHARGE_THRESHOLD ? MSP430_MSG_VUSB_FASTCHARGE_BIT : 0);
		}

		SPIMessageOut[MSP430_MSG_VUSB_IDX] = (voltageUSBGet() > VOLTAGE_USB_PLUGGED_IN_THRESHOLD ? 1 : 0);
		SPIMessageOut[MSP430_MSG_POWER_BUTTON_IDX] = powerButtonGetValue();
		SPIMessageOut[MSP430_MSG_VERSION_IDX] = FULL_ID_VERSION_NUMBER;
		// Pack checksum
		SPIMessageOut[MSP430_MSG_CHECKSUM_IDX] = messageChecksum(SPIMessageOut, MSP430_CODE_LENGTH, MSP430_MSG_LENGTH);

		// Load the message into the SPI Buffer
		SPI8962SetMessage(SPIMessageOut);

		// Update sensor values
		reflectiveSensorsUpdate();
		bumpSensorUpdate();
		accelUpdate();
		gyroUpdate();

		// Process the received message
		if (messageChecksum(SPIMessageIn, MSP430_CODE_LENGTH, MSP430_MSG_LENGTH) == SPIMessageIn[MSP430_CMD_C_CHECKSUM_IDX]) {
			switch(SPIMessageIn[MSP430_CMD_COMMAND_IDX]){
			case MSP430_CMD_COMMAND_NORMAL:
				setSystemLEDRampBrightness(SPIMessageIn[MSP430_CMD_C_SYSTEM_LED_IDX]);
				buttonsUpdate(SPIMessageIn[MSP430_CMD_C_BUTTONS_IDX]);
				ledUpdate(SPIMessageIn + MSP430_CMD_C_LED_IDX);
				ledTimeoutReset();
				break;
			case MSP430_CMD_COMMAND_SHUTDOWN:
				setPowerOffRequest(TRUE);
				break;
			case MSP430_CMD_COMMAND_RESET:
				// Toggle the reset line
				setPowerResetRequest(TRUE);
				setPowerOffRequest(TRUE);
				break;
			case MSP430_CMD_COMMAND_REPROGRAM:
				// This case is used exclusively when the ARM wants to invoke a reprogramming of the MSP430
				startBSL();
				break;
			default:
				setSystemLEDRampBrightness(SPIMessageIn[MSP430_CMD_C_SYSTEM_LED_IDX]);
				buttonsUpdate(SPIMessageIn[MSP430_CMD_C_BUTTONS_IDX]);
				ledUpdate(SPIMessageIn + MSP430_CMD_C_LED_IDX);
				ledTimeoutReset();
				break;
			}
		}
	}
}
