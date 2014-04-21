/**
 * @file sd_card.c
 * @authors Jeremy Hunt and Nathan Alison
 * @brief Functions that control the SD card.
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_qei.h"

#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"

#include "roneos.h"
#include "../src/System/spiInternal.h"


#if (defined(RONE_V9) || defined(RONE_V12))

	uint8 cardType;
	uint8 sdCardStatus = SD_CARD_STATUS_NOINIT;
	uint32 sdCardFrequency = SD_CARD_DEFAULT_FREQUENCY;


	uint32 SDCardGetFrequency(void){
		return sdCardFrequency;
	}

	/**
	 * Transmit a byte to SD via SPI
	 */
	void SDCardXmitSPI(uint8 dat) {
		uint32 rcvdat;
		MAP_SSIDataPut(SSI0_BASE, dat); /* Write the data to the tx fifo */
		MAP_SSIDataGet(SSI0_BASE, &rcvdat); /* flush data read during the write */
	}

	/**
	 * Transmit a byte to SD via SPI without selecting it. Sent to nothing.
	 */
	void SDCardXmitSPINoSS(uint8 dat) {
		SPIConfigureDevice(SPI_SDCARD);
		SPIXmitNullByte(dat);
	}

	/**
	 * Deselects the SPI line for the SD card command, but sends a byte after
	 * driving Slave Select High and clocking 1 Byte of data out. This is to
	 * maintain compatibility with having multiple slave devices on a single
	 * SPI bus. SD cards do not like this because they think that they are
	 * deselected at the end of each byte.
	 * Tip found here: http://elm-chan.org/docs/mmc/mmc_e.html in the
	 * Consideration on Multi-slave Configuration section.
	 */
	void SDCardSPIDeselect(void) {
		//SPIConfigureDevice(SPI_SDCARD);
		SPIDeselectSynchronous();
	}

	/**
	 * Receive a byte from SD via SPI
	 */
	uint8 SDCardRecvSPI(void) {
		uint32 rcvdat;
		MAP_SSIDataPut(SSI0_BASE, 0xFF); /* Write dummy data */
		MAP_SSIDataGet(SSI0_BASE, &rcvdat); /* read data from rx fifo */
		return (uint8) rcvdat;
	}

	/**
	 * Transmits 0xFF bytes until the line is idle (reads 0xFF).
	 * Will timeout after SD_CARD_CMD_TIMEOUT bytes.
	 *
	 * @returns boolean TRUE if line is idle, FALSE if timeout occurred.
	 *
	 */
	boolean SDCardWaitIdle(boolean selectCard){
		uint32 j;
		uint32 i = 0;
		uint8 tmp = 0x00;
		do{
			// Select the card if asked
			if(selectCard) {
				SPISelectDevice(SPI_SDCARD);
			}

			j = 0;
			do {
				tmp = SDCardRecvSPI();
				i++;
				j++;
			} while(j < 10 && tmp != 0xFF);

			//Deselect the SD Card if asked to.
			if(selectCard) {
				SDCardSPIDeselect();
				if(tmp != 0xFF){
					osTaskDelay(1);
				}
			}

		}while(i < SD_CARD_BUSY_TIMEOUT && tmp != 0xFF);

		if(i >= SD_CARD_BUSY_TIMEOUT){
			return FALSE;
		}

		return TRUE;
	}

	/**
	 * Synchronizes the SD card with byte aligned values for bad cards.
	 * Essentially clocks bits out one at a time until a 0 is read.
	 *
	 * @param firstBytePtr A pointer to where the first byte of data read will
	 *  be stored after synchronization.
	 * @param fast A boolean value that is true if the function will attempt to
	 *  clock as fast as possible.
	 *
	 * @returns boolean FALSE if the synchronization timed out, TRUE otherwise.
	 */
	static boolean SDCardSyncronizeSPI(uint8 * firstBytePtr, boolean fast) {
		/*
		 * This is incredibly stupid, but some cards (especially Kingston
		 * MicroSD cards) do not byte-align their responses. Essentially,
		 * we just need to clock a random number of bits out (bit bang)
		 * until we see a response or timeout. Here are some Internet infos
		 * on the problem:
		 * http://www.gossamer-threads.com/lists/linux/kernel/1044895
		 * Confirmed with our logic analyzer and oscilloscope in the MRSL.
		 *
		 * This needs to be bit banged because the SPI driver does not support
		 * unaligned SPI (and really shouldn't, goddamned SD cards violate
		 * SPI spec).
		 */

		uint8 tmp;
		uint16 i = 0;
		volatile uint16 j = 0;
		uint8 readBitVal = 0;

		//Wait until the current transmission is done.
		while (MAP_SSIBusy(SSI0_BASE)) {
			i++;
			if (i > SPI_MAX_XFER_IRQ_DELAY) {
				return FALSE;
			}
		}

		// Make the SPI channel into GPIO so that it can be bit banged
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_CLK_PIN));
		MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, SPI_MISO_PIN);

		// Drive the MOSI Pin High to Write all 1's to the SD card
		MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_MOSI_PIN, SPI_MOSI_PIN);

		// Start with the clock low
		MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, 0);

		// Clock out bits until a 0 is received (0 marks the start of a SPI SD card Message).
		tmp = 0;
		i = 0;
		readBitVal = 0;
		if(fast){
			do {
				// Clock high and read the input
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, SPI_CLK_PIN);
				readBitVal = MAP_GPIOPinRead(GPIO_PORTA_BASE, SPI_MISO_PIN);

				// Clock low
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, 0);

				i++;
			} while(readBitVal == SPI_MISO_PIN && i < (SD_CARD_CMD_TIMEOUT*8));
		} else {
			do {
				for(j=0; j<SD_CARD_SYNCHRONIZE_WAIT; j++) {}

				// Clock high and read the input
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, SPI_CLK_PIN);
				readBitVal = MAP_GPIOPinRead(GPIO_PORTA_BASE, SPI_MISO_PIN);
				for(j=0; j<SD_CARD_SYNCHRONIZE_WAIT; j++) {}

				// Clock low
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, 0);

				i++;
			} while(readBitVal == SPI_MISO_PIN && i < (SD_CARD_CMD_TIMEOUT*8));
		}
		// Return failure if there was no response
		if (i >= (SD_CARD_CMD_TIMEOUT*8)) {
			// Revert to hardware control of the SSI lines and re-enable the pull-ups
			// configure SSI pins for peripheral control
			MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN));
			// TODO: Make sure this works and figure out a way to make it be pulldowns on V0-V11 and Pullups on V12+ -Jeremy
			// Also, it may be better to just get rid of the pullups/downs entirely. -Jeremy
			MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN), GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
			return FALSE;
		}

		// Read the rest of that byte (7 bits) into tmp so that the line is fully synchronized
		if(fast) {
			for(i = 0; i<7; i++){
				// Clock high and read the input, shifting it into tmp
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, SPI_CLK_PIN);
				if(MAP_GPIOPinRead(GPIO_PORTA_BASE, SPI_MISO_PIN)) {
					tmp = (tmp << 1) | 0x01;
				} else {
					tmp = (tmp << 1);
				}

				// Clock low
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, 0);
			}
		} else {
			for(i = 0; i<7; i++){
				for(j=0; j<SD_CARD_SYNCHRONIZE_WAIT; j++) {}

				// Clock high and read the input, shifting it into tmp
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, SPI_CLK_PIN);
				if(MAP_GPIOPinRead(GPIO_PORTA_BASE, SPI_MISO_PIN)) {
					tmp = (tmp << 1) | 0x01;
				} else {
					tmp = (tmp << 1);
				}
				for(j=0; j<SD_CARD_SYNCHRONIZE_WAIT; j++) {}

				// Clock low
				MAP_GPIOPinWrite(GPIO_PORTA_BASE, SPI_CLK_PIN, 0);
			}
			for(j=0; j<SD_CARD_SYNCHRONIZE_WAIT; j++) {}
		}

		// Revert to hardware control of the SSI lines and re-enable the pull-ups
		// configure SSI pins for peripheral control
		MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN));
		// TODO: Make sure this works and figure out a way to make it be pulldowns on V0-V11 and Pullups on V12+ -Jeremy
		// Also, it may be better to just get rid of the pullups/downs entirely. -Jeremy
		MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, (SPI_MOSI_PIN | SPI_MISO_PIN | SPI_CLK_PIN), GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

		// Load up the data and return success
		*firstBytePtr = tmp;
		return TRUE;
	}


	/**
	 * Receive a data packet from SD card. The CRC is not checked but sent
	 * back.
	 *
	 * @param buff A pointer to the buffer where the data will be put.
	 * @param blockLen the length of the block to be read in in bytes.
	 * @param crcPtr A pointer to the place where the 16 bit CCITT CRC will be stored.
	 * @param littleEndian A boolean that describes how data will be read into the buffer. TRUE indicates MSB in byte (blockLen-1), FALSE indicates MSB in byte 0.
	 *
	 * @returns boolean TRUE if the block was read successfully, FALSE otherwise.
	 */
	boolean SDCardRecvDatablock(uint8 * buff, uint16 blockLen, uint16 * crcPtr, boolean littleEndian) {
		uint8 tmp = 0x00;
		uint16 i;

		//You have to read in at least 1 byte.
		if (blockLen == 0) {
			return FALSE;
		}

		// Read bytes until the read data token is received or
		// we timeout or there is an error token
		i = 0;
		do {
			tmp = SDCardRecvSPI();
		} while(tmp != SD_CARD_READ_TOKEN && !((tmp & 0xC0) == 0x00 && (tmp & 0x01) == 0x01) && i < SD_CARD_READ_TIMEOUT);

		// Was there an error or a timeout?
		if (((tmp & 0xC0) == 0x00 && (tmp & 0x01) == 0x01) ||
			i >= SD_CARD_READ_TIMEOUT) {
			return FALSE; // Return with Failure
		}

		if(littleEndian){
			// Read in the datablock LSB in byte 0
			i = blockLen;
			do {
				*buff = SDCardRecvSPI();
				buff += 1;
				i--;
			} while (i > 0);
		} else {
			// Read in the datablock MSB in byte 0
			i = blockLen;
			do {
				i--;
				*(buff + i) = SDCardRecvSPI();
			} while (i > 0);
		}


		// Read in the CRC
		uint16 CRC;
		CRC = (uint16)SDCardRecvSPI();
		CRC = (CRC << 8) | (uint16)SDCardRecvSPI();
		*crcPtr = CRC;

		// Success!
		return TRUE;
	}

	/**
	 * Sent a data packet to SD card.
	 */
	boolean SDCardXmitDatablock(const uint8 *buff, uint8 tokenToXmit) {
		uint16 i;
		uint8 response;

		// Wait until the previous write or read finished
		if(!SDCardWaitIdle(FALSE)){
			return FALSE;
		}

		// Transmit the token
		SDCardXmitSPI(tokenToXmit);

		if(tokenToXmit == SD_CARD_WRITE_BLOCK_TOKEN || tokenToXmit == SD_CARD_WRITE_DATA_TOKEN){
			// Transmit the datablock
			i=0;
			do {
				SDCardXmitSPI(buff[i]);
				i++;
			} while(i < SD_CARD_SECTOR_LEN);

			// Transmit a dummy CRC
			// TODO: Look into making this a real CRC
			SDCardXmitSPI(0xFF);
			SDCardXmitSPI(0xFF);

			// Receive the response byte
			response = SDCardRecvSPI();
			if((response & 0x1F) != SD_CARD_WRITE_RESPONSE_OK){
				return FALSE;
			}
		}

		return TRUE; /* Return with Success */
	}

	/**
	 * Send a command packet to SD. Also alows for stupid cards which are not byte-aligned in their
	 * responses. Attempts several retries if the first attempt fails.
	 * @param cmd Command byte
	 * @param responseType The type of SD command (R1, R1B, R2, R3), also determines what length response to get.
	 * @param argument Argument
	 * @param response A pointer to the array where the response will be stored.
	 * @param selectCard A boolean value stating wether or not to do the card selection.
	 *
	 * @returns boolean TRUE if the command send succeed at least once, FALSE otherwise
	 */
	boolean SDCardSendCmd (uint8 cmd, uint8 responseType, uint32 argument, uint8 * response, boolean selectCard) {
		boolean retValue = false;
		uint8 i = 0;

		// Select the card if asked
		if(selectCard) {
			SPISelectDevice(SPI_SDCARD);
		}

		// Try to send the command multiple times
		do {
			retValue = SDCardSendCmdOnce(cmd, responseType, argument, response, FALSE);
			i++;
		} while(!retValue && i < SD_CARD_CMD_RETRY_TIMEOUT);

		if(i >= SD_CARD_CMD_RETRY_TIMEOUT){
			//Deselect the SD Card if asked to.
			if(selectCard) {
				SDCardSPIDeselect();
			}
			return FALSE;
		}

		//Deselect the SD Card if asked to.
		if(selectCard) {
			SDCardSPIDeselect();
		}
		return TRUE;
	}

	/**
	 * @brief Send an application command packet to SD. First sends the application
	 * switch command (55) and then sends the application command. Also allows
	 * for stupid cards which are not byte-aligned in their
	 * responses. Attempts several retries if the first attempt fails.
	 * @param acmd Command byte
	 * @param responseType The type of SD command (R1, R1B, R2, R3), also determines what length response to get.
	 * @param argument Argument
	 * @param response A pointer to the array where the response will be stored.
	 * @param selectCard if TRUE, selects the card
	 *
	 * @returns boolean TRUE if the command send succeed at least once, FALSE otherwise
	 */
	boolean SDCardSendAcmd (uint8 acmd, uint8 responseType, uint32 argument, uint8 * response, boolean selectCard) {
		boolean retValue = false;
		uint8 cmd55Response[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
		uint8 i = 0;

		// Select the card if asked
		if(selectCard) {
			SPISelectDevice(SPI_SDCARD);
		}

		do {
			retValue = SDCardSendCmdOnce(SD_CARD_CMD55, SD_CARD_CMD55_R, 0x00, cmd55Response, FALSE);
			/*
			 * If there was no response, try again, but if the response is error,
			 * return that there was a response, but use the CMD55 response
			 */
			if(retValue){
				if((cmd55Response[0] & 0x7E) == SD_CARD_R1_OK) {
					retValue = SDCardSendCmdOnce(acmd, responseType, argument, response, FALSE);
				} else {
					//Deselect the SD Card if asked to.
					if(selectCard) {
						SDCardSPIDeselect();
					}
					return TRUE;
				}
			}
			i++;
		} while(!retValue && i < SD_CARD_CMD_RETRY_TIMEOUT);

		if(i >= SD_CARD_CMD_RETRY_TIMEOUT){
			//Deselect the SD Card if asked to.
			if(selectCard) {
				SDCardSPIDeselect();
			}
			return FALSE;
		}

		//Deselect the SD Card if asked to.
		if(selectCard) {
			SDCardSPIDeselect();
		}
		return TRUE;
	}

	/**
	 * @brief Send a command packet to SD. Also alows for stupid cards which are not byte-aligned in their
	 * responses.
	 * @param cmd Command byte
	 * @param responseType The type of SD command (R1, R1B, R2, R3), also determines what length response to get.
	 * @param argument Argument
	 * @param response A pointer to the array where the response will be stored.
	 * @param selectCard if TRUE, selects the card
	 *
	 * @returns boolean TRUE if the command send succeed, FALSE otherwise
	 */
	boolean SDCardSendCmdOnce (uint8 cmd, uint8 responseType, uint32 argument, uint8 * response, boolean selectCard) {
		int16 i;
		uint8 tmp;
		uint8 crc = 0x95; // Default CRC is for CMD0 since there is no CRC in SPI mode
		uint8 responseLength = 0;

		// Select the card if asked
		if(selectCard) {
			SPISelectDevice(SPI_SDCARD);
		}

		/* Send the header/command */
		/* Format:
		cmd[7:6] : 01
		cmd[5:0] : command */
		/* Send command packet. */
		// Command
		SDCardXmitSPI((cmd & 0x3F) | 0x40);
		// Arguments
		SDCardXmitSPI((uint8)(argument >> 24)); // arg[31..24]
		SDCardXmitSPI((uint8)(argument >> 16)); // arg[23..16]
		SDCardXmitSPI((uint8)(argument >> 8)); // arg[15..8]
		SDCardXmitSPI((uint8)argument); // arg[7..0]
		// CRC
		if (cmd == SD_CARD_CMD0) { crc = 0x95; } // CRC for CMD0 (0)
		if (cmd == SD_CARD_CMD8) { crc = 0x87; } // CRC for CMD8 (0x1AA)
		SDCardXmitSPI(crc);

		// Receive command response
		switch (responseType) {
			case SD_CARD_R1:
			case SD_CARD_R1B:
				responseLength = 1;
				break;
			case SD_CARD_R2:
				responseLength = 2;
				break;
			case SD_CARD_R3:
			case SD_CARD_R7:
				responseLength = 5;
				break;
			default:
				responseLength = 1;
				break;
		}

		// The byte after a CMD12 (STOP_TRANSMISSION) may be part of data, and
		// is a unknown byte unrelated to the command response. Skip it.
		if(cmd == SD_CARD_CMD12) {
			tmp = SDCardRecvSPI();
			tmp = 0x00;
		}

		/*
		 * This is incredibly stupid, but some cards (especially Kingston
		 * MicroSD cards) do not byte-align their responses. Essentially,
		 * we just need to clock a random number of bits out (bit bang)
		 * until we see a response or timeout. Here are some Internet infos
		 * on the problem:
		 * http://www.gossamer-threads.com/lists/linux/kernel/1044895
		 * Confirmed with our logic analyzer and oscilloscope in the MRSL.
		 *
		 * This needs to be bit banged because the SPI driver does not support
		 * unaligned SPI (and really shouldn't, goddamned SD cards violate
		 * SPI spec).
		 *
		 * Wait for a response. A response can be recognized by the
		 * start bit (a zero). tmp is also loaded up with the first byte
		 * of response.
		 */
		// Also, move faster if the sdCardFrequency is greater than the initialization frequency.
		if(!SDCardSyncronizeSPI(&tmp, (sdCardFrequency > SD_CARD_INIT_FREQUENCY))){
			//Deselect the SD Card if asked to.
			if(selectCard) {
				SDCardSPIDeselect();
			}
			//Return failure if there was no response
			return FALSE;
		}

		// Read the rest of the bytes
		for (i = responseLength-1; i>=0; i--) {
			response[i] = tmp;
			/* This handles the trailing-byte requirement. */
			tmp = SDCardRecvSPI();
		}

		//Deselect the SD Card if asked to.
		if(selectCard) {
			SDCardSPIDeselect();
		}
		return TRUE;
	}

	/*
	 * Helper function for SDCardInit(). Initialized V2 SD Cards
	 */
	static uint8 SDCardV2Init(uint8 *tmpCardTypePtr){
		uint16 i;
		uint32 OCRRegister;
		uint8 response[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

		// Get the Operations Condition Register
		if (!SDCardSendCmd(SD_CARD_CMD58, SD_CARD_CMD58_R, 0x00, response, TRUE)) {
			sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}
		//Was there a Bad Response to CMD58
		if (response[4] != SD_CARD_R1_IDLE){
			sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}
		OCRRegister = (((uint32)response[3]) << 24) | (((uint32)response[2]) << 16) | (((uint32)response[1]) << 8) | ((uint32)response[0]);
		// Does this card support the necessary voltage range?
		if((OCRRegister & SD_CARD_MIN_VOLTAGE_RANGE) != SD_CARD_MIN_VOLTAGE_RANGE){
			sdCardStatus |= (SD_CARD_STATUS_NOT_SUPPORTED | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}

		// Actually Initialize the Card:
		// Wait until the card comes out of idle. (Idle bit clear)
		// Error out if there is a problem.
		i = 0;
		do {
			i++;
			// Send the initialization command ACMD41 enabling the High Capacity Support
			if (!SDCardSendAcmd(SD_CARD_ACMD41, SD_CARD_ACMD41_R, SD_CARD_HCS, response, TRUE)) {
				sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}
			// Check for response errors
			if((response[0] & 0x7E) != SD_CARD_R1_OK){
				sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}
		} while (response[0] != SD_CARD_R1_OK && i < SD_CARD_IDLE_WAIT_TIMEOUT);

		// Did the initialization time out?
		if (i >= SD_CARD_IDLE_WAIT_TIMEOUT) {
			sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}

		// Get the Operations Condition Register
		if (!SDCardSendCmd(SD_CARD_CMD58, SD_CARD_CMD58_R, 0x00, response, TRUE)) {
			sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}
		//Was there a Bad Response to CMD58
		if (response[4]!= 0x00){
			sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}
		OCRRegister = (((uint32)response[3]) << 24) | (((uint32)response[2]) << 16) | (((uint32)response[1]) << 8) | ((uint32)response[0]);

		// Is the card fully on and initialized?
		if ((OCRRegister & SD_CARD_OCD_POWER_ON_BIT) != SD_CARD_OCD_POWER_ON_BIT) {
			//Card is not on...
			sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}

		// Is the card a SDSC_V2 or a SDHC/SDXC
		if ((OCRRegister & SD_CARD_OCD_CCS_BIT) != SD_CARD_OCD_CCS_BIT) {
			//SDSC V2 Cards
			*tmpCardTypePtr = SD_CARD_TYPE_V2_SDSC;
		} else {
			//SDHC and SDXC type cards
			*tmpCardTypePtr = SD_CARD_TYPE_V2_SDHC_SDXC;
		}

		// If you are here, then the initialization succeeded! Return the status
		return sdCardStatus;
	}

	/*
	 * Helper function for SDCardInit()
	 */
	static uint8 SDCardInitCmdSequence(uint8 *tmpCardTypePtr){
		uint16 i;
		uint8 response[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

		// Clock out 74+ clocks with the SD card selected before sending
		// CMD0 to help the card. Out of SD card spec, but seems to help.
		// Commented out because sending another CMD0 also works
		//for(i=0; i<8; i++){
		//	SDCardXmitSPI(0xFF);
		//}

		// Send a CMD0 and don't expect anything from it. This makes the
		// communication more reliable when the card first turns on,
		// and makes sure that the card will always enter SPI mode.
		SDCardSendCmd(SD_CARD_CMD0, SD_CARD_CMD0_R, 0x00, response, TRUE);

		// Try to software reset the card and put it in idle state
		boolean didRespond = FALSE;
		i = 0;
		do {
			didRespond = SDCardSendCmd(SD_CARD_CMD0, SD_CARD_CMD0_R, 0x00, response, TRUE);
			i++;
		} while(i < SD_CARD_INIT_TIMEOUT && !didRespond);
		if (i >= SD_CARD_INIT_TIMEOUT) {
			sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}

		// Check for a valid Idle response
		if (response[0] != SD_CARD_R1_IDLE) {
			sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}

		// Test if the Card is SDC Ver2+ (SDHC and SDXC)
		// Also, if the card is Ver2+, tell it that the current operating voltage
		// is 2.7-3.6V, and make sure it responds correctly.
		if (!SDCardSendCmd(SD_CARD_CMD8, SD_CARD_CMD8_R, 0x1AA, response, TRUE)) {
			sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}
		//Card Idle and command accepted, probably a V2 SD Card
		if (response[4] == SD_CARD_R1_IDLE){
			//Check the Echo Command
			if(response[0] != 0xAA) { //Bad Response to CMD8
				sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}

			//Check card voltage support
			if((response[1] & 0x0F) != 0x01) {
				// The card does not support 2.7-3.6V range... Bad...
				sdCardStatus |= (SD_CARD_STATUS_NOT_SUPPORTED | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}

			//Initialize the card as a V2 card
			if(SDCardV2Init(tmpCardTypePtr) != SD_CARD_STATUS_NOINIT) {
				//There was an error initializing the card, end and return the status
				return sdCardStatus;
			}

		} else if ((response[4] & SD_CARD_R1_ILLEGAL_CMD) == SD_CARD_R1_ILLEGAL_CMD){

			// Get the Operations Condition Register
			if (!SDCardSendCmd(SD_CARD_CMD58, SD_CARD_CMD58_R, 0x00, response, TRUE)) {
				sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}

			//Was there a Bad Response to CMD58? If so, the card is not supported (probably MMC).
			if (response[4] != SD_CARD_R1_IDLE){
				if((response[4] & SD_CARD_R1_ILLEGAL_CMD) == SD_CARD_R1_ILLEGAL_CMD) {
					sdCardStatus |= (SD_CARD_STATUS_NOT_SUPPORTED | SD_CARD_STATUS_NOINIT);
					return sdCardStatus;
				} else {
					// Something actually went wrong and the card isn't SD or an unsupported card
					sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
					return sdCardStatus;
				}
			}
			uint32 OCRRegister = (response[3] << 24) | (response[2] << 16) | (response[1] << 8) | response[0];

			// Does this card support the necessary voltage range?
			if((OCRRegister & SD_CARD_MIN_VOLTAGE_RANGE) != SD_CARD_MIN_VOLTAGE_RANGE){
				sdCardStatus |= (SD_CARD_STATUS_NOT_SUPPORTED | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}

			// Actually Initialize the Card:
			// Wait until the card comes out of idle.
			i = 0;
			do {
				i++;

				// Send the initialization command ACMD41 enabling the High Capacity Support
				if (!SDCardSendAcmd(SD_CARD_ACMD41, SD_CARD_ACMD41_R, SD_CARD_HCS, response, TRUE)) {
					sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
					return sdCardStatus;
				}
				// Did the card recognize the command?
				if ((response[0] & SD_CARD_R1_ILLEGAL_CMD) == SD_CARD_R1_ILLEGAL_CMD) {
					// If Not, the Card is not supported (Probably MMC). End this loop
					i = SD_CARD_IDLE_WAIT_TIMEOUT;
					sdCardStatus |= (SD_CARD_STATUS_NOT_SUPPORTED | SD_CARD_STATUS_NOINIT);
					return sdCardStatus;
				}
			} while ((response[0] & SD_CARD_R1_IDLE) == SD_CARD_R1_IDLE && i < SD_CARD_IDLE_WAIT_TIMEOUT);

			// Did the initialization time out?
			if (i >= SD_CARD_IDLE_WAIT_TIMEOUT) {
				sdCardStatus |= (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				return sdCardStatus;
			}

			// The card turned out to be a valid SDSC V1 Card. Set it as such
			*tmpCardTypePtr = SD_CARD_TYPE_V1_SDSC;


		} else { //Something went terribly wrong...
			sdCardStatus |= (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
			return sdCardStatus;
		}

		// If you are here, the card initialized! Return the Card Status
		return sdCardStatus;
	}

	/*
	 * Reads the raw CSD as 16 bytes. Returns false if it fails to read it.
	 */
	boolean SDCardReadCSD(uint8 * CSDRaw, boolean selectCard) {
		uint8 response[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

		// Select the Device
		if(selectCard) {
			SPISelectDevice(SPI_SDCARD);
		}

		// Tell the card to send the CSD
		if (!SDCardSendCmd(SD_CARD_CMD9, SD_CARD_CMD9_R, 0x00, response, FALSE)){
			sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			SDCardSPIDeselect();
			return FALSE;
		}
		// Check for response errors
		if ((response[0] & 0x7E) != 0) {
			sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
			SDCardSPIDeselect();
			return FALSE;
		}

		//Receive the 16 Byte Data Block CSD
		//Do this backwards because the SD card Spec is backwards and wrong....
		// Again...
		uint16 recivedCRC = 0;
		if(!SDCardRecvDatablock(CSDRaw, 16, &recivedCRC, FALSE)){
			sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
			SDCardSPIDeselect();
			return FALSE;
		}

		//Deselect the SD Card
		if(selectCard) {
			SDCardSPIDeselect();
		}

		//Check the CRC
		uint16 calculatedCRC = crcCCITTCalculate(CSDRaw, 16, FALSE);
		if(recivedCRC != calculatedCRC){
			return FALSE;
		}

		return TRUE;
	}

	/*
	 * Reads the CSD on V1.0 versions of the SD card
	 */
	boolean SDCardProcessCSDSDV1(uint8 * CSDRaw, SDCardCSDSDV1 *CSDStruct){
		//Check the structure bits to make sure that it is a V1 CSD
		if ((CSDRaw[15] & 0xC0) != 0x00) {
			return FALSE;
		}

		// Parse the CSD into the struct
		CSDStruct->CSD_STRUCTURE = 		(uint8)((CSDRaw[15] & 0xC0) >> 6);
		CSDStruct->TAAC = 				(uint8)(CSDRaw[14]);
		CSDStruct->NSAC = 				(uint8)(CSDRaw[13]);
		CSDStruct->TRAN_SPEED = 		(uint8)(CSDRaw[12]);
		CSDStruct->CCC =				(uint16)((((uint16)CSDRaw[11] << 8) | ((uint16)CSDRaw[10] & 0xF0)) >> 4);
		CSDStruct->READ_BL_LEN = 		(uint8)(CSDRaw[10] & 0x0F);
		CSDStruct->READ_BL_PARTIAL = 	(boolean)((CSDRaw[9] & 0x80) >> 7);
		CSDStruct->WRITE_BLK_MISALIGN = (boolean)((CSDRaw[9] & 0x40) >> 6);
		CSDStruct->READ_BLK_MISALIGN = 	(boolean)((CSDRaw[9] & 0x20) >> 5);
		CSDStruct->DSR_IMP =			(boolean)((CSDRaw[9] & 0x10) >> 4);
		CSDStruct->C_SIZE =				(uint16)( (((uint16)CSDRaw[9] & 0x03) << 10) | (((uint16)CSDRaw[8]) << 2) | (((uint16)CSDRaw[7] & 0xC0) >> 6) );
		CSDStruct->VDD_R_CURR_MIN = 	(uint8)((CSDRaw[7] & 0x38) >> 3);
		CSDStruct->VDD_R_CURR_MAX = 	(uint8)(CSDRaw[7] & 0x07);
		CSDStruct->VDD_W_CURR_MIN = 	(uint8)((CSDRaw[6] & 0xE0) >> 5);
		CSDStruct->VDD_W_CURR_MAX = 	(uint8)((CSDRaw[6] & 0x1C) >> 2);
		CSDStruct->C_SIZE_MULT = 		(uint8)(((CSDRaw[6] & 0x03) << 1) | ((CSDRaw[5] & 0x80) >> 7));
		CSDStruct->ERASE_BLK_EN = 		(uint8)((CSDRaw[5] & 0x40) >> 6);
		CSDStruct->SECTOR_SIZE =		(uint8)(((CSDRaw[5] & 0x3F) << 1) | ((CSDRaw[4] & 0x80) >> 7));
		CSDStruct->WP_GRP_SIZE = 		(uint8)(CSDRaw[4] & 0x7F);
		CSDStruct->WP_GRP_ENABLE = 		(boolean)((CSDRaw[3] & 0x80) >> 7);
		CSDStruct->R2W_FACTOR = 		(uint8)((CSDRaw[3] & 0x1C) >> 2);
		CSDStruct->WRITE_BL_LEN = 		(uint8)((CSDRaw[3] & 0x03) << 2) | ((CSDRaw[2] & 0xC0) >> 6);
		CSDStruct->WRITE_BL_PARTIAL = 	(boolean)((CSDRaw[2] & 0x20) >> 5);
		CSDStruct->FILE_FORMAT_GRP =	(uint8)((CSDRaw[1] & 0x80) >> 7);
		CSDStruct->COPY =				(boolean)((CSDRaw[1] & 0x40) >> 6);
		CSDStruct->PERM_WRITE_PROTECT = (boolean)((CSDRaw[1] & 0x20) >> 5);
		CSDStruct->TMP_WRITE_PROTECT = 	(boolean)((CSDRaw[1] & 0x10) >> 4);
		CSDStruct->FILE_FORMAT = 		(uint8)((CSDRaw[1] & 0xC0) >> 2);
		CSDStruct->CRC = 				(uint8)((CSDRaw[0] & 0xFE) >> 1);

		// Done!
		return TRUE;
	}

	/*
	 * Reads the CSD on V2.0 versions of the SD card
	 */
	boolean SDCardProcessCSDSDV2(uint8 * CSDRaw, SDCardCSDSDV2 *CSDStruct){
		//Check the structure bits to make sure that it is a V2 CSD
		if ((CSDRaw[15] & 0xC0) != 0x40) {
			return FALSE;
		}

		// Parse the CSD into the struct
		CSDStruct->CSD_STRUCTURE = 		(uint8)((CSDRaw[15] & 0xC0) >> 6);
		CSDStruct->TRAN_SPEED = 		(uint8)(CSDRaw[12]);
		CSDStruct->CCC =				(uint16)((((uint16)CSDRaw[11] << 8) | ((uint16)CSDRaw[10] & 0xF0)) >> 4);
		CSDStruct->DSR_IMP =			(uint8)((CSDRaw[9] & 0x10) >> 4);
		CSDStruct->C_SIZE =				(uint32)((((uint32)CSDRaw[8] & 0x3F) << 16) | ((uint32)CSDRaw[7] << 8) | (uint32)CSDRaw[6]);
		CSDStruct->COPY =				(uint8)((CSDRaw[1] & 0x40) >> 6);
		CSDStruct->PERM_WRITE_PROTECT = (uint8)((CSDRaw[1] & 0x20) >> 5);
		CSDStruct->TMP_WRITE_PROTECT = 	(uint8)((CSDRaw[1] & 0x10) >> 4);
		CSDStruct->CRC = 				(uint8)((CSDRaw[0] & 0xFE) >> 1);

		// Done!
		return TRUE;
	}

	uint8 SDCardInit(void) {
		uint16 i;
		uint8 CSDRaw[16];
		uint8 response[5];

		//Make the Card un-initialized
		sdCardStatus = SD_CARD_STATUS_NOINIT;

		//Set the SPI Clock to 400kHz to maintain compatibility with all cards
		sdCardFrequency = SD_CARD_INIT_FREQUENCY;
		// Clock out 96 clock cycles.
		SPIXmitNullBytes(SPI_SDCARD, 12);

		// Start the initialization
		uint8 tmpCardType = SD_CARD_TYPE_UNKNOWN;

		// Send the initialization commands and set the card type
		if (SDCardInitCmdSequence(&tmpCardType) != SD_CARD_STATUS_NOINIT) {
			//Something went wrong
			return sdCardStatus;
		}

		// Update the card status and up the frequency
		cardType = tmpCardType;
		if (cardType == SD_CARD_TYPE_UNKNOWN) {
			sdCardStatus |= SD_CARD_STATUS_NOINIT;
			//cprintf("SD Initialization Failed...\n");
			//cprintf("SD Card Initialization Status: 0x%02X\n", sdCardStatus);
			return sdCardStatus;
		}

		// Initialization succeeded! Hurrah!
		sdCardStatus &= ~SD_CARD_STATUS_NOINIT; // Mark the card as initialized
		// Go fast. Really fast.
		sdCardFrequency = SD_CARD_FAST_FREQUENCY;

		// Print out some info about the card
		if(sdCardStatus != SD_CARD_STATUS_OK) {
			//cprintf("SD Initialization Failed...\n");
			//cprintf("SD Card Initialization Status: 0x%02X\n", sdCardStatus);
			return sdCardStatus;
		}

		//cprintf("SD Card Status: OK\n");

		// Print out the card type
//		if (cardType == SD_CARD_TYPE_V1_SDSC) {
//			cprintf("SDSC Ver. 1 Card Found.\n");
//		} else if (cardType == SD_CARD_TYPE_V2_SDSC) {
//			cprintf("SDSC Ver. 2+ Card Found.\n");
//		} else if (cardType == SD_CARD_TYPE_V2_SDHC_SDXC) {
//			cprintf("SDHC or SDXC Ver. 2+ Card Found.\n");
//		} else {
//			cprintf("Card Type Unknown. Very Bad...\n");
//		}

		// Set the read and write block len to 512 if it is an SDSC card
		if (cardType == SD_CARD_TYPE_V1_SDSC || cardType == SD_CARD_TYPE_V2_SDSC) {
			// Tell the card to change the block len
			if (!SDCardSendCmd(SD_CARD_CMD16, SD_CARD_CMD16_R, SD_CARD_SECTOR_LEN, response, TRUE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				return FALSE;
			}
			// Check for response errors
			if (response[0] != SD_CARD_R1_OK) {
				sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				return FALSE;
			}
		}

		// Finish up with the card
		return sdCardStatus;
	}

	uint8 SDCardGetType(void) {
		return cardType;
	}

	uint8 SDCardGetStatus(void) {
		return sdCardStatus;
	}

	/**
	 * @brief Read sector(s).
	 * @param buff Pointer to the data buffer to store read data.
	 * @param sector Start sector number (LBA). For SDSC cards this is the byte address.
	 * @param numSectors The number of 512 byte sectors to be read (1..255).
	 *
	 * @retuns
	 */
	SDCardResult SDCardRead(uint8 * buff, uint32 sector, uint8 numSectors) {
		uint8 response[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
		uint16 recivedCRC = 0;

		// Basic error checking and error if the card isn't initialized.
		if (numSectors < 1) {
			return SD_CARD_RESULT_PARERR;
		}
		if (sdCardStatus != SD_CARD_STATUS_OK) {
			return SD_CARD_RESULT_NOTRDY;
		}

		// Translate the address to byte address for SDSC cards
		if (cardType == SD_CARD_TYPE_V1_SDSC || cardType == SD_CARD_TYPE_V2_SDSC) {
			sector *= SD_CARD_SECTOR_LEN;
		}

		// Single block read
		if (numSectors == 1) {
			//Finish any previous writes or reads
			if(!SDCardWaitIdle(TRUE)){
				return SD_CARD_RESULT_ERROR;
			}

			SPISelectDevice(SPI_SDCARD);

			// Tell the card to send the block
			if (!SDCardSendCmd(SD_CARD_CMD17, SD_CARD_CMD17_R, sector, response, FALSE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// Check for response errors
			if (response[0] != SD_CARD_R1_OK) {
				sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			//Receive the 512 Byte Data Block CSD
			if(!SDCardRecvDatablock(buff, SD_CARD_SECTOR_LEN, &recivedCRC, TRUE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			SDCardSPIDeselect();

			//Check the CRC
			uint16 calculatedCRC = crcCCITTCalculate(buff, SD_CARD_SECTOR_LEN, TRUE);
			//cprintf("Received CRC: %04X, Calculated CRC: %04X\n", recivedCRC, calculatedCRC);
			if(recivedCRC != calculatedCRC){
				//cprintf("CRC Error...\n");
				sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_BAD_CRC;
			}
		} else {
			//Finish any previous writes or reads
			if(!SDCardWaitIdle(TRUE)){
				return SD_CARD_RESULT_ERROR;
			}

			SPISelectDevice(SPI_SDCARD);

			// Multiple block read
			if (!SDCardSendCmd(SD_CARD_CMD18, SD_CARD_CMD18_R, sector, response, FALSE)) {
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// Check for response errors
			if (response[0] != SD_CARD_R1_OK) {
				sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// Receive the data block
			do {
				if (!SDCardRecvDatablock(buff, SD_CARD_SECTOR_LEN, &recivedCRC, TRUE)) {
					SDCardSPIDeselect();
					return SD_CARD_RESULT_ERROR;
				}
				//Check the CRC
				uint16 calculatedCRC = crcCCITTCalculate(buff, SD_CARD_SECTOR_LEN, TRUE);
				//cprintf("Received CRC: %04X, Calculated CRC: %04X\n", recivedCRC, calculatedCRC);
				if(recivedCRC != calculatedCRC){
					//cprintf("CRC Error...\n");
					sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
					SDCardSPIDeselect();
					return SD_CARD_RESULT_BAD_CRC;
				}
				buff += SD_CARD_SECTOR_LEN;
				numSectors--;
			} while (numSectors);
			// Stop the transmission
			if (!SDCardSendCmd(SD_CARD_CMD12, SD_CARD_CMD12_R, sector, response, FALSE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// Wait until the line is no longer busy
			if (!SDCardWaitIdle(FALSE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// Check for response errors
			if (response[0] != SD_CARD_R1_OK) {
				sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// De-Select the card and return with the data stored
			SDCardSPIDeselect();

		}

		return SD_CARD_RESULT_OK;
	}

	/**
	 * @brief Write sector(s).
	 * @param buff Pointer to the data to be written
	 * @param sector Start sector number (LBA).
	 * @param numSectors The number of 512 byte Sectors to be written (1..255).
	 */
	SDCardResult SDCardWrite(const uint8 *buff, uint32 sector, uint8 numSectors) {
		uint8 response[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

		// Basic error checking and error if the card isn't initialized.
		if (numSectors < 1) {
			return SD_CARD_RESULT_PARERR;
		}
		if (sdCardStatus != SD_CARD_STATUS_OK) {
			return SD_CARD_RESULT_NOTRDY;
		}

		// Translate the address to byte address for SDSC cards
		if (cardType == SD_CARD_TYPE_V1_SDSC || cardType == SD_CARD_TYPE_V2_SDSC) {
			sector *= SD_CARD_SECTOR_LEN;
		}

		// Single block writes
		if (numSectors == 1) {
			//Finish any previous writes or reads
			if(!SDCardWaitIdle(TRUE)){
				return SD_CARD_RESULT_ERROR;
			}

			// Select the card
			SPISelectDevice(SPI_SDCARD);

			// Single block write
			if (!SDCardSendCmd(SD_CARD_CMD24, SD_CARD_CMD24_R, sector, response, FALSE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			// Check for response errors
			if (response[0] != SD_CARD_R1_OK) {
				sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			//Send the 512 byte datablock with the appropriate token
			if(!SDCardXmitDatablock(buff, SD_CARD_WRITE_BLOCK_TOKEN)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			SDCardSPIDeselect();
		} else {
			// Multiple block writes

			//Finish any previous writes or reads
			if(!SDCardWaitIdle(TRUE)){
				return SD_CARD_RESULT_ERROR;
			}

			// Select the card
			SPISelectDevice(SPI_SDCARD);

			// Set the number of sectors to write to
			if (!SDCardSendAcmd(SD_CARD_ACMD23, SD_CARD_ACMD23_R, numSectors, response, FALSE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}

			// Send the sectors
			if (!SDCardSendCmd(SD_CARD_CMD25, SD_CARD_CMD25_R, sector, response, FALSE)){
				sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
				SDCardSPIDeselect();
				return SD_CARD_RESULT_ERROR;
			}
			if(response[0] == 0x00){
				do {
					if (!SDCardXmitDatablock(buff, SD_CARD_WRITE_DATA_TOKEN)) {
						sdCardStatus = (SD_CARD_STATUS_BAD_RESPONSE | SD_CARD_STATUS_NOINIT);
						SDCardSPIDeselect();
						return SD_CARD_RESULT_ERROR;
					}
					buff += SD_CARD_SECTOR_LEN;
					numSectors--;
				} while (numSectors > 0);
				if (!SDCardXmitDatablock(NULL, SD_CARD_STOP_TRANS_TOKEN)) {
					sdCardStatus = (SD_CARD_STATUS_NO_RESPONSE | SD_CARD_STATUS_NOINIT);
					SDCardSPIDeselect();
					return SD_CARD_RESULT_ERROR;
				}
			}
			SDCardSPIDeselect();
		}

		return SD_CARD_RESULT_OK;
	}
#endif
