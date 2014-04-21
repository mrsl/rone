/*
 * @file diskio.c
 * @author Jeremy Hunt
 * @brief Interfaces the FatFS file-system SD card function calls to the SD card
 * implementation for roneos.
 */

#include "roneos.h"

/*
 * @brief Initialized the disk drive (the SD Card).
 *
 * @param drive The drive which is to be initialized (Must be 0).
 * @returns DSTATUS The status of the drive as flags.
 */
DSTATUS disk_initialize (BYTE drive){
	/* There is only one drive */
	if(drive != 0){
		return (STA_NODISK | STA_NOINIT);
	}

	uint8 status = SDCardInit();
	if(status != SD_CARD_STATUS_OK){
		return STA_NOINIT;
	}

	//Initialization Succeed
	return STA_OK;
}


/*
 * @brief Gets the current status of the disk (SD card).
 *
 * @param drive The drive which is to be read (Must be 0).
 * @returns DSTATUS The status of the drive as flags
 */
DSTATUS disk_status (BYTE drive){
	if(drive != 0){
		return (STA_NODISK | STA_NOINIT);
	}

	if(SDCardGetStatus() != SD_CARD_STATUS_OK){
		return STA_NOINIT;
	}

	return STA_OK;
}

DRESULT disk_read (BYTE drive, BYTE * buff, DWORD sector, BYTE numberOfSectors){
	if(drive != 0){
		return RES_PARERR;
	}
	if(SDCardGetStatus() != SD_CARD_STATUS_OK){
			return RES_NOTRDY;
	}

	/* Attempt to read the card. Try up to READ_RETRIES times in the event of a CRC error */
	int i = 0;
	SDCardResult readStatus;
	do {
		i++;
		readStatus = SDCardRead(buff, sector, numberOfSectors);
		if(readStatus != SD_CARD_RESULT_BAD_CRC &&
			readStatus != SD_CARD_RESULT_OK) {
			return RES_ERROR;
		}
	} while(i < READ_RETRIES && readStatus != SD_CARD_RESULT_OK);
	if(i >= READ_RETRIES){
		return RES_ERROR;
	}

	return RES_OK;
}


#if	_READONLY == 0
DRESULT disk_write (BYTE drive, const BYTE * buff, DWORD sector, BYTE numberOfSectors){
	if(drive != 0){
		return RES_PARERR;
	}
	if(SDCardGetStatus() != SD_CARD_STATUS_OK){
			return RES_NOTRDY;
	}

	if(SDCardWrite(buff, sector, numberOfSectors) != SD_CARD_RESULT_OK){
		return RES_ERROR;
	}
	return RES_OK;
}
#endif


DRESULT disk_ioctl (BYTE drive, BYTE command, void * buff){
	if(drive != 0){
		return RES_PARERR;
	}
	if(SDCardGetStatus() != SD_CARD_STATUS_OK){
		return RES_NOTRDY;
	}

	switch(command){
	case CTRL_SYNC: {
		if(!SDCardWaitIdle(TRUE)){
			return RES_ERROR;
		}
		return RES_OK;
	}
	case GET_SECTOR_SIZE: {
		/* Put the block/sector length in bytes into the WORD pointed to by the void* buff pointer */
		*((WORD*)buff) = SD_CARD_SECTOR_LEN;
		return RES_OK;
	}
	case GET_SECTOR_COUNT: {
		uint32 numSectors = 0;
		uint8 CSDRaw[16];

		// Read the card Specific data register.
		if(!SDCardReadCSD(CSDRaw, TRUE)){
			cprintf("Error Reading Card Specific Data Register\n");
			return RES_ERROR;
		}
		// Process the CSD based on its type
		if((CSDRaw[15] & 0xC0) == 0x00){ //Version 1.0
			SDCardCSDSDV1 cardData;
			SDCardProcessCSDSDV1(CSDRaw, &cardData);
			numSectors = (cardData.C_SIZE + 1) << (cardData.C_SIZE_MULT + cardData.READ_BL_LEN - 7);
			cprintf("Card Size: %d Sectors (Each 512 Bytes).\n", numSectors);
		} else if((CSDRaw[15] & 0xC0) == 0x40) { //Version 2.0
			SDCardCSDSDV2 cardData;
			SDCardProcessCSDSDV2(CSDRaw, &cardData);
			numSectors = (cardData.C_SIZE + 1) << 10;
			cprintf("Card Size: %d Sectors (Each 512 Bytes).\n", numSectors);
		} else {
			cprintf("Error Reading Card Specific Data Register\n");
			return RES_ERROR;
		}

		// Put the card size into the correct pointer location
		*((DWORD*) buff) = (DWORD)numSectors;
		return RES_OK;
	}
	case GET_BLOCK_SIZE: {
		/* Get erase block size in number of sectors (DWORD) */
		*((DWORD*)buff) = SD_CARD_ERASE_BLOCK_LEN;
		break;
	}
	/* Erasing is not supported */
	default: {
		return RES_PARERR;
	}
	}

	return RES_ERROR;
}


//TODO: Get a real Real Time Clock on the robots so that this function doesn't
//return a fake date...
/*
 * Returns the current time (currently a fake time) as a packed 32 bit value.
 * Used in the FatFs File system.
 */
DWORD get_fattime (void){
	// Return January 1, 2012 at Noon
	DWORD time = 0;
	time |= (32 & 0x7F) << 25; //Year since 1980 (7 bits)
	time |= (1  & 0x0F) << 21; //Month (1...12)
	time |= (1  & 0x1F) << 16; //Day in month (1...31)
	time |= (12 & 0x1F) << 11; //Hour (0...23)
	time |= (0  & 0x3F) << 5 ; //Minute (0...59)
	time |= (0  & 0x1F)		 ; //Seconds/2 (0...29)

	return time;
}
