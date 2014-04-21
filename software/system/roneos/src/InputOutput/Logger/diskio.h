/**
 * @file diskio.h
 * @brief Interfaces the FatFS file-system SD card function calls to the SD card
 * implementation for roneos.
 *
 * @details
 * @since May 28, 2013
 * @author Jeremy Hunt
 */

/*-----------------------------------------------------------------------
/  Low level disk interface module include file
/-----------------------------------------------------------------------*/

#ifndef _DISKIO

#define _READONLY	0	/* 1: Remove write functions */
#define _USE_IOCTL	1	/* 1: Use disk_ioctl function */

#include "integer.h"


/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

//int assign_drives (int, int);
DSTATUS disk_initialize (BYTE);
DSTATUS disk_status (BYTE);
DRESULT disk_read (BYTE, BYTE*, DWORD, BYTE);
#if	_READONLY == 0
DRESULT disk_write (BYTE, const BYTE*, DWORD, BYTE);
#endif
DRESULT disk_ioctl (BYTE, BYTE, void*);

#define READ_RETRIES	5		/* The number of read re-tries before giving up */


/* Disk Status Bits (DSTATUS) */
#define STA_OK			0x00	/* Status is OK */
#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */


/* Command code for disk_ioctrl fucntion */

/* Generic command (defined for FatFs) */
#define CTRL_SYNC			0	/* Flush disk cache (for write functions) */
#define GET_SECTOR_COUNT	1	/* Get media size (for only f_mkfs()) */
#define GET_SECTOR_SIZE		2	/* Get sector size (for multiple sector size (_MAX_SS >= 1024)) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (for only f_mkfs()) */
#define CTRL_ERASE_SECTOR	4	/* Force erased a block of sectors (for only _USE_ERASE) */

/* Generic command */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */

/* NAND specific ioctl command */
#define NAND_FORMAT			30	/* Create physical format */


/**
 * @brief Initialized the disk drive (the SD Card).
 *
 * @param drive The drive which is to be initialized (Must be 0).
 * @returns DSTATUS The status of the drive as flags.
 */
DSTATUS disk_initialize (BYTE drive);


/**
 * @brief Gets the current status of the disk (SD card).
 *
 * @param drive The drive which is to be read (Must be 0).
 * @returns DSTATUS The status of the drive as flags
 */
DSTATUS disk_status (BYTE drive);

/**
 * Returns the current time (currently a fake time) as a packed 32 bit value.
 * Used in the FatFs File system.
 */
DWORD get_fattime (void);

#define _DISKIO
#endif
