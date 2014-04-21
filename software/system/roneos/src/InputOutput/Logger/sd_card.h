/*
 * sd_card.h
 *
 * Created on: Jun 28, 2011
 * 		Author: Jeremy Hunt and Nathan Alison
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

// The Card Specific Data structure for V1 SD cards
struct SDCardCSDSDV1 {
	uint8 CSD_STRUCTURE			: 2;
	uint8						: 6;
	uint8 TAAC					: 8;
	uint8 NSAC					: 8;
	uint8 TRAN_SPEED			: 8;
	uint16 CCC					: 12;
	uint8 READ_BL_LEN			: 4;
	boolean READ_BL_PARTIAL		: 1;
	boolean WRITE_BLK_MISALIGN	: 1;
	boolean READ_BLK_MISALIGN	: 1;
	boolean DSR_IMP				: 1;
	uint8						: 2;
	uint16 C_SIZE				: 12;
	uint8 VDD_R_CURR_MIN		: 3;
	uint8 VDD_R_CURR_MAX		: 3;
	uint8 VDD_W_CURR_MIN		: 3;
	uint8 VDD_W_CURR_MAX		: 3;
	uint8 C_SIZE_MULT			: 3;
	boolean ERASE_BLK_EN		: 1;
	uint8 SECTOR_SIZE			: 7;
	uint8 WP_GRP_SIZE			: 7;
	boolean WP_GRP_ENABLE		: 1;
	uint8						: 2;
	uint8 R2W_FACTOR			: 3;
	uint8 WRITE_BL_LEN			: 4;
	boolean WRITE_BL_PARTIAL	: 1;
	uint8						: 5;
	uint8 FILE_FORMAT_GRP		: 1;
	boolean COPY				: 1;
	boolean PERM_WRITE_PROTECT	: 1;
	boolean TMP_WRITE_PROTECT	: 1;
	uint8 FILE_FORMAT			: 2;
	uint8						: 2;
	uint8 CRC					: 7;
	uint8						: 1;
};

// The Card Specific Data Structure for V2 SD Cards
struct SDCardCSDSDV2 {
	uint8 CSD_STRUCTURE			: 2;
	uint8 TRAN_SPEED			: 8;
	uint16 CCC					: 12;
	uint8 DSR_IMP				: 1;
	uint32 C_SIZE				: 22;
	uint8 COPY					: 1;
	uint8 PERM_WRITE_PROTECT	: 1;
	uint8 TMP_WRITE_PROTECT		: 1;
	uint8 CRC					: 7;
};

typedef struct SDCardCSDSDV1 SDCardCSDSDV1;
typedef struct SDCardCSDSDV2 SDCardCSDSDV2;


/* Base SD Card Parameters */
#define SD_CARD_DEFAULT_FREQUENCY		350000
#define SD_CARD_INIT_FREQUENCY			350000
// Note that this fast frequency can probably go up to 20 MHz or higher, but this should work for all MMC/SD cards.
// ALthough this may be as fast as it can go
#define SD_CARD_FAST_FREQUENCY			12500000
// This is to allow non-byte aligned cards to synchronize with this system.
// This is the speed of the synchronization clock. Lower is faster.
#define SD_CARD_SYNCHRONIZE_WAIT		3
/*
 * This defines which voltage ranges must be defined to work on the robot
 * Currently set to 3.0V-3.5V
 * Bit | Range
 * 0-14 reserved
 * 15 2.7-2.8
 * 16 2.8-2.9
 * 17 2.9-3.0
 * 18 3.0-3.1
 * 19 3.1-3.2
 * 20 3.2-3.3
 * 21 3.3-3.4
 * 22 3.4-3.5
 * 23 3.5-3.6
 */
#define SD_CARD_MIN_VOLTAGE_RANGE		0x007C0000
// The length of each sector. Do not change this from 512 bytes if you want to maintain support with V2+ cards
#define SD_CARD_SECTOR_LEN				512
// The length of a block erase
#define SD_CARD_ERASE_BLOCK_LEN			128 /* In number of sectors */

// Do we support (0x40000000) or not support (0x00) high capacity cards
#define SD_CARD_HCS	0x40000000	/* This code supports high capacity cards */

// R1 Response Bits
#define SD_CARD_R1_OK				0x00
#define SD_CARD_R1_IDLE				0x01
#define SD_CARD_R1_ERASE_RST		0x02
#define SD_CARD_R1_ILLEGAL_CMD		0x04
#define SD_CARD_R1_CMD_CRC_ERROR	0x08
#define SD_CARD_R1_ERASE_SEQ_ERROR	0x10
#define SD_CARD_R1_ADDRESS_ERROR	0x20
#define SD_CARD_R1_PARAMETER_ERROR	0x40

// Response Tokens
#define SD_CARD_READ_TOKEN			0xFE
#define SD_CARD_WRITE_BLOCK_TOKEN	0xFE
#define SD_CARD_WRITE_DATA_TOKEN	0xFC
#define SD_CARD_STOP_TRANS_TOKEN	0xFD
#define SD_CARD_WRITE_RESPONSE_OK   0x05

// Operating Condition Register Bits
#define SD_CARD_OCD_CCS_BIT			0x40000000	/* Card Capacity Status Bit */
#define SD_CARD_OCD_POWER_ON_BIT	0x80000000  /* Fully initialized on bit */

// SD Card Types
#define SD_CARD_TYPE_UNKNOWN		0
#define SD_CARD_TYPE_V1_SDSC		1 /* Older SD cards <=2 GB using the V1.0 Specs */
#define SD_CARD_TYPE_V2_SDSC		2 /* Newer small capcity (<=2 GB) using the V2.0+ Specs */
#define SD_CARD_TYPE_V2_SDHC_SDXC	3 /* Cards in the 4GB <= Capacity <= 32GB range using the V2.0+ Specs and Cards in the 32GB <= Capacity <= 2 TB range using the V2.0+ Specs */

// Results of disk functions
#define SD_CARD_RESPONSE_OK 		0	/* Successful */
#define SD_CARD_RESPONSE_ERROR 		1	/* R/W Error */
#define SD_CARD_RESPONSE_WRPRT		2 	/* Write Protected */
#define SD_CARD_RESPONSE_NOTRDY		3 	/* Not Ready */
#define SD_CARD_RESPONSE_PARERR		4	/* Invalid Parameter */

// SD Card Error Result Bits
/* Results of Disk Functions */
typedef enum {
	SD_CARD_RESULT_OK = 0,		/* The function succeeded. */
	SD_CARD_RESULT_ERROR,		/* Any hard error occured during the write operation and could not recover it. */
	SD_CARD_RESULT_WRPRT,		/* The medium is write protected. */
	SD_CARD_RESULT_NOTRDY,		/* The disk drive has not been initialized. */
	SD_CARD_RESULT_PARERR,		/* Invalid Parameter */
	SD_CARD_RESULT_BAD_CRC,		/* The received CRC is incorrect */
} SDCardResult;

// Timeouts in number of attempts
#define SD_CARD_CMD_TIMEOUT			10		/* In bytes */
#define SD_CARD_CMD_RETRY_TIMEOUT	3		/* In number of attempts */
#define SD_CARD_INIT_TIMEOUT		2		/* In number of CMD0s sent */
#define SD_CARD_IDLE_WAIT_TIMEOUT	4000	/* Apparently, some cards can take a long time to initialize. This is about 1 Second */
#define SD_CARD_READ_TIMEOUT		7500	/* This is number of bytes. */
#define SD_CARD_BUSY_TIMEOUT		1200000 /* In number of bytes. SD cards take a loooong time... About 0.7 Seconds at 12.5MHz */

/* Disk Status Bits */
#define SD_CARD_STATUS_OK				0x00	/* The SD Card is fine */
#define SD_CARD_STATUS_NOINIT			0x01	/* Drive not initialized */
#define SD_CARD_STATUS_NO_RESPONSE		0x02	/* No Response from the SD Card */
#define SD_CARD_STATUS_BAD_RESPONSE		0x04	/* The card gave an incorrect command response */
#define SD_CARD_STATUS_PROTECT			0x08	/* Write protected */
#define SD_CARD_STATUS_NOT_SUPPORTED	0x10	/* This SD card is not supported */


/* Definitions for SD Card command */
#define SD_CARD_CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define SD_CARD_CMD1	(0x40+1)	/* SEND_OP_COND */
#define SD_CARD_CMD8	(0x40+8)	/* SEND_IF_COND */
#define SD_CARD_CMD9	(0x40+9)	/* SEND_CSD */
#define SD_CARD_CMD10	(0x40+10)	/* SEND_CID */
#define SD_CARD_CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define SD_CARD_CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define SD_CARD_CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define SD_CARD_CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define SD_CARD_ACMD23	(0x40+23)	/* SET_BLOCK_COUNT */
#define SD_CARD_CMD24	(0x40+24)	/* WRITE_BLOCK */
#define SD_CARD_CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define SD_CARD_ACMD41	(0x40+41)	/* SEND_OP_COND (ACMD) */
#define SD_CARD_CMD55	(0x40+55)	/* APP_CMD */
#define SD_CARD_CMD58	(0x40+58)	/* READ_OCR */

/* Response Types */
#define SD_CARD_R1		0
#define SD_CARD_R1B		1
#define SD_CARD_R2		2
#define SD_CARD_R3		3
#define SD_CARD_R6		4
#define SD_CARD_R7		5

/* Definitions for MMC/SDC command Responses*/
#define SD_CARD_CMD0_R		SD_CARD_R1	/* GO_IDLE_STATE */
#define SD_CARD_CMD1_R		SD_CARD_R1	/* SEND_OP_COND */
#define SD_CARD_CMD8_R		SD_CARD_R7	/* SEND_IF_COND */
#define SD_CARD_CMD9_R		SD_CARD_R1	/* SEND_CSD */
#define SD_CARD_CMD10_R		SD_CARD_R1	/* SEND_CID */
#define SD_CARD_CMD12_R		SD_CARD_R1B	/* STOP_TRANSMISSION */
#define SD_CARD_CMD16_R		SD_CARD_R1	/* SET_BLOCKLEN */
#define SD_CARD_CMD17_R		SD_CARD_R1	/* READ_SINGLE_BLOCK */
#define SD_CARD_CMD18_R		SD_CARD_R1	/* READ_MULTIPLE_BLOCK */
#define SD_CARD_ACMD23_R	SD_CARD_R1	/* SET_BLOCK_COUNT */
#define SD_CARD_CMD24_R		SD_CARD_R1	/* WRITE_BLOCK */
#define SD_CARD_CMD25_R		SD_CARD_R1	/* WRITE_MULTIPLE_BLOCK */
#define SD_CARD_ACMD41_R	SD_CARD_R1	/* SEND_OP_COND (ACMD) */
#define SD_CARD_CMD55_R		SD_CARD_R1	/* APP_CMD */
#define SD_CARD_CMD58_R		SD_CARD_R3	/* READ_OCR */


uint32 SDCardGetFrequency(void);
uint8 SDCardInit(void);
boolean SDCardWaitIdle(boolean selectCard);
boolean SDCardReadCSDSDV2(SDCardCSDSDV2 *CSDStruct, boolean selectCard);
boolean SDCardReadCSDSDV1(SDCardCSDSDV1 *CSDStruct, boolean selectCard);
uint8 SDCardRecvSPI(void);
boolean SDCardRecvDatablock(uint8 * buff, uint16 blockLen, uint16 * crcPtr, boolean littleEndian);
boolean SDCardXmitDatablock(const uint8 *buff, uint8 token);
boolean SDCardSendCmd(uint8 cmd, uint8 responseType, uint32 argument, uint8 * response, boolean selectCard);
boolean SDCardSendAcmd(uint8 acmd, uint8 responseType, uint32 argument, uint8 * response, boolean selectCard);
boolean SDCardSendCmdOnce(uint8 cmd, uint8 responseType, uint32 argument, uint8 * response, boolean selectCard);
uint8 SDCardGetStatus(void);
uint8 SDCardGetType(void);
uint8 SDCardRead(uint8 *buff, uint32 sector, uint8 count) ;
uint8 SDCardWrite(const uint8 *buff, uint32 sector, uint8 count);
boolean SDCardReadCSD(uint8 * CSDRaw, boolean selectCard);
boolean SDCardProcessCSDSDV1(uint8 * CSDRaw, SDCardCSDSDV1 *CSDStruct);
boolean SDCardProcessCSDSDV2(uint8 * CSDRaw, SDCardCSDSDV2 *CSDStruct);


#endif /* SD_CARD_H_ */
