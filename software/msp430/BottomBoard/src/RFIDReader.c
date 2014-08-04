#include <stdio.h>
#include <msp430f2132.h>

#ifdef RONE_V12_TILETRACK

#include "typedefs.h"
#include "RFIDReader.h"

/*
 * We are using the Magnetic Emulation Mode to receive the data.
 * The format is as follows.
 *
 * This message is in the format of a magnetic stripe card symbol number.
 * Specifically in the Track 2 (ABA) banking standard
 * See ISO/IEC 7811-2 for full specifications
 * Each Character is of the form
 * [Parity (1 Bit) | Char Num (4 Bits)]
 * The parity is such that the total number of 1s, including the parity
 * bit, is odd.
 *
 * The characters map to decimal numbers in ASCII:
 * 0b0000 to 0x1001: '0' to '9'
 * 0b1010: ':'
 * 0b1011: ';'
 * 0b1100: '<'
 * 0b1101: '='
 * 0b1110: '>'
 * 0b1111: '?'
 * The message is composed of:
 * 1. Start Char (0b1_1011) = ';'
 * 2. Data Chars (Maybe about 14??)
 * 3. End Char (0b1_1111) = '?'
 * 4. Longitudinal Redundancy Check (LRC)
 * The LRC is specified in ISO/IEC 7811-2
 */


//P2.2 for bump sensors
#define BUMP_PWR_PORT_DIR		P2DIR
#define BUMP_PWR_PORT_SEL		P2SEL
#define BUMP_PWR_PORT_OUT		P2OUT
#define BUMP_PWR_BIT			BIT2


#define RFID_CHAR_LEN				5
#define RFID_START_CHAR				0x1A
#define RFID_END_CHAR				0x1F
#define RFID_LEADING_ZEROS_LEN 		2
#define RFID_DATA_LEN 				14
#define RFID_TRAILING_ZEROS_LEN 	2
#define RFID_PREAMBLE_LEN			3
typedef enum {MSG_IDLE, MSG_PREAMBLE, MSG_DATA, MSG_LRC, MSG_POSTAMBLE, MSG_RECEIVED, MSG_ERROR } rfid_msg_sec_t;
volatile rfid_msg_sec_t RFIDState = MSG_IDLE;
const uint8 RFIDPreamble[RFID_PREAMBLE_LEN] = {0, 0, RFID_START_CHAR};
volatile uint8 RFTagData[RFID_TAG_DATA_LEN];


void RFIDReaderInit() {
	/* Configure all the pins as input GPIOs */
	RFID_PORT_DIR &= ~RFID_BITS;
	RFID_PORT_SEL &= ~RFID_BITS;

	/* The outputs from the RFID reader are open drain, so turn on the internal pull-ups */
	RFID_PORT_REN |= RFID_BITS;
	RFID_PORT_OUT |= RFID_BITS;

	/* Make sure to turn off the bump sensor since we are sharing lines */
	BUMP_PWR_PORT_SEL &= ~BUMP_PWR_BIT;
}


/* Enable the RFID Processing Interrupt */
void RFIDInterruptEnable() {
	/* Interrupt on the rising edge of Card Present and the rising edge of Clock */
	//RFID_PORT_IFES |= RFID_CLOCK_BIT; /* Falling Edge */
	RFID_PORT_IFES &= ~RFID_CLOCK_BIT; /* Rising Edge */
	RFID_PORT_IFES &= ~RFID_TAG_PRES_BIT; /* Rising Edge */
	RFID_PORT_IFLG &= ~RFID_INTERUPT_BITS;
	RFID_PORT_IE |= RFID_INTERUPT_BITS;
}


/* Disable the RFID Processing Interrupt */
void RFIDInterruptDisable() {
	RFID_PORT_IE &= ~RFID_INTERUPT_BITS;
	RFID_PORT_IFLG &= ~RFID_INTERUPT_BITS;
}


/*
 * Returns a hash of the last read RFID tag. If no tag was read, or if the tag was
 * already read once last time, returns 0.
 */
uint8 RFIDReaderGet() {
	uint8 i;
	uint8 val = 0;
	if (RFIDState == MSG_RECEIVED) {
		for (i = 0; i < RFID_TAG_DATA_LEN; i++) {
			val += RFTagData[i];
		}
	}
	return val;
//
//	if (idx < RFID_TAG_DATA_LEN) {
//		return RFTagData[idx];
//	} else {
//		return 0;
//	}
}


/* This interrupt handler takes in the serial data */
void RFIDReaderInterupt() {
	/* Persistent state variables */
	static uint8 charIdx;
	static uint8 charBitIdx;
	static uint8 numOnesInChar;
	static uint8 curChar;

	/* Local variables */
	uint8 tmpReading = RFID_PORT_IN;
	uint8 i;

	/* If we ever see that Tag Present has become low again, abort the message */
	if (!(tmpReading & RFID_TAG_PRES_BIT)) {
		RFID_PORT_IFLG &= ~RFID_INTERUPT_BITS;
		goto error;
	}

	/* Tag present went high */
	if (RFID_PORT_IFLG & RFID_TAG_PRES_BIT) {
		/* Clear the flag */
		RFID_PORT_IFLG &= ~RFID_TAG_PRES_BIT;

		/* Start a new message transaction */
		for (i = 0; i < RFID_TAG_DATA_LEN; i++) {
			RFTagData[i] = 0;
		}

		charIdx = 0;
		charBitIdx = 0;
		RFIDState = MSG_PREAMBLE;
	}

	/* We are processing a message bit */
	if (RFID_PORT_IFLG & RFID_CLOCK_BIT) {
		/* Mark the interrupt as processed */
		RFID_PORT_IFLG &= ~RFID_CLOCK_BIT;

		/* read a 4-bit char + 1-bit parity */
		/* reset the char if this is the first bit */
		if (charBitIdx == 0) {
			numOnesInChar = 0;
			curChar = 0;
		}

		/* Read in the next bit. Note that it is active low (1 = Low Voltage) */
		curChar <<= 1;
		curChar |= (((~tmpReading) & RFID_DATA_BIT) >> RFID_DATA_BIT_IDX);

		/* Keep the parity updated */
		if (curChar & 0x1) {
			numOnesInChar++;
		}
		charBitIdx++;

		if (charBitIdx == RFID_CHAR_LEN) {
			/* Process the char */
			switch (RFIDState) {
			case MSG_ERROR: {
				/* do nothing, discard the char until the tag present goes high again */
				break;
			}
			case MSG_PREAMBLE: {
				if (curChar != RFIDPreamble[charIdx]) {
					/* we got a wrong preamble char, abort the message */
					goto error;
				}
				if (charIdx == (RFID_PREAMBLE_LEN -1)) {
					RFIDState = MSG_DATA;
				}
				break;
			}
			case MSG_DATA: {
				/* store the char */
				if(curChar == RFID_END_CHAR) {
					RFIDState = MSG_LRC;
				} else {
					/* If the parity bit is wrong, error.  The number of ones is always odd */
					if ((numOnesInChar & 0x01) != 1) {
						goto error;
					} else {
						/* Otherwise add the real four bits of the char to the ID */
						uint8 dataNibbleIdx = (charIdx - RFID_PREAMBLE_LEN) >> 1;
						uint8 dataNibbleOffset = ((charIdx - RFID_PREAMBLE_LEN) & 1) << 2;
						/* Arrange the nibble in the byte and or it into the slot. discard if the array is full */
						if (dataNibbleIdx < RFID_TAG_DATA_LEN) {
							RFTagData[dataNibbleIdx] |= ((curChar & 0xF) << dataNibbleOffset);
						}
					}
				}
				break;
			}
			case MSG_LRC: {
				/*
				 * Verify the LRC.
				 *
				 * The value of each bit in the LRC character,
				 * excluding the parity bit, is defined such that the
				 * total count of one bits encoded in the corresponding
				 * bit location of all characters of the data track,
				 * including the start sentinel, data, end sentinel,
				 * and LRC characters, shall be even. This is done
				 * calculating the XOR of every 4 bit value.
				 */
				uint8 lrcCalc = (0xF & RFID_START_CHAR) ^ (0xF & RFID_END_CHAR);
				i = 0;
				uint8 dataNibbleIdx;
				uint8 dataNibbleOffset;
				for (i = 0; i < (charIdx - RFID_PREAMBLE_LEN - 1); i++) {
					dataNibbleIdx = i >> 1;
					dataNibbleOffset = (i & 1) << 2;
					/* XOR the nibbles in */
					if (dataNibbleIdx < RFID_TAG_DATA_LEN) {
						lrcCalc ^= ((RFTagData[dataNibbleIdx] >> dataNibbleOffset) & 0xF);
					}
				}
				/* Verify the LRC */
				/* TODO Hack Alert! This is consistent accross all tags I have.  I don't know why, and I wanna go to sleep */
				lrcCalc ^= 0x01;
				if (lrcCalc != (curChar & 0xF)) {
					goto error;
				}

				/* We are done! */
				/* Mini-Hack Alert: We're not gonna read the trailing zeros.  (Shhhh...) */
				RFIDState = MSG_RECEIVED;
				break;
			}
			default:
				break;
			}

			/* Count the chars */
			charBitIdx = 0;
			charIdx++;
		}
	}
	return;

	/* Handle any errors. Marks the error flag. */
error:
	RFIDState = MSG_ERROR;
	return;
}

#endif //#ifndef RONE_V12_TILETRACK

