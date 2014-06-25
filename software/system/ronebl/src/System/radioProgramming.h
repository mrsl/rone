/*
 * radioProgramming.h
 *
 *  Created on: Aug 8, 2013
 *      Author: wgx1
 */

#ifndef RADIOPROGRAMMING_H_
#define RADIOPROGRAMMING_H_

// Modes
#define RADIO_PROGRAMMING_HOST		0
#define RADIO_PROGRAMMING_REMOTE	1
#define RADIO_PROGRAMMING_LIMBO		2

// Delay timings
#define TRANSITION_DELAY_MS			500
#define RADIO_MIN_TX_DELAY_US		100

#define CRC_SEGMENT_SIZE_KB			4
#define CRC_SEGMENT_SIZE			CRC_SEGMENT_SIZE_KB * 1024
#define CRC_TABLE_SIZE				MAX_PROGRAM_SIZE / CRC_SEGMENT_SIZE_KB

#define CRC_TABLE_SEGMENT_IDX		255

#define MAX_CRC_BROADCAST			50
#define MAX_QUERY_REQUEST			7
#define MAX_REBOOT_COUNT			50
#define MAX_PROGRAM_TIME_COUNT		75
#define MAX_QUERY_REQUEST_WAIT  	30000

#define PRINT_BYTE_SIZE				256

extern uint8 robotIDMin;
extern uint8 robotIDMax;



/**
 * @brief Set the query range for the host robot from user serial input
 *
 * @return void
 */
void radioProgrammingInput();

/**
 * @brief Construct a CRC table for all segments of the program
 * @param startingAddress The starting address of the program
 * @param length Length in KB of the program
 * @returns void
 */
void crcTableInit(uint32 startingAddress, uint16 length);


/**
 * @brief Retrieve the CRC value from CRC table. Should be called after crcTableInit
 * @param index The segment index
 * @returns The CRC-32 value
 */
uint32 getCRCTableValue(uint16 index);


/**
 * @brief Compute local CRCTable and run host/radio mode code.
 *
 * @param mode select the host or remote mode (see header file)
 * @return void
 */
void radioProgramming(uint8 mode);


#endif /* RADIOPROGRAMMING_H_ */
