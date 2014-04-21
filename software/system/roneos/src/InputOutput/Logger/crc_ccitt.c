#include "roneos.h"

/*******************************************************************\
*                                                                   *
*   Library         : lib_crc                                       *
*   File            : lib_crc.c                                     *
*   Author          : Lammert Bies  1999-2008                       *
*   E-mail          : info@lammertbies.nl                           *
*   Language        : ANSI C                                        *
*   Website:		: 												*
*   http://www.lammertbies.nl/comm/info/crc-calculation.html#intr	*
*                                                                   *
*                                                                   *
*   Description                                                     *
*   ===========                                                     *
*   The file lib_crc.c contains the private  and  public  func-     *
*   tions  used  for  the  calculation of CRC-CCITT cyclic 			*
*   redundancy values.                                				*
*                                                                   *
*                                                                   *
*   Dependencies                                                    *
*   ============                                                    *
*   lib_crc.h       CRC definitions and prototypes                  *
\*******************************************************************/

// The Coefficient for the CRC CCITT algorithm's polynomial in hex
#define                 P_CCITT     0x1021

/*******************************************************************\
*                                                                   *
*   unsigned short update_crc_ccitt( unsigned long crc, char c );   *
*                                                                   *
*   The function update_crc_ccitt calculates  a  new  CRC-CCITT     *
*   value  based  on the previous value of the CRC and the next     *
*   byte of the data to be checked.                                 *
*                                                                   *
\*******************************************************************/
uint16 crcCCITTUpdate( uint16 crc, uint8 byte ) {

	uint16 tmp, short_byte;

    short_byte  = 0x00ff & (uint16)byte;

    tmp = (crc >> 8) ^ short_byte;
    crc = (crc << 8) ^ crcCCITTTab[tmp];

    return crc;

}

/*
 * Calculates the CCITT crc of a block of data pointed to by the variable
 * data of length dataLen.
 *
 * @param littleEndian A boolean that describes how data will be read into the buffer. TRUE indicates MSB in byte (blockLen-1), FALSE indicates MSB in byte 0.
 *
 */
uint16 crcCCITTCalculate(uint8 * data, uint16 dataLen, boolean littleEndian) {
	// Initialize the CCITT initialization value of 0 (used for SD cards)
	uint16 crc_ccitt = 0x00;
	uint16 i=0;

	// Return if there is no data
	if(dataLen == 0) {
		return crc_ccitt;
	}

	// Calculate the CRC
	if(littleEndian){
		i=0;
		do {
			crc_ccitt = crcCCITTUpdate(crc_ccitt, data[i]);
			i++;
		} while (i < dataLen);
	} else {
		do {
			dataLen--;
			crc_ccitt = crcCCITTUpdate(crc_ccitt, data[dataLen]);
		} while (dataLen > 0);
	}
	return crc_ccitt;
}


