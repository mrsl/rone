/*
 * @file crc.c
 * @brief Bit by bit implementation of CRC (CCITT-32)
 * Source: http://www.barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 * @since August 8, 2013
 * @author William Xie
 */
#include <stdio.h>

#include "System/bootloader.h"

#define GENERATOR_POLYNOMIAL 0x04C11DB7
#define WIDTH  (8 * sizeof(uint32))
#define TOPBIT (1 << (WIDTH - 1))

/*
 * @brief Compute CCITT-32 CRC value
 * @param message Pointer to the starting byte
 * @param nBytes Number of bytes in the packet
 * @returns The CRC value
 */
uint32 crcSlow(uint8 const message[], int nBytes)
{
    uint32 remainder = 0;
    uint32 byte;
    uint8 bit;

    //Perform modulo-2 division, a byte at a time.     
    for (byte = 0; byte < nBytes; byte++) {
        // Bring the next byte into the remainder.      
        remainder ^= (message[byte] << (WIDTH - 8));

		// Perform modulo-2 division, a bit at a time.
        for (bit = 8; bit > 0; bit--) {
			// Try to divide the current data bit.
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ GENERATOR_POLYNOMIAL;
            } else {
                remainder = (remainder << 1);
            }
        }
    }

    // The final remainder is the CRC result.
    return (remainder);
} 
