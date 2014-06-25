/*
 * @file crc.h
 * @brief Bit by bit implementation of CRC (CCITT-32)
 * Source: http://www.barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 * @since August 8, 2013
 * @author William Xie
 */

#ifndef CRC_H_
#define CRC_H_


/**
 * @brief Compute CCITT-32 CRC value
 * @param message Pointer to the starting byte
 * @param nBytes Number of bytes in the packet
 * @returns The CRC value
 */
uint32 crcSlow(uint8 const message[], int nBytes);


#endif /* CRC_H_ */
