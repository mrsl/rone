/*
 * @file basicPrinting.c
 * @brief Print methods for converting data into strings with different formats.
 * @since March 24, 2012
 * @author: James McLurkin
 */

#include <stdio.h>

#include "roneos.h"


/*
 * 	@brief Function that allows us to print binary.
 *
 * 	Creates a binary version of the input character in the space given by string.
 *	@param *string is the char pointer that will point to the binary bitstring of val
 *	@param val is the value to be converted to binary
 *	@return a pointer to the binary string converted from val
 */
char* bitString8(char* string, uint8 val) {
	uint8 i;
	char* charPtr = string;

	for (i = 0; i < 8; i++) {
		if (val & 0b10000000) {
			*charPtr++ = '1';
		} else {
			*charPtr++ = '0';
		}
		val = val << 1;
	}
	*charPtr = '\0';
	return (string);
}


/*
 * 	@brief Function that allows us to print binary.
 *
 * 	Creates a binary version of the input character in the space given by string.
 *	@param *string is the char pointer that will point to the binary bitstring of val
 *	@param val is the value to be converted to binary
 *	@return a pointer to the binary string converted from val
 */
char* bitString16(char* string, uint16 val) {
	bitString8(string, (uint8)(val >> 8));
	bitString8(string + 8, (uint8)(val & 0xFF));
	return (string);
}


/*
 * 	@brief Print a pose structure
 *
 * 	Print a pose structure.  Prints in braces to be fancy.
 *	@param posePtr is a pointer to a pose.
 */
void posePrint(Pose* posePtr) {
	if(posePtr) {
		cprintf("{%d,%d,%d}", posePtr->x, posePtr->y, posePtr->theta);
	}
}


/*
 * @brief Convert single character to an integer.
 *
 * Converts a single character that represents a unicode number (hex) to a unicode number(integer)
 * @param c is the character to be converted
 * @returns val is the integer value
 */
uint8 ctoi_hex4(char c) {
	uint8 val = 0;

	if ((c >= '0') && (c <= '9')) {
		val = c - '0';
	} else if (c >= 'a' && c <= 'f') {
		val = c - 'a' + 10;
	} else if (c >= 'A' && c <= 'F') {
		val = c - 'A' + 10;
	}

	return val;
}

/*
 * @brief Convert 8-bit hex string to an integer.
 *
 * @param string 8-bit hex string to be converted
 * @returns hex integer version of input
 */
uint8 atoi_hex8(char *string) {
       uint8 temp;
       temp = ctoi_hex4(*string) * 16;
       temp = temp + ctoi_hex4(*(string + 1));
       return temp;
}

/*
 * @brief Convert 16-bit hex string to an integer.
 *
 * @param string 16-bit hex string to be converted
 * @returns hex integer version of input
 */
uint16 atoi_hex16(char *string) {
       uint16 temp;
       temp = (uint16)(atoi_hex8(string)) * 256;
       temp += (uint16)(atoi_hex8(string + 2));
       return temp;
}

/*
 * @brief Convert 32-bit hex string to an integer.
 *
 * @param string 32-bit hex string to be converted
 * @returns hex integer version of input
 */
uint32 atoi_hex32(char *string) {
       uint32 temp;
       temp = (uint32)atoi_hex16(string) * 65536;
       temp += (uint32)atoi_hex16(string + 4);
       return temp;
}

