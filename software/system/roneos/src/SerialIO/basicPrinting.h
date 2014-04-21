/**
 * @file basicPrinting.h
 *
 * @brief Print methods for converting data into strings with different formats.
 * @since March 24, 2012
 * @author: James McLurkin
 */

#ifndef BASICPRINTING_H_
#define BASICPRINTING_H_


/**
 * 	@brief Function that allows us to print binary.
 *
 * 	Creates a binary version of the input character in the space given by string.
 *	@param string is the char pointer that will point to the binary bitstring of val
 *	@param val is the value to be converted to binary
 *	@return a pointer to the binary string converted from val
 */
char* bitString8(char* string, uint8 val);
char* bitString16(char* string, uint16 val);


/**
 * @brief Convert single character to an integer.
 *
 * Converts a single character that represents a unicode number (hex) to a unicode number(integer)
 * @param c is the character to be converted
 * @returns val is the integer value
 */
uint8 ctoi_hex4(char c);


/**
 * @brief Convert 8-bit hex string to an integer.
 *
 * @param string 8-bit hex string to be converted
 * @returns hex integer version of input
 */
uint8 atoi_hex8(char *string);


/**
 * @brief Convert 16-bit hex string to an integer.
 *
 * @param string 16-bit hex string to be converted
 * @returns hex integer version of input
 */
uint16 atoi_hex16(char *string);


/**
 * @brief Convert 32-bit hex string to an integer.
 *
 * @param string 32-bit hex string to be converted
 * @returns hex integer version of input
 */
uint32 atoi_hex32(char *string);


/**
 * 	@brief Print a pose structure
 *
 * 	Print a pose structure.  Prints in braces to be fancy.
 *	@param posePtr is a pointer to a pose.
 *	@returns void
 */
void posePrint(Pose* posePtr);

#endif /* BASICPRINTING_H_ */
