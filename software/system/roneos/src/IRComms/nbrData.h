/**
 * @file nbrData.h
 * @brief Init, setter, and getter functions for the nbrData structure.
 *
 * nbrData is ideal for keeping track of messages sent between robots.
 *
 * @since February 12, 2013
 * @author James McLurkin
 */

#ifndef NBRDATA_H_
#define NBRDATA_H_

/**
 * @brief Create a neighbor data.  This is transmitted using the radio each
 * neighbor round.
 *
 * This function is called only once to init each neighbor data
 * @param nbrDataPtr the pointer to the neighbor data
 * @param name the name for the type of neighbor data being stored
 * @param size the size of the neighbor data
 * @param value the value of the neighbor data
 * @returns void
 */
void nbrDataCreate(NbrData* nbrDataPtr, const char* name, uint8 size, uint8 value);

/**
 * @brief Create a neighbor data (16-bit).  This is transmitted using the radio each
 * neighbor round.
 *
 * This function is called only once to init each neighbor data
 * @param nbrDataPtrH the pointer to the neighbor data
 * @param nbrDataPtrL the pointer to the neighbor data
 * @param nameH the name for the type of neighbor data being stored
 * @param nameL the name for the type of neighbor data being stored
 * @param value the value of the neighbor data
 * @returns void
 */
void nbrDataCreate16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL, const char* nameH, const char* nameL, uint16 value);

/**
 */
void nbrDataCreate32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL,
	const char* nameHH, const char* nameHL, const char* nameLH, const char* nameLL, uint32 value);

/**
 * @brief Create a IR neighbor data.  This is transmitted via IR.  The neighbor
 * system makes one 7-bit data by default for the robot ID.  Only data that need to share
 * the fate of an IR message should go here.  The rest should use radio (See nbrDataCreate())
 *
 * This function is called only once to init each neighbor data
 * @param nbrDataPtr the pointer to the neighbor data
 * @param name the name for the neighbor data
 * @param size the size of the neighbor data
 * @param value the value of the neighbor data
 * @returns void
 */
void nbrDataCreateIR(NbrData* nbrDataPtr, const char* name, uint8 size, uint8 value);


/**
 * @brief Set the value of the neighbor data.
 *
 * \internal
 * Mask is 0XFF shifted right the difference between the size of the message pointer and 8 bits.
 * Mask ensures that we only store the same number of bits of value as message pointer size.
 * Size must be 8 or smaller.
 * \endinternal
 *
 * @param nbrDataPtr the point to the nbrMsgPtr to be set. The size of the pointer must be 8 or smaller.
 * @param value the value the nbrMsgPtr should be set to
 * @returns void
 */
void nbrDataSet(NbrData* nbrDataPtr, uint8 value);

/**
 * @brief Set the value of the neighbor data.
 *
 * @param nbrDataPtrH the point to the nbrMsgPtr to be set.
 * @param nbrDataPtrL the point to the nbrMsgPtr to be set.
 * @param value the value the nbrMsgPtr should be set to
 * @returns void
 */
void nbrDataSet16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL, uint16 value);

/**
 */
void nbrDataSet32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL,
	uint32 value);

/**
 *@brief Get value in a neighbor message.
 *
 *@param nbrDataPtr pointer to neighbor message
 *@returns if nbrDataPtr is valid, value; else, 0
 */
uint8 nbrDataGet(NbrData* nbrDataPtr);

/**
 *@brief Get value in a neighbor message.
 *
 *@param nbrDataPtrH pointer to neighbor message
 *@param nbrDataPtrL pointer to neighbor message
 *@returns value
 */
uint16 nbrDataGet16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL);

uint32 nbrDataGet32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL);

/**
 * @brief Get value of the local nbrData
 *
 * @param nbrDataPtr pointer to neighbor message
 * @param nbrPtr neighbor pointer
 * @returns if nbrPtr and nbrMsgPtr are valid, value; else, 0
 */
uint8 nbrDataGetNbr(NbrData* nbrDataPtr, Nbr* nbrPtr);

/**
 * @brief Get value of the local nbrData
 *
 * @param nbrDataPtrH pointer to neighbor message
 * @param nbrDataPtrL pointer to neighbor message
 * @param nbrPtr neighbor pointer
 * @returns value
 */
uint16 nbrDataGetNbr16(NbrData* nbrDataPtrH, NbrData* nbrDataPtrL, Nbr* nbrPtr);

uint32 nbrDataGetNbr32(NbrData* nbrDataPtrHH, NbrData* nbrDataPtrHL, NbrData* nbrDataPtrLH, NbrData* nbrDataPtrLL, Nbr* nbrPtr);

/**
 * @brief Get name of neighbor message.
 *
 * As specified in NbrMsgCreate()
 * @param nbrDataPtr pointer to neighbor message
 * @returns if nbrMsgPtr is valid, name; else, 0
 */
const char* nbrDataGetName(NbrData* nbrDataPtr);


/**
 * @brief Get size of neighbor message.
 *
 * As specified in NbrMsgCreate()
 * @param nbrDataPtr pointer to neighbor message
 * @returns if nbrMsgPtr is valid, size; else, 0
 */
uint8 nbrDataGetSize(NbrData* nbrDataPtr);


/**
 * @brief Print all the neighbor messages of a given neighbor.
 *
 * @param nbrPtr neighbor pointer
 * @returns void
 */
void nbrDataPrintNbr(Nbr* nbrPtr);


/**
 * @brief Print all the neighbor messages of the given neighbor.  Prints in verbose format, with names on separate lines.
 *
 * @param nbrPtr neighbor pointer
 * @returns void
 */
void nbrDataPrintNbrVerbose(Nbr* nbrPtr);


/**
 * @brief Print the names of all the nbr data in a header suitable for data logging
 *
 * @returns void
 */
void nbrDataPrintHeaders(void);


/**
 * @brief Get the total number of nbr data that have been created
 *
 * @returns the count of nbrData objects
 */
uint8 nbrDataCount(void);


#endif /* NBRDATA_H_ */
