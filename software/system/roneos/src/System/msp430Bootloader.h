/**
 * @file msp430Bootloader.h
 * @brief  boot loader functions on MSP430
 *  @since Jul 31, 2012
 *  @author MRSL
 *  \internal
 *  TODO FIND mrdouglass
 *  \endinternal
 */

#ifndef MSP430BOOTLOADER_H_
#define MSP430BOOTLOADER_H_

/**
 * @brief Gets msp430 local software version
 * @returns msp430 local software version
 */
uint8 msp430BSLGetLocalVersionNumber(void);

/**
 * @brief Gets msp430 local hardware version
 * @returns msp430 local hardware version
 */
uint8 msp430BSLGetLocalVersionHardwareNumber(void);

/**
 * @brief This method is used to initialize the MSP430 and interrupts for the BSL
 * @returns void
 */
void msp430BSLInit(void);

#endif /* MSP430BOOTLOADER_H_ */
