/**
 * @file xmodem.h
 *
 * @brief XMODEM binary download functions
 *
 * @since July 29, 2013
 * @author William Xie, James McLurkin
 */

#ifndef XMODEM_H_
#define XMODEM_H_

#define SOH 0x01
#define STX 0x02
#define EOT	0x04
#define ACK	0x06
#define NAK	0x15
#define CAN	0x18

#define RX_MAX_ERRORS 30
#define RX_TIMEOUT_US 50000
#define MAX_PACKET_SIZE (1024 + 3)

// Error values
#define ERROR_CANCEL	1
#define ERROR_TIMEOUT	2
#define ERROR_UNKOWN	3

/**
 * @brief Grabs binary file (robot software) via XMODEM and writes to flash starting at address OS_START_ADDRESS.
 * It uses XMODEM 1024 protocol through secureCRT.
 * @returns 0 if the binary is successfully installed. For other error values, see bootloader.h
 */
uint8 xmodemDownalodBinary(void);


#endif /* XMODEM_H_ */
