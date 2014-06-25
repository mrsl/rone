/**
 * @file xmodem.c
 *
 * @brief XMODEM binary download functions
 *
 * @since July 29, 2013
 * @author William Xie, James McLurkin
 */

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/flash.h"

#include "System/bootloader.h"

/*
 * @brief Grabs binary file (robot software) via XMODEM and writes to flash starting at address OS_START_ADDRESS.
 * It uses XMODEM 1024 protocol through secureCRT.
 * @returns 0 if the binary is successfully installed. For other error values, see bootloader.h
 */
uint8 xmodemDownalodBinary(void) {
    int c;
    uint32 i, j, totalErrors, osMemeoryLocation;
    uint16 dataPacketSize;
    uint8 checkSum, packetNumer, tempByte, error;
    uint8 dataPacket[MAX_PACKET_SIZE];
    boolean packetError, endOfFile, gotEOT;

    totalErrors = 0;
    endOfFile = FALSE;
    osMemeoryLocation = (uint32)OS_START_ADDRESS;

    // For every packet
    while (!(endOfFile) && (totalErrors < RX_MAX_ERRORS)) {
        dataPacketSize = 0;
        checkSum = 0;
        error = 0;
        packetError = FALSE;
        gotEOT = FALSE;

        // Send a NAK every 1/2 second until the transmission starts.
        if ((c = getSerialByteWithTimeout(RX_TIMEOUT_US)) >= 0) {
        	// First byte determines the packet type
        	tempByte = (uint8)c;
            switch (tempByte)  {
            case SOH:
            	// This is the beginning of a 128 byte packet
            	dataPacketSize = 128;
            	break;
            case STX:
            	// This is the beginning of a 1024 byte packet
            	dataPacketSize = 1024;
            	break;
            case EOT:
            	// This transmission is finished.
            	gotEOT = TRUE;
            	endOfFile = TRUE;
            	break;
            case CAN:
            	// This transmission has been canceled.
            	packetError = TRUE;
            	endOfFile = TRUE;
            	error = ERROR_CANCEL;
            	break;
            default:
            	// Unknown error. Abort.
            	packetError = TRUE;
            	endOfFile = TRUE;
            	error = ERROR_UNKOWN;
            	break;
            }

        	// EOT
            if (gotEOT) {
            	sendByte(ACK);
            	break;
            }

            // Get packet data if there's no error
            if (error == 0) {
            	// Get packet number, inverse Packet number, payload[dataBytesInPacket], checksum
            	for (i = 0; i < dataPacketSize + 3; i++) {
            		// Poll interrupt pin until the FIFO reaches the triggering level.
            		while (!GPIOPinRead(UART0_BASE, UART_INT_RX));
            		c = getSerialByteNonblocking();
            		dataPacket[i] = (uint8)c;
            	}

            	packetNumer = dataPacket[0];

            	if (packetNumer != (uint8)(~dataPacket[1])) {
            		// Mismatch in packet ID and inverse packet ID
            		packetError = TRUE;
            	}

            	for (j = 2; j < dataPacketSize + 2; j++) {
            		checkSum += dataPacket[j];
            	}

            	// The last byte is the checksum byte
            	if (checkSum != dataPacket[dataPacketSize + 2]) {
            		packetError = TRUE;
            	}
            }

            if (packetError) {
                // Some kind of packet error. Flush RX buffer and retry.
            	totalErrors++;
            	error = ERROR_TIMEOUT;
            	sendByte(NAK);
            } else if (dataPacketSize > 0) {
                // Received a complete xmodem packet, program flash, and signal the sender.
            	FlashErase(osMemeoryLocation);
            	FlashProgram((uint32 *)(dataPacket + 2), osMemeoryLocation, dataPacketSize);
            	osMemeoryLocation += dataPacketSize;
            	blinkyLedToggle();
            	sendByte(ACK);
            }
        } else {
            // Received nothing. Signal NAK.
        	sendByte(NAK);
        }
    }

    // Give SecureCRT some time to close the dialog box
    systemDelay(300);

    return error;
}
