/**
 * @file radioProgramming.c
 *
 * @brief Functions that control the flow of the radio programming
 *
 * @since August 7, 2013
 * @author William Xie
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

uint8 robotIDMin = ROBOT_ID_MIN;
uint8 robotIDMax = ROBOT_ID_MAX;

// crcTable includes entries + crc for the entries
uint32 crcTableLocal[CRC_TABLE_SIZE + 1];
uint32 crcTableReceive[CRC_TABLE_SIZE + 1];
boolean segmentRequestTable[CRC_TABLE_SIZE];

// Segment buffer variables
uint32 lastModifiedSegmentNumber;


/*
 * @brief Set the query range for the host robot from user serial input
 *
 * @return void
 */
void radioProgrammingInput() {
	int tempVal;
	char c, buf[2];

	do {
		faultPrintSerial("Please select subnet (0 - 3)(default = ");
		faultPrintSerialDec((unsigned long)readBootloaderSubnet());
		faultPrintSerial(") \n");
		faultPrintSerial("Subnet: ");
		tempVal = -1;
		while (1) {
			c = (char)getSerialByte();
			buf[0] = c;
			buf[1] = '\0';
			// Parse number chars into an int
			if ((c >= '0') && (c <= '9')) {
				faultPrintSerial(buf);
				tempVal = (tempVal == -1) ? (int)(c - '0') : tempVal * 10 + (int)(c - '0');
			}
			// ESC to retype the number
			if (c == '\e') {
				faultPrintSerial("\n");
				tempVal = -1;
			}
			// Use default value if the user skips this step
			if ((c == '\r') || (c == '\n')) {
				if (tempVal == -1) {
					bootloaderSubnet = (unsigned long)readBootloaderSubnet();
					faultPrintSerialDec((unsigned long)bootloaderSubnet);
				} else {
					bootloaderSubnet = tempVal & RADIO_COMMAND_SUBNET_MASK;
				}
				faultPrintSerial("\n");
				break;
			}
		}
		if (tempVal > 3) {
			faultPrintSerial("Error: Invalid subnet.\n\n");
		}
	} while (tempVal > 3);
	do {
		faultPrintSerial("Please select the range of robot ID to be programmed (default = [");
		faultPrintSerialDec(ROBOT_ID_MIN);
		faultPrintSerial(", ");
		faultPrintSerialDec(ROBOT_ID_MAX);
		faultPrintSerial("])\n");
		faultPrintSerial("start: ");
		tempVal = -1;
		while (1) {
			c = (char)getSerialByte();
			buf[0] = c;
			buf[1] = '\0';
			// Parse number chars into an int
			if ((c >= '0') && (c <= '9')) {
				faultPrintSerial(buf);
				tempVal = (tempVal == -1) ? (int)(c - '0') : tempVal * 10 + (int)(c - '0');
			}
			// ESC to retype the number
			if (c == '\e') {
				faultPrintSerial("\n");
				tempVal = -1;
			}
			// Use default value if the user skips this step
			if ((c == '\r') || (c == '\n')) {
				if (tempVal == -1) {
					robotIDMin = ROBOT_ID_MIN;
					faultPrintSerialDec((unsigned long)robotIDMin);
				} else {
					robotIDMin = tempVal;
				}
				faultPrintSerial("\n");
				break;
			}
		}
		faultPrintSerial("end: ");
		tempVal = -1;
		while (1) {
			c = (char)getSerialByte();
			buf[0] = c;
			buf[1] = '\0';
			// Parse number chars into an int
			if ((c >= '0') && (c <= '9')) {
				faultPrintSerial(buf);
				tempVal = (tempVal == -1) ? (int)(c - '0') : tempVal * 10 + (int)(c - '0');
			}
			// ESC to retype the number
			if (c == '\e') {
				faultPrintSerial("\n");
				tempVal = -1;
			}
			// Use default value if the user skips this step
			if ((c == '\r') || (c == '\n')) {
				if (tempVal == -1) {
					robotIDMax = ROBOT_ID_MAX;
					faultPrintSerialDec((unsigned long)robotIDMax);
				} else {
					robotIDMax = tempVal;
				}
				faultPrintSerial("\n");
				break;
			}
		}
		if (robotIDMax < robotIDMin) {
			faultPrintSerial("Error: Invalid range.\n\n");
		}
	} while (robotIDMax < robotIDMin);
	faultPrintSerial("\n");
}


/*
 * @brief Construct a CRC table for all segments of the program
 * @param startingAddress The starting address of the program
 * @param length Length in KB of the program
 * @returns void
 */
void crcTableInit(uint32 startingAddress, uint16 length) {
	uint32 segmentNumber;

	// Default 4-KB segments
	for (segmentNumber = 0; segmentNumber < length; segmentNumber++) {
		crcTableLocal[segmentNumber] = crcSlow((uint8 *)(startingAddress + segmentNumber * CRC_SEGMENT_SIZE), CRC_SEGMENT_SIZE);
	}

	// Crc for the crcTable entries
	crcTableLocal[segmentNumber] = crcSlow((uint8 *)crcTableLocal, sizeof(crcTableLocal) - 1 * sizeof(uint32));
}


/*
 * @brief Retrieve the CRC value from CRC table. Should be called after crcTableInit
 * @param index The segment index
 * @returns The CRC-32 value
 */
uint32 getCRCTableLocalValue(uint16 index) {
	return crcTableLocal[index];
}


/*
 * @brief Packs the header of a program radio message
 * @param messagePtr Pointer to the radio message
 * @param type The type of radio message
 * @param segment The segment number
 * @param subsegment The subsegment number or destination robot ID depending on message type
 * @param robotID Sender's ID
 * @return void
 */
void packMessageHeader(RadioMessage *messagePtr, uint8 type, uint8 segment, uint8 subsegment, uint8 robotID) {
	messagePtr->program.type = (bootloaderSubnet << RADIO_COMMAND_SUBNET_SHIFTS | type);
	messagePtr->program.segment = segment;
	messagePtr->program.subsegment = subsegment;
	messagePtr->program.robotID = robotID;
}


/*
 * @brief Check if the message is a query response and radio message is from the expected robot and addressing the host.
 * This should be called by the host.
 * @param messagePtr Pointer to the radio message
 * @param expectedID ID of the expected remote robot
 * @param hostID ID of the host robot (selfID)
 * @returns TRUE, if the message is a query response to host from expected remote; False, otherwise
 */
boolean isValidQueryResponse(RadioMessage *messagePtr, uint8 expectedID, uint8 hostID) {
	boolean retval;

	retval = TRUE;
	if ((messagePtr->program.type & RADIO_COMMAND_TYPE_MASK) != RADIO_COMMAND_TYPE_QUERY_REQUEST ||
			messagePtr->program.subsegment != hostID ||
			messagePtr->program.robotID != expectedID) {
		retval = FALSE;
	}

	return retval;
}


/*
 * @brief Erase all required/mismatched segments to 0xff
 * This should be called after segmentRequestTable is constructed
 * @returns void
 */
void initRequiredSegments(void) {
	uint32 i, j, flashAddress;

	for (i = 0; i < CRC_TABLE_SIZE; i++) {
		if (segmentRequestTable[i]) {
			// Starting address of the segment
			flashAddress = OS_START_ADDRESS + CRC_SEGMENT_SIZE * i;
			for (j = 0; j < CRC_SEGMENT_SIZE_KB; j++) {
				FlashErase(flashAddress + j * 1024);
			}
		}
	}
}


/*
 * @brief Compare local and received crcTables to create a segment request table.
 * This should be called after crcTableReceive is complete and verified.
 * This should be called before starting to receive segments
 * @returns void
 */
void constructSegmentRequestTable(void) {
	uint32 i;

	for (i = 0; i < CRC_TABLE_SIZE; i++) {
		if (crcTableLocal[i] != crcTableReceive[i]) {
			segmentRequestTable[i] = TRUE;
		} else {
			segmentRequestTable[i] = FALSE;
		}
	}
}


/*
 * @brief Get the next segment index from the segment request table needed for to update the software.
 * This should be called after the request table is constructed.
 * @return The next required segment index. Return -1 if all required segments obtained
 */
int32 getSegementRequest(void) {
	int32 segmentIdx;
	uint32 i;

	segmentIdx = -1;
	for (i = 0; i < CRC_TABLE_SIZE; i++) {
		if (segmentRequestTable[i] == TRUE) {
			segmentIdx = i;
			break;
		}
	}

	return segmentIdx;
}


/*
 * @brief Recompute, update, and check the most recently changed segment's CRC
 * This should be called after the segment is finished receiving.
 * @param segmentNumber Index of the segment to be compute
 * @return TRUE, if it's match; FALSE, otherwise
 */
boolean checkAndUpdateLocalSegmentCrc(uint32 segmentNumber) {
	crcTableLocal[segmentNumber] = crcSlow((uint8 *)(OS_START_ADDRESS + segmentNumber * CRC_SEGMENT_SIZE), CRC_SEGMENT_SIZE);
	if (crcTableLocal[segmentNumber] == crcTableReceive[segmentNumber]) {
		return TRUE;
	} else {
		return FALSE;
	}
}


/*
 * @brief Broadcast local CRC Table
 * @return void
 */
void sendCRCTable() {
	RadioMessage outgoingRadioMessage;
	uint16 segmentNumber, payloadIdx;
	for (segmentNumber = 0; segmentNumber < sizeof(crcTableReceive) / RADIO_PROGRAM_MESSAGE_DATA_LENGTH + 1; segmentNumber++) {
		// RADIO_COMMAND_TYPE_CRC_TABLE: message_header[4] = [type + subnet, CRC package number, N/A, N/A]
		packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_CRC_TABLE, segmentNumber, 0, getSelfID());
		for (payloadIdx = 0; payloadIdx < RADIO_PROGRAM_MESSAGE_DATA_LENGTH; payloadIdx++) {
			if (segmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH + payloadIdx < sizeof(crcTableReceive)) {
				outgoingRadioMessage.program.data[payloadIdx] = *((uint8 *)crcTableLocal + segmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH + payloadIdx);
			} else {
				outgoingRadioMessage.program.data[payloadIdx] = 0;
			}
		}
		radioSendMessage(&outgoingRadioMessage);
	}
}


/*
 * @brief Send program segment, one way communication with no guarantee that the receiver gets all packages
 * @returns void
 */
void sendSegment(uint8 segmentNumber) {
	uint32 subsegmentNumber, length, payloadIdx;
	uint32 startingAddress;
	uint32 printCount = 0;

	startingAddress = OS_START_ADDRESS + CRC_SEGMENT_SIZE * segmentNumber;				// Start address of the segment
	length = CRC_SEGMENT_SIZE;															// segment size (default 4-KB)

	for (subsegmentNumber = 0; subsegmentNumber < length / RADIO_PROGRAM_MESSAGE_DATA_LENGTH + 1; subsegmentNumber++) {
		RadioMessage outgoingRadioMessage;

		// Pack
		packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_SEGMENTS, segmentNumber, subsegmentNumber, getSelfID());
		for (payloadIdx = 0; payloadIdx < RADIO_PROGRAM_MESSAGE_DATA_LENGTH; payloadIdx++) {
			if (subsegmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH + payloadIdx < length) {
				outgoingRadioMessage.program.data[payloadIdx] = *((uint8 *)(startingAddress + subsegmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH + payloadIdx));
			} else {
				outgoingRadioMessage.program.data[payloadIdx] = 0;
			}
		}

		// And blast
		radioSendMessage(&outgoingRadioMessage);

		// print '.' every 256 bytes
		if (((subsegmentNumber + 1) * RADIO_PROGRAM_MESSAGE_DATA_LENGTH - printCount * PRINT_BYTE_SIZE) >= PRINT_BYTE_SIZE) {
			faultPrintSerial(".");
			printCount++;
		}
	}
}


/*
 * @brief Unpack segment radio message and flash the subsegment to the proper locations in flash
 * @returns TRUE if the last subsegment is flashed
 */
boolean flashSubsegment(RadioMessage *messagePtr) {
	uint32 flashAddress;
	uint16 subsegmentNumber, segmentNumber;
	boolean retval;

	retval = FALSE;

	// Unpack
	segmentNumber = messagePtr->program.segment;
	subsegmentNumber = messagePtr->program.subsegment;
	flashAddress = OS_START_ADDRESS + segmentNumber * CRC_SEGMENT_SIZE + subsegmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH;
	// Last subsegment is smaller than RADIO_PROGRAM_MESSAGE_DATA_LENGTH but still multiple of 4
	if ((subsegmentNumber + 1) * RADIO_PROGRAM_MESSAGE_DATA_LENGTH <= CRC_SEGMENT_SIZE) {
		FlashProgram((unsigned long *)messagePtr->program.data, flashAddress, RADIO_PROGRAM_MESSAGE_DATA_LENGTH);
	} else {
		FlashProgram((unsigned long *)messagePtr->program.data, flashAddress, CRC_SEGMENT_SIZE % RADIO_PROGRAM_MESSAGE_DATA_LENGTH);
		retval = TRUE;
	}

	// Save most recent segment
	lastModifiedSegmentNumber = segmentNumber;

	return retval;
}


/*
 * @brief Recompute CRCTableLocal and check for any mismatched segments. If everything matches, the program is successfully transferred.
 * Note, this is a time consuming task. It should be called as a final check.
 * This should be called after CRCTableRecieve is finished.
 * @return TRUE, if the crcTables match. FALSE, otherwise.
 */
boolean isProgramFinished() {
	boolean retval;

	retval = FALSE;

	crcTableInit(OS_START_ADDRESS, CRC_TABLE_SIZE);
	constructSegmentRequestTable();
	if (getSegementRequest() == -1) {
		retval = TRUE;
	}
	return retval;
}


/*
 * @brief Host mode routine
 * @return void
 */
void radioHost(void) {
	RadioMessage incomingRadioMessage, outgoingRadioMessage;
	uint32 i;
	uint32 broadcastCount;

	// Broadcast 'Program Time' message to bring all active robots in the same subnet to bootloader radio receiver mode
	faultPrintSerial("Broadcasting PROGRAM TIME command\n");
	for (broadcastCount = 0; broadcastCount < MAX_PROGRAM_TIME_COUNT; broadcastCount++) {
		// RADIO_COMMAND_TYPE_PROGRAM_TIME: message_header[4] = [type + subnet, id_range_min, id_range_max, sender ID]
		RadioMessage outgoingRadioMessage;
#if defined(RONE_V9)
		packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_PROGRAM_TIME_V11, robotIDMin, robotIDMax, getSelfID());
#elif defined(RONE_V12)
		packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_PROGRAM_TIME_V12, robotIDMin, robotIDMax, getSelfID());
#endif
		radioSendMessage(&outgoingRadioMessage);
		blinkyLedToggle();
		systemDelay(50);
	}
	
	// Broadcast Segment-CRC once
	faultPrintSerial("Broadcasting CRC Table\n");
	for (broadcastCount = 0; broadcastCount < MAX_CRC_BROADCAST; broadcastCount++) {
		sendCRCTable();
		blinkyLedToggle();
		systemDelay(50);
	}

	systemDelay(TRANSITION_DELAY_MS);	// Short wait before first message

	// Query robots for segment requests or crcTable rebroadcast
	uint32 robotIDCount;
	uint32 requestAttempts, segmentRequestCounter;
	uint8 previousSegmentNumber = CRC_TABLE_SEGMENT_IDX;

	for (robotIDCount = robotIDMin; robotIDCount < robotIDMax + 1; robotIDCount++) {
		faultPrintSerial("Querying robot ");
		faultPrintSerialDec((unsigned long)robotIDCount);

		// RADIO_COMMAND_TYPE_QUERY_REQUEST: message_header[4] = [type + subnet, segment request number, destination robot ID, sender ID]
		packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_QUERY_REQUEST, 0, robotIDCount, getSelfID());
		requestAttempts = 0;
		segmentRequestCounter = 0;

		// Ping remote robot until timeout
		while (requestAttempts++ < MAX_QUERY_REQUEST) {
			// Send query request
			radioSendMessage(&outgoingRadioMessage);
			faultPrintSerial(".");
			// Check for reply
			for (i = 0; i < MAX_QUERY_REQUEST_WAIT; i++) {
				while (radioRecvMessage(&incomingRadioMessage)) {
					if (isValidQueryResponse(&incomingRadioMessage, robotIDCount, getSelfID())) {
						// Got a valid response from target remote robot
						// Respond to query request response with crc table or segment
						if (incomingRadioMessage.program.segment == CRC_TABLE_SEGMENT_IDX) {
							faultPrintSerial("\n");
							faultPrintSerial("Broadcasting CRC Table\n");
							sendCRCTable();
						} else {
							faultPrintSerial("\n");
							faultPrintSerial("Broadcasting Segment ");
							faultPrintSerialDec((unsigned long)incomingRadioMessage.program.segment);
							sendSegment(incomingRadioMessage.program.segment);
							faultPrintSerial("\n");
						}
						// Reset requestAttempts when the host gets a response from remote
						requestAttempts = 0;

						// Check if we are sending the same segment as before
						if (previousSegmentNumber == incomingRadioMessage.program.segment) {
							// Timeout if the same segment is being requested too many times
							if (segmentRequestCounter > SEGMENT_REQUEST_TIMEOUT) {
								// Skip current queried robot if it is stuck
								faultPrintSerial("Error: Robot ");
								faultPrintSerialDec(robotIDCount);
								faultPrintSerial(" timed out\n");
								i = MAX_QUERY_REQUEST_WAIT;
								requestAttempts = MAX_QUERY_REQUEST;
								segmentRequestCounter = 0;
								break;
							} else {
								segmentRequestCounter++;
							}
						} else {
							previousSegmentNumber = incomingRadioMessage.program.segment;
							segmentRequestCounter = 0;
						}


					}
				}
			}
		}
		faultPrintSerial("\n");
		blinkyLedToggle();
	}

	// Reboot all robots
	faultPrintSerial("Broadcasting reboot command\n");
	for (broadcastCount = 0; broadcastCount < MAX_REBOOT_COUNT; broadcastCount++) {
		// RADIO_COMMAND_TYPE_REBOOT: message_header[4] = [type + subnet, 0, 0, sender ID]
		RadioMessage outgoingRadioMessage;
		packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_REBOOT, 0, 0, getSelfID());
		radioSendMessage(&outgoingRadioMessage);
		blinkyLedToggle();
		systemDelay(50);
	}
}


/*
 * @brief Remote mode routine
 * @return void
 */
void radioRemote(void) {
	RadioMessage outgoingRadioMessage;
	int32 segmentIdx;
	uint32 byteCount, i;
	uint16 crcSegmentNumber, payloadIdx;
	boolean crcTableReceiveFinished, programFinished;

	// Initialize variables
	crcTableReceiveFinished = FALSE;
	programFinished = FALSE;
	byteCount = 0;

	// Enter receive message loop
	while (!programFinished) {
		RadioMessage incomingMessage;
		if (radioRecvMessage(&incomingMessage)) {
			switch (incomingMessage.program.type & RADIO_COMMAND_TYPE_MASK) {
			case RADIO_COMMAND_TYPE_CRC_TABLE: {
				// Only receive crcTable if the table is incomplete
				if (crcTableReceiveFinished) {
					break;
				}
				// Unpack
				crcSegmentNumber = incomingMessage.program.segment;
				for (payloadIdx = 0; payloadIdx < RADIO_PROGRAM_MESSAGE_DATA_LENGTH; payloadIdx++) {
					if (payloadIdx + crcSegmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH < sizeof(crcTableReceive)) {
						*((uint8 *)crcTableReceive + crcSegmentNumber * RADIO_PROGRAM_MESSAGE_DATA_LENGTH + payloadIdx) = incomingMessage.program.data[payloadIdx];
						byteCount++;
					} else {
						break;
					}
				}
				// Check for crcTable integrity under two conditions: received expected number of bytes or all entries are non-zero
				for (i = 0; i < sizeof(crcTableReceive) / sizeof(uint32); i++) {
					if (crcTableReceive[i] == 0) {
						break;
					}
				}
				if (byteCount >= sizeof(crcTableReceive) ||
						i == sizeof(crcTableReceive)/sizeof(uint32)) {
					// Compute and compare crc for crcTable
					uint32 crcTableRecieveCrc;
					crcTableRecieveCrc = crcSlow((uint8 *)crcTableReceive, sizeof(crcTableReceive) - 1 * sizeof(uint32));
					if (crcTableRecieveCrc == crcTableReceive[sizeof(crcTableReceive)/sizeof(uint32) - 1]) {
						// Clear flash for all needed segments and prepare to receive
						faultPrintSerial("CRC Table complete\n");
						crcTableReceiveFinished = TRUE;
						constructSegmentRequestTable();
						initRequiredSegments();
					}
				}
				break;
			}
			case RADIO_COMMAND_TYPE_QUERY_REQUEST: {
				// Destination ID must match self
				if (incomingMessage.program.subsegment == getSelfID()) {
					if (!crcTableReceiveFinished) {
						// If CRC Table not complete, request for it
						packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_QUERY_REQUEST, CRC_TABLE_SEGMENT_IDX, incomingMessage.program.robotID, getSelfID());
						radioSendMessage(&outgoingRadioMessage);
					} else {
						// Check last received segment's CRC before requesting
						if (checkAndUpdateLocalSegmentCrc(lastModifiedSegmentNumber)) {
							segmentRequestTable[lastModifiedSegmentNumber] = FALSE;
						}
						// Find next needed segment and request for it
						segmentIdx = getSegementRequest();
						if(segmentIdx >= 0) {
							packMessageHeader(&outgoingRadioMessage, RADIO_COMMAND_TYPE_QUERY_REQUEST, segmentIdx, incomingMessage.program.robotID, getSelfID());
							radioSendMessage(&outgoingRadioMessage);
						} else {
							// All segments received, do final checks
							programFinished = isProgramFinished();
							// If programFinished, this robot will not longer respond to requests
						}
					}
				}
				break;
			}
			case RADIO_COMMAND_TYPE_SEGMENTS: {
				boolean segmentFinished;
				// Only receive segments when the crcTableReceive is complete
				if (!crcTableReceiveFinished) {
					break;
				}
				// Only flash subsegment if segment is needed
				if (!segmentRequestTable[incomingMessage.program.segment]){
					break;
				}
				segmentFinished = flashSubsegment(&incomingMessage);
				if (segmentFinished) {
					blinkyLedToggle(1);
					if (checkAndUpdateLocalSegmentCrc(lastModifiedSegmentNumber)) {
						// Update segment request table and increment
						segmentRequestTable[lastModifiedSegmentNumber] = FALSE;
						faultPrintSerial("Segment ");
						faultPrintSerialDec(lastModifiedSegmentNumber);
						faultPrintSerial(" received\n");
						// Check if this is the last segment needed
						if (getSegementRequest() == -1) {
							programFinished = isProgramFinished();
						}
					} else {
						// Message corrupted or incomplete
						faultPrintSerial("Segment ");
						faultPrintSerialDec(lastModifiedSegmentNumber);
						faultPrintSerial(" corrupted\n");
					}
				}
				break;
			}
			}
		}
	}

	faultPrintSerial("All segments received\n");

	// Finished programming, wait for reboot message
	blinkyLedSet(1);
	while (1) {
		RadioMessage incomingMessage;
		if (radioRecvMessage(&incomingMessage) && ((incomingMessage.program.type & RADIO_COMMAND_TYPE_MASK) == RADIO_COMMAND_TYPE_REBOOT)) {
			break;
		}
	}
}


/*
 * @brief Limbo mode routine
 * @return void
 */
void radioLimbo() {
	blinkyLedSet(1);
	while (1) {
		RadioMessage incomingMessage;
		if (radioRecvMessage(&incomingMessage) && ((incomingMessage.program.type & RADIO_COMMAND_TYPE_MASK) == RADIO_COMMAND_TYPE_REBOOT)) {
			break;
		}
	}
}


/*
 * @brief Compute local CRCTable and run host/radio mode code.
 *
 * @param mode select the host or remote mode (see header file)
 * @return void
 */
void radioProgramming(uint8 mode) {
	// Create local crc table
	crcTableInit(OS_START_ADDRESS, CRC_TABLE_SIZE);

	faultPrintSerial("Local CRC computed\n");
	if (mode == RADIO_PROGRAMMING_HOST) {
		radioHost();
	} else if (mode == RADIO_PROGRAMMING_REMOTE) {
		radioRemote();
	} else if (mode == RADIO_PROGRAMMING_LIMBO) {
		radioLimbo();
	}

	//Reset robot after programming
	faultPrintSerial("Rebooting\n");
	writeBootloaderState(BL_STATE_NORMAL);
	hardReset();
}
