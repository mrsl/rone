/*
 * SPI.h
 *
 *  Created on: Oct 30, 2013
 *  Updated on: Mar 3, 2014
 *      Author: Taylor
 */

#ifndef SPI_H_
#define SPI_H_

#include "typeDefs.h"

/* SPI settings */
#define SPI_A0_CLK_BIT		BIT4
#define SPI_A0_SEL_BIT		BIT5
#define SPI_A0_MISO_BIT		BIT1
#define SPI_A0_MOSI_BIT		BIT2

/* SPI message */
#define GRIPPER_CODE_LENGTH		3		// 3 code bytes for syncing
#define GRIPPER_PAYLOAD_LENGTH	3		// 8 data bytes
#define GRIPPER_MSG_LENGTH		(GRIPPER_PAYLOAD_LENGTH + GRIPPER_CODE_LENGTH + 1) // sync bytes + data bytes + check sum byte
// Incoming and Outgoing
#define	GRIPPER_MSG_SERVO_VALUE_IDX		0 + GRIPPER_CODE_LENGTH
// Outgoing only
#define	GRIPPER_MSG_CURRENT_VALUE_IDX	1 + GRIPPER_CODE_LENGTH
#define	GRIPPER_MSG_IO_IDX				2 + GRIPPER_CODE_LENGTH

//High force bits packaged with mode
#define FORCE_BIT			BIT7
#define SOFT_STOP1			BIT6
#define SOFT_STOP2			BIT5

#define FORCE_BIT_OFFSET	7
#define SOFT_STOP0_OFFSET	6
#define SOFT_STOP1_OFFSET	5


void initSPI(void);
//uint8 SPIChecksum(uint8* msg);
//void shiftBuffer(uint8* msg);
void initMessage(void);
void messagePackOut(void);
void messagePackCurrentAverage(current currentValue);
void messagePackServoValues(volatile uint8 servoValue);
uint8 messageGetIO(void);
uint8 messageGetServoValue(void);
void messagePackIO(uint8 message);


#endif /* SPI_H_ */
