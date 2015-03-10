/*
 * gripper.h
 *
 *  Created on: Mar 6, 2015
 *      Author: Golnaz
 */

#ifndef GRIPPER_H_
#define GRIPPER_H_

#include "roneos.h"

#define GRIPPER_TASK_PERIOD	75

#define GRIPPER_CODE_LENGTH		3
#define GRIPPER_PAYLOAD_LENGTH	EXPAND0_PAYLOAD_LENGTH
#define GRIPPER_MSG_LENGTH		(GRIPPER_PAYLOAD_LENGTH + GRIPPER_CODE_LENGTH + 1)

#define GRIPPER_HISTORY_SIZE	20

#define GRIPPER_IDLE_SERVO		90

#define GRIPPER_GRIPPED_CURRENT	30

#define GRIPPER_IDLE	0
#define GRIPPER_TRYGRIP	1

typedef union gripperMsgCast gripperMsg;

struct gripperMsg {
	uint8 servo;
	uint8 current;
	uint8 : 5;
	uint8 stop1 : 1;
	uint8 stop0 : 1;
	uint8 force : 1;
};

union gripperMsgCast {
	struct gripperMsg msg;
	uint8 bytes[EXPAND0_PAYLOAD_LENGTH];
};

typedef struct gripperCalibrationData gripperCalibrationData;
struct gripperCalibrationData {
	uint8 CWMax;
	uint8 CCWMax;
	uint8 CWOff;
	uint8 CCWOff;
	uint8 CWDone : 1;
	uint8 CCWDone : 1;
};

void gripperInit(void);

#endif /* GRIPPER_H_ */
