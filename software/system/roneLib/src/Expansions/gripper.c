/*
 * gripper.c
 *
 *  Created on: Mar 6, 2015
 *      Author: Golnaz
 */

#include "gripper.h"

static gripperMsg gripperOutMsg;
static gripperMsg gripperInMsg;

uint8 gripperGripped = 0;

gripperCalibrationData gripperCalibration;

uint8 gripperCurrentPast[GRIPPER_HISTORY_SIZE];
uint8 gripperCurrentPastIdx = 0;

uint8 gripperMode = GRIPPER_IDLE;

void gripperInitMsg(gripperMsg *msg) {
	msg->msg.servo = 0;
	msg->msg.current = 0;
	msg->msg.force = 0;
	msg->msg.stop0 = 0;
	msg->msg.stop1 = 0;
}

void gripperPrintMsg(gripperMsg *msg) {
	cprintf(
		"%u, %u, %u, %u, %u\n",
		msg->msg.servo,
		msg->msg.current,
		msg->msg.force,
		msg->msg.stop0,
		msg->msg.stop1
	);
}

uint8 gripperIsGripped(void) {
	return gripperGripped;
}

void gripperInitCurrentHistory(void) {
	uint8 idx;
	for (idx = 0; idx < GRIPPER_HISTORY_SIZE; idx++) {
		gripperCurrentPast[idx] = 0;
	}
}

void gripperUpdateCurrentHistory(void) {
	gripperCurrentPast[gripperCurrentPastIdx] = gripperInMsg.msg.current;
	gripperCurrentPastIdx = (gripperCurrentPastIdx + 1) % GRIPPER_HISTORY_SIZE;
}

uint8 gripperGetAverageCurrent(void) {
	float currentAvg = 0.;

	uint8 idx;
	for (idx = 0; idx < GRIPPER_HISTORY_SIZE; idx++) {
		currentAvg += (float) gripperCurrentPast[idx];
	}

	currentAvg /= GRIPPER_HISTORY_SIZE;

	return (uint8) currentAvg;
}


void gripperSetDestinationServo(uint8 servoPos) {
	gripperOutMsg.msg.servo = servoPos;
}

void gripperInitCalibration(void) {
	gripperCalibration.CWMax = 0;
	gripperCalibration.CCWMax = 0;
	gripperCalibration.CWOff = 0;
	gripperCalibration.CCWOff = 0;
	gripperCalibration.CWDone = 0;
	gripperCalibration.CCWDone = 0;
}

uint8 gripperIsCalibrated(void) {
	return gripperCalibration.CWDone && gripperCalibration.CCWDone;
}

void gripperCalibrationRoutine(void) {
	if (!gripperCalibration.CWDone) {
		gripperSetDestinationServo(0);

		if (gripperGetAverageCurrent() > GRIPPER_GRIPPED_CURRENT) {
			gripperCalibration.CWMax = gripperInMsg.msg.servo;

		}
	} else if (!gripperCalibration.CCWDone) {

	}
}

uint8 gripperGetDestinationServo(void) {
	return gripperOutMsg.msg.servo;
}

void gripperGripCW(void) {
	gripperSetDestinationServo(0);
}

void gripperGripCCW(void) {
	gripperSetDestinationServo(180);
}

void gripperSetMode(uint8 mode) {
	gripperMode = mode;
}

void gripperLoop(void *args) {
	for (;;) {
		portTickType lastWakeTime = osTaskGetTickCount();

		expand0MsgRead(gripperInMsg.bytes);
		gripperUpdateCurrentHistory();

		if (gripperGetAverageCurrent() > GRIPPER_GRIPPED_CURRENT) {
			gripperGripped = 1;
		} else {
			gripperGripped = 0;
		}

		expand0MsgWrite(gripperOutMsg.bytes);

		osTaskDelayUntil(&lastWakeTime, GRIPPER_TASK_PERIOD);
	}
}

void gripperInit(void) {
	expand0Init();

	gripperInitMsg(&gripperOutMsg);
	gripperInitMsg(&gripperInMsg);

	gripperInitCurrentHistory();

	gripperInitCalibration();

	osTaskCreate(gripperLoop, "gripper", 1024, NULL, NEIGHBORS_TASK_PRIORITY);
}
