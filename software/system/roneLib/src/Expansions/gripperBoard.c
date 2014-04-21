/**
 * @file gripperBoard.h
 * @brief Controls the gripper extension board functions
 * @details Functionality includes controlling servos and motors
 * @since June 24, 2013
 * @author William Xie
 */
#include <string.h>

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

uint8 gripperMessageOut[GRIPPER_PAYLOAD_LENGTH];
uint8 gripperMessageIn[GRIPPER_PAYLOAD_LENGTH];

/*
 *  @brief Initialize the message and communication protocol to the gripperboard
 *  @param mode Select software the gripperboard is currently using:
 *  GRIPPER_MODE_SERVOS = 3 servos
 *  GRIPPER_MODE_MOTORS = 2 motors + 1 servo
 *  GRIPPER_MODE_STEPPER = 1 stepper motor + 1 servo (not implemented yet)
 *
 *  @returns void
 */
void gripperBoardInit() {
	expand0Init();
	uint8 i;
	// Clear message
	for (i = 0; i < GRIPPER_PAYLOAD_LENGTH; i++) {
		gripperMessageOut[i] = 0;
	}

	expand0MsgWrite(gripperMessageOut);
}

/*
 *  @brief Change the position of the servo
 *  @details The range of the value translates into [0, 180] degree for the servo
 *  @param value The new position (range = [0, 255])
 *  @returns void
 */
void gripperBoardSetServo(uint8 value) {
		gripperMessageOut[GRIPPER_MSG_SERVO_VALUE_IDX] = value;
		expand0MsgWrite(gripperMessageOut);
}


/*
 *  @brief Retrieve the current value from the gripperboard
 *  @details The experimental 'gripped' value in the current scale is 50
 *  @returns The average current value (range = [0, 255]).
 */
uint8 gripperBoardGetCurrent() {
	expand0MsgRead(gripperMessageIn);
	return gripperMessageIn[GRIPPER_MSG_CURRENT_VALUE_IDX];
}

/*
 *  @brief Retrieve the servo value from the gripperboard
 *  @returns The servo value (range = [0, 255]).
 */
uint8 gripperBoardGetServo() {
	expand0MsgRead(gripperMessageIn);
	return gripperMessageIn[GRIPPER_MSG_SERVO_VALUE_IDX];
}


/*
 *  @brief Check to see if the force is high
 *  @returns Value of force bit, 0 or 1
 */
uint8 gripperBoardGetForce() {
	expand0MsgRead(gripperMessageIn);
	return ((gripperMessageIn[GRIPPER_MSG_IO_IDX] && FORCE_BIT) >> FORCE_BIT_OFFSET);
}

/*
 *  @brief Check to see if we have a soft stop
 *  @param takes in 0 or 1
 *  @returns Value of force bit, 0 or 1
 */
uint8 gripperBoardGetStop(uint8 value) {
	expand0MsgRead(gripperMessageIn);
	if (value) {
		return ((gripperMessageIn[GRIPPER_MSG_IO_IDX] && SOFT_STOP1) >> SOFT_STOP1_OFFSET);
	} else {
		return ((gripperMessageIn[GRIPPER_MSG_IO_IDX] && SOFT_STOP0) >> SOFT_STOP0_OFFSET);
	}

}

