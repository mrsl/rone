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

static uint8 gripperMessageOut[GRIPPER_PAYLOAD_LENGTH];
static uint8 gripperMessageIn[GRIPPER_PAYLOAD_LENGTH];

static uint32 gripperPeriod = GRIPPER_PERIOD_LENGTH;

static int state;
static boolean servoComplete;

static gripperState gripper;
static SerialCmd serialCmdSG;

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

	gripperDataInit();

	state = GRIPPER_IDLE;
	servoComplete = TRUE;

	// serial IO gripper information
	serialCommandAdd(&serialCmdSG, "sg", serialCmdSGFunc);
	// Create gripper task
	osTaskCreate(gripperTask, "gripper", 2048, NULL, NEIGHBORS_TASK_PRIORITY);
}

void gripperDataInit() {
	gripper.current = 0;
	gripper.readServo = 0;
	gripper.currServo = 90;
	gripper.goalServo = 0;
	gripper.force = 0;
	gripper.stop0 = 0;
	gripper.stop1 = 0;
	gripper.grippedHistory = 0;
	gripper.gripped = 0;
	gripper.calibrated = 0;
	gripper.calibratedMaxAngle = GRIPPER_DEFUALT_MAX_ANGLE;
	gripper.calibratedMinAngle = GRIPPER_DEFUALT_MIN_ANGLE;
	gripper.calibratedRestAngle = GRIPPER_DEFUALT_REST;
	gripper.ungrip = 0;
	gripper.pastPlace = 0;

}

void gripperTask(void* parameters) {
	portTickType lastWakeTime = osTaskGetTickCount();

	for (;;) {
		expand0MsgRead(gripperMessageIn);

		/* Read in all state values */
		gripper.current = gripperMessageIn[GRIPPER_MSG_CURRENT_VALUE_IDX];
		gripper.readServo = gripperMessageIn[GRIPPER_MSG_SERVO_VALUE_IDX];
		gripper.force = (gripperMessageIn[GRIPPER_MSG_IO_IDX] & FORCE_BIT) >> FORCE_BIT_OFFSET;
		gripper.stop0 = (gripperMessageIn[GRIPPER_MSG_IO_IDX] & SOFT_STOP0) >> SOFT_STOP0_OFFSET;
		gripper.stop1 = (gripperMessageIn[GRIPPER_MSG_IO_IDX] & SOFT_STOP1) >> SOFT_STOP1_OFFSET;

		if (abs(gripper.readServo - gripper.goalServo) < 2)
			servoComplete = TRUE;

		/* Determine if gripped */
		gripperCheckGripped();

		switch (state) {
		case GRIPPER_IDLE: {

			break;
		}
		case GRIPPER_MOVING: {

			break;
		}
		case GRIPPER_ATTEMPT: {
			if (gripper.gripped) {
				state = GRIPPER_GRIPPED;
			}
			if (servoComplete) {
				if (gripper.goalServo == gripper.calibratedMinAngle)
					gripperGripCounterClockwise();
				else
					gripperGripClockwise();
			}
			break;
		}
		case GRIPPER_CALIBRATE_MIN: {
			if (gripper.gripped || (gripper.readServo == 0)) {
				if(gripper.gripped){
					gripper.calibratedMinAngle = gripper.readServo + 15;
				} else{
					gripper.calibratedMinAngle = gripper.readServo;
				}
				state = GRIPPER_CALIBRATE_MAX;
				gripper.ungrip = 0;
			} else{
				gripperBoardSetServo(0);
			}
			break;
		}
		case GRIPPER_CALIBRATE_MAX: {
			if ((gripper.readServo == 255) ||( (gripper.readServo >  (gripper.calibratedMinAngle + 60)) && gripper.ungrip && (gripper.gripped || (gripper.readServo >= (gripper.calibratedMinAngle + 195))))) {
				gripper.calibrated = 1;
				state = GRIPPER_IDLE;
				if(gripper.gripped){
					gripper.calibratedMaxAngle = gripper.readServo - 15;
				} else{
					gripper.calibratedMaxAngle = gripper.readServo;
				}
				gripper.calibratedRestAngle = (gripper.calibratedMaxAngle + gripper.calibratedMinAngle)/2;
				gripperBoardSetServo(gripper.calibratedRestAngle);
			} else{
				gripperBoardSetServo(255);
			}
			break;
		}
		case GRIPPER_GRIPPED: {

			break;
		}
		default: {
			break;
		}
		}
		cprintf("State %d Max %d Min %d r %d cs %d gr %d sC %d \n",state, gripper.calibratedMaxAngle,gripper.calibratedMinAngle, gripper.readServo, gripper.currServo, gripper.gripped, servoComplete);
		/* Update Servo Position */
		if (!servoComplete)
			gripperUpdateServo();

		osTaskDelayUntil(&lastWakeTime, gripperPeriod);
		lastWakeTime = osTaskGetTickCount();
	}
}

void gripperGripClockwise() {
	gripperBoardSetServo(gripper.calibratedMinAngle);
}

void gripperGripCounterClockwise() {
	gripperBoardSetServo(gripper.calibratedMaxAngle);
}

void gripperGripRelax() {
	gripperBoardSetServo(gripper.calibratedRestAngle);
}

void gripperGripUntilGripped() {
	state = GRIPPER_ATTEMPT;
}

void gripperCheckGripped() {
	int i, c;

	gripper.grippedHistory = gripper.grippedHistory << 1;
	if ( gripper.current > 50 && gripper.force)
		gripper.grippedHistory++;

	c = 0;
	for (i = 0; i < 8; i++) {
		if ((gripper.grippedHistory >> i) % 2)
			c++;
	}

	if ((c >= 6) || ((gripper.pastPlace == gripperBoardGetServo()) && gripper.force )){
		gripper.gripped = TRUE;
		return;
	}else
		gripper.ungrip = 1;
		gripper.gripped = FALSE;
		gripper.pastPlace = gripperBoardGetServo();
}

void gripperUpdateServo() {
	if (gripper.currServo != gripper.goalServo) {
		if (gripper.currServo > gripper.goalServo) {
			if (gripper.currServo > (gripper.goalServo + GRIPPER_STEP)){
				gripper.currServo -= GRIPPER_STEP;
			}else{
				gripper.currServo = gripper.goalServo;
			}
		} else {
			if (gripper.currServo < (gripper.goalServo - GRIPPER_STEP)){
				gripper.currServo += GRIPPER_STEP;
			}else{
				gripper.currServo = gripper.goalServo;
			}
		}
	}

	/* Write out goal servo position */
	gripperMessageOut[GRIPPER_MSG_SERVO_VALUE_IDX] = gripper.currServo;
	expand0MsgWrite(gripperMessageOut);
}

/*
 *  @brief Change the position of the servo
 *  @details The range of the value translates into [0, 180] degree for the servo
 *  @param value The new position (range = [0, 255])
 *  @returns void
 */
void gripperBoardSetServo(uint8 value) {
	servoComplete = FALSE;
	gripper.goalServo = value;
}


/*
 *  @brief Retrieve the current value from the gripperboard
 *  @details The experimental 'gripped' value in the current scale is 50
 *  @returns The average current value (range = [0, 255]).
 */
uint8 gripperBoardGetCurrent() {
	return gripper.current;
}

/*
 *  @brief Retrieve the servo value from the gripperboard
 *  @returns The servo value (range = [0, 255]).
 */
uint8 gripperBoardGetServo() {
	return gripper.readServo;
}


/*
 *  @brief Check to see if the force is high
 *  @returns Value of force bit, 0 or 1
 */
uint8 gripperBoardGetForce() {
	return (uint8)gripper.force;
}

/*
 *  @brief Check to see if we have a soft stop
 *  @param takes in 0 or 1
 *  @returns Value of force bit, 0 or 1
 */
uint8 gripperBoardGetStop(uint8 value) {
	if (value) {
		return (uint8)gripper.stop1;
	} else {
		return (uint8)gripper.stop0;
	}

}

uint8 gripperBoardGetGripped() {
	return (uint8)gripper.gripped;
}

/*
 * @brief Send gripper information over serial
 * @param Serial input
 * @returns nothing
 */
void serialCmdSGFunc(char* command) {
	cprintf("svp %02X,%02X,%02X\n",
			gripperBoardGetCurrent(),
			gripperBoardGetServo(),
			gripperBoardGetGripped());
}

void gripperCalibratServo(){
	state = GRIPPER_CALIBRATE_MIN;
}


uint8 gripperServoCalibratFinish(){
	return gripper.calibrated;
}
