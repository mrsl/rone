/**
 * @file gripperBoard.h
 * @brief Controls the gripper extension board functions
 * @details Functionality includes controlling servos and motors
 * @since June 24, 2013
 * @author William Xie
 */
#ifndef GRIPPERBOARD_H_
#define GRIPPERBOARD_H_
// Modes

// Servo mode
/* SPI message */

#define GRIPPER_PERIOD_LENGTH	75

#define GRIPPER_CODE_LENGTH		3		// 3 code bytes for syncing
#define GRIPPER_PAYLOAD_LENGTH	EXPAND0_PAYLOAD_LENGTH		// 8 data bytes
#define GRIPPER_MSG_LENGTH		(GRIPPER_PAYLOAD_LENGTH + GRIPPER_CODE_LENGTH + 1) // sync bytes + data bytes + check sum byte

// Incoming and Outgoing
#define	GRIPPER_MSG_SERVO_VALUE_IDX		0
// Outgoing only
#define	GRIPPER_MSG_CURRENT_VALUE_IDX	1
#define	GRIPPER_MSG_IO_IDX				2

//High force bits packaged with mode
#define FORCE_BIT			0x80
#define SOFT_STOP0			0x40
#define SOFT_STOP1			0x20

#define FORCE_BIT_OFFSET	7
#define SOFT_STOP0_OFFSET	6
#define SOFT_STOP1_OFFSET	5

#define GRIPPER_DEFUALT_MAX_ANGLE	180
#define GRIPPER_DEFUALT_MIN_ANGLE	30
#define GRIPPER_DEFUALT_REST		90

#define GRIPPER_STEP		5

#define GRIPPER_IDLE 			0
#define GRIPPER_MOVING 			1
#define GRIPPER_ATTEMPT 		2
#define GRIPPER_GRIPPED 		3
#define GRIPPER_CALIBRATE_MIN 	4
#define GRIPPER_CALIBRATE_MAX 	5

typedef struct gripperState {
	uint8 current;
	uint8 readServo;
	uint8 currServo;
	uint8 goalServo;
	boolean force;
	boolean stop0;
	boolean stop1;
	uint8 grippedHistory;
	uint8 pastPlace;
	boolean gripped;
	boolean ungrip;
	boolean calibrated;
	uint8	calibratedMaxAngle;
	uint8	calibratedMinAngle;
	uint8   calibratedRestAngle;
} gripperState;

/**
 *  @brief Initialize the message and communication protocol to the gripperboard
 *  @param mode Select software the gripperboard is currently using:
 *  GRIPPER_MODE_SERVOS = 3 servos
 *  GRIPPER_MODE_MOTORS = 2 motors + 1 servo
 *  GRIPPER_MODE_STEPPER = 1 stepper motor + 1 servo (not implemented yet)
 *
 *  @returns void
 */
void gripperBoardInit();

void gripperDataInit();

void gripperTask(void* parameters);

void gripperGripClockwise();

void gripperGripCounterClockwise();

void gripperGripRelax();

void gripperGripUntilGripped();

void gripperCheckGripped();

void gripperUpdateServo();

void gripperCalibratServo();

uint8 gripperServoCalibratFinish();

/**
 *  @brief Change the position of the servo
 *  @details The range of the value translates into [0, 180] degree for the servo
 *  @param servoIndex The index of servo to be moved (see gripperBoard.h)
 *  @param value The new position (range = [0, 255])
 *  @returns void
 */
void gripperBoardSetServo(uint8 value);

/*
 *  @brief Retrieve the servo value from the gripperboard
 *  @returns The servo value (range = [0, 255]).
 */
uint8 gripperBoardGetServo();

/**
 *  @brief Retrieve the current value from the gripperboard
 *  @details The experimental 'gripped' value in the current scale is 50
 *  @param servoIndex The index of servo to get current value from
 *  @returns The average current value (range = [0, 255]).
 */
uint8 gripperBoardGetCurrent();

/*
 *  @brief Check to see if the force is high
 *  @returns Value of force bit, 0 or 1
 */
uint8 gripperBoardGetForce();

/*
 *  @brief Check to see if we have a soft stop
 *  @param takes in 0 or 1
 *  @returns Value of force bit, 0 or 1
 */
uint8 gripperBoardGetStop(uint8 value);

/*
 * @brief Send gripper information over serial
 * @param Serial input
 * @returns nothing
 */
void serialCmdSGFunc(char* command);
#endif /* GRIPPERBOARD_H_ */
