/*
 * params.h
 *
 * Parameters for object sizes and positions.
 *
 *  Created on: Aug 6, 2011
 *      Author: nathan
 */

#ifndef PARAMS_H_
#define PARAMS_H_

/*
 * Rone constants.
 *
 * These are copies of values specific to the rone robot.
 */
#define MAX_LIGHT_SENSOR_VALUE 		1024
#define MAX_ACCELEROMETER_VALUE		1040
#define MAX_GYRO_VALUE 				40000
#define MAX_VBAT_VALUE				256
#define MAX_VEXP_VALUE				256
#define MAX_SPEED 					300
#define MAX_PWM						100
#define MAX_VEL						200

#define LEN_RADIO_MESSAGE			30
#define LEN_IR_MESSAGE				4

#define NUM_BUTTONS					3
#define NUM_LIGHT_SENSORS			4
#define NUM_ENCODERS				2
#define NUM_BUMP_SENSORS			8
#define NUM_IR_SENSORS 				8


/*
 * These are the various timing delay parameters
 * used with glutTimerFunc()
 *
 * The number is in milliseconds.
 */
#define QUERY_DELAY 				23
#define NBR_QUERY_DELAY				100
#define G_QUERY_DELAY				100
#define READ_DELAY 					1
#define MOTOR_DELAY					100
#define AUTOSELECT_DELAY			100

/*
 * Drawing parameters.
 *
 * Sizes and positions of objects on the canvas.
 */

/* General */
#define LEFT_ALIGN_X				-13
#define RIGHT_ALIGN_X				10.5

/* Title */
#define TITLE_POS_X					-13
#define TITLE_POS_Y					11.7

/* COM port */
#define COMPORT_POS_X				(TITLE_POS_X + 0.5)
#define COMPORT_POS_Y				(TITLE_POS_Y - 1.5)

/* Auto select button */
#define SELECT_POS_X				(COMPORT_POS_X + 3.5)
#define SELECT_POS_Y				(COMPORT_POS_Y + 0.45)

/* Pushbutton */
#define PUSHBUTTON_RADIUS			0.6
#define PUSHBUTTON_RING_WIDTH		0.05

#define PUSHBUTTON_TEXT_OFFSET_X	0.65
#define PUSHBUTTON_TEXT_OFFSET_Y	-(PUSHBUTTON_RADIUS / 2)

/* Accelerometer */
#define ACC_SCALE					3.0

#define ACC_POS_X 					(RIGHT_ALIGN_X - .8)
#define ACC_POS_Y 					-11

/* Gyro */
#define GYRO_RADIUS 				3

#define GYRO_POS_X 					LEFT_ALIGN_X
#define GYRO_POS_Y 					-8.1

/* Light sensors */
#define LS_WIDTH 					0.5
#define LS_HEIGHT 					3

#define LS_POS_X 					-13
#define LS_POS_Y					-1
#define LS_OFFSET_X 				4
#define LS_OFFSET_X_G				6
#define LS_OFFSET_Y 				3.5

#define LS_TEXT_OFFSET				0.2

/* Encoders */
#define ENC_RADIUS 					2

#define ENC_POS_X 					0
#define ENC_POS_Y 					-10
#define ENC_OFFSET_X 				2

/* Velocity or PWM Selector */
#define PWM_X						-3
#define PWM_Y						-3
#define VEL_X						(PWM_X + 3)
#define VEL_Y						PWM_Y


//#define VOP_X							0
//#define VOP_Y							-6.75
//#define VOP_OFFSET_X					2.2

/* IR messages */
#define IR_POS_X 					LEFT_ALIGN_X
#define IR_POS_Y					-2

#define IR_BUTTON_X					(IR_POS_X - 1.6)
#define IR_BUTTON_Y					(IR_POS_Y + 1)

#define IR_OFFSET_X_SMALL			1
#define IR_OFFSET_X_LARGE			2.5
//2.25

#define IR_OFFSET_Y_SMALL			1.25
#define IR_OFFSET_Y_LARGE			2.75

/* Radio messages */
#define RADIO_POS_X					0
#define RADIOOUT_POS_Y				10
#define RADIOIN_POS_Y				(RADIOOUT_POS_Y + 1.5)

/* Robot */
#define ROBOT_RADIUS 				4

#define ROBOT_POS_X 				0
#define ROBOT_POS_Y 				3.5

/* Gripper */
#define GRIPPER_RADIUS				4.6
#define SEPERATOR_RADIUS			4.3
#define GRIPPER_CLAW_SIZE_X			0.25
#define GRIPPER_CLAW_SIZE_Y			0.4
#define GRIPPER_ANGLE_MAX			32.

/* IR Obstacle Sensor */
#define IR_ROBOT_RADIUS				2
#define IR_FIELD_RADIUS				5
#define IR_ARROW_RADIUS				2.3
#define IR_TRIANGLE					0.125
#define IR_ROBOT_POS_X				11.5
#define IR_ROBOT_POS_Y				ROBOT_POS_Y

/* Neighbor diagram */
#define NBR_ROBOT_RADIUS			0.25
#define NBR_FIELD_SIZE				5
#define NBR_RANGE_SCALE				5. / 1000.

#define NBR_ROBOT_POS_X				-11.5
#define NBR_ROBOT_POS_Y				ROBOT_POS_Y

/* Smoothing buttom */
#define SMOOTH_POS_X				-16
#define SMOOTH_POS_Y				(ROBOT_POS_Y - 6)

/* Button output | LED input */
#define BTN_SIZE 					0.35
#define LED_SIZE					0.75

#define CIRCLE_BTN_SIZE				0.35

#define BTNLED_OFFSET_X 			2
#define BTNLED_OFFSET_Y 			0

#define BTNLED_POS_X				0
#define BTNLED_POS_Y				(ROBOT_POS_Y - 1.5)

/* PWM motor buttons */
#define MOTOR_POS_X					0
#define MOTOR_POS_Y					-8

#define MOTOR_OFFSET_X				1
#define MOTOR_OFFSET_Y_10			2
#define MOTOR_OFFSET_Y_1			1

#define MOTOR_TEXT_OFFSET_X			(4 + 0.25)
#define MOTOR_TEXT_POS_Y			-5

#define MOTOR_SYNC_OFFSET_X			(MOTOR_TEXT_OFFSET_X - 2.25)
#define MOTOR_SYNC_POS_Y			(MOTOR_TEXT_POS_Y+0.5)

// GUI settings
// TODO: Make line width and point size variable with changing window size.
#define POINT_SIZE 					10
#define LINE_WIDTH 					2
#define LINE_WIDTH_MEDIUM			1
#define LINE_WIDTH_SMALL 			0.25

#define PICK_DELTA					3.0

#define DISK_SLICES 				60
#define DISK_LOOPS					3

#define ANGLE_90 					90
#define ANGLE_180 					180

#define ANNULUS_THIN 				.05
#define ANNULUS_THICK				0.1
#define BUMP_ANGLE 					22.5
#define TEXTBOX_BORDER 				0.25

/* Initial window dimensions */
#define WINDOW_WIDTH				 800
#define WINDOW_HEIGHT 				600

/* GUI dimensions */
#define ASPECT 						(GUI_WIDTH / GUI_HEIGHT)
#define GUI_WIDTH 					39.0
#define GUI_HEIGHT 					26.0


#endif /* PARAMS_H_ */
