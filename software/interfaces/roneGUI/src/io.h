/*
 * io.h
 *
 * Header for input/output.
 *
 *  Created on: Aug 6, 2011
 *      Author: nathan
 */

#ifndef IO_H_
#define IO_H_

/*
 * Command characters. These are used when the robot and GUI
 * are talking to each other over serial.
 */

/* Input data names */

#define NUM_TEXTBOXES 		3
#define NUM_LED_BUTTONS 	3
#define NUM_MOTOR_LEFT		5
#define NUM_MOTOR_RIGHT	5

/* Max number of neighbors - defined in the nbr code in roneos */
#define NEIGHBOR_MAX		30

#define NAME_TEXTBOX		0
#define NAME_LED_BUTTON	1
#define NAME_MOTOR_LEFT	2
#define NAME_MOTOR_RIGHT	3
#define NAME_MOTOR_SYNC	4
#define NAME_TRANSMIT		5
#define NAME_VEL			6
#define NAME_PWM			7
#define NAME_SMOOTH		8
#define NAME_SELECT		9

#define BUTTON_LED_RED		0
#define BUTTON_LED_GREEN	1
#define BUTTON_LED_BLUE	2
#define BUTTON_LED_ALL		3
#define BUTTON_LED_OFF		4

#define MOTOR_ADD10		0
#define MOTOR_ADD1			1
#define MOTOR_OFF			2
#define MOTOR_SUB1			3
#define MOTOR_SUB10		4

#define TEXTBOX_COM_PORT	0
#define TEXTBOX_RADIO		1
#define LEN_TXT_COM_PORT	4
#define LEN_TXT_RADIO		(LEN_RADIO_MESSAGE)

#define PWM				0
#define VEL 				1


#define MAX_TEXTBOX_LENGTH 256

typedef struct textbox {
	int length;
	int index;
	char message[MAX_TEXTBOX_LENGTH];
} textbox;

extern int currentLED;
extern int currentTextbox;
extern int motorLeft;
extern int motorRight;
extern int motorsSynced;
extern int transmit;
extern int velOrPwm;
extern int smoothing;
extern int autoselect;
extern textbox textboxes[];


/* Output commands. These are sent *from* the robot *to* the GUI. */
#define DAT_ACCEL	'a'
#define DAT_BTN		'b'
#define DAT_ENC		'e'
#define DAT_GYRO	'g'
#define DAT_IR		'i'
#define DAT_LIGHT	'l'
#define DAT_BUMP	'm'
#define DAT_NBR		'n'
#define DAT_NBRS	'o'
#define DAT_GRIP	'p'
#define DAT_RADIO	'r'
#define DAT_ID		's'
#define DAT_BATT	'v'
#define DAT_EXP		'x'

/* Input commands. These are sent *from* the GUI *to* the robot. */
#define CMD_AUDIO	'a'
#define CMD_PWM		'p'
#define CMD_LEDS	'l'
#define CMD_TVRV	'm'
#define CMD_RADIO	'r'

/* Command parameters. */
#define ARG_COUNT	8
#define ARG_SIZE	64

#define ALPHA		0.4
#define ALPHAF		0.3

/* Output data index values. */
#define BTN_RED 		0
#define BTN_GREEN 		1
#define BTN_BLUE 		2

#define LS_FRONT_LEFT 	0
#define LS_FRONT_RIGHT 1
#define LS_BACK_LEFT 	2
#define LS_BACK_RIGHT	3

#define ENC_LEFT 		0
#define ENC_RIGHT 		1

typedef struct nbrData nbrData;

/* Command structure to be used by both GUI and robot. */
typedef struct guiCmdData {
	char cmd; /* Command letter */
	char arg[ARG_COUNT][ARG_SIZE]; /* Array of argument strings */
} guiCmdData;

/*
 * Robot typedefs.
 *
 * Use *only* when exact bit lengths are needed (input to the robots).
 */
typedef signed char 	int8;
typedef signed short 	int16;
typedef signed long 	int32;

typedef unsigned char	uint8;
typedef unsigned short	uint16;
typedef unsigned long	uint32;


/* Data values for the gyroscope (v9) and accelerometer. */
typedef struct axisData{
	int x, y, z;
} axisData;

/* Data values for the encoder */
typedef struct encoderData{
	int ticks;
	int velocity;
} encoderData;

typedef struct nbrData {
	int id;
	float bearing;
	float orientation;
	float range;
} nbrData;

typedef struct gripData {
	int isOn;
	int current;
	int servo;
	int gripped;
} gripData;

/*
 * Data values for all output.
 */
typedef struct outputData {
	int id;
	int isv11;
	char radioMsg[LEN_TXT_RADIO];
	unsigned int buttons[NUM_BUTTONS];
	unsigned int lightSensors[NUM_LIGHT_SENSORS];
	axisData gyro;
	axisData accelerometer;
	encoderData encoders[NUM_ENCODERS];
	unsigned int bumpSensors;
	unsigned int irData[8];
	nbrData nbrs[NEIGHBOR_MAX];
	int numNbrs;
	int expansionVoltage;
	int batteryVoltage;
	gripData gripper;
} outputData;


#endif /* IO_H_ */
