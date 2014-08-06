/*
 * @file systemCommands.c
 *
 * @brief Parses char strings that are system commands.
 * @since April 2, 2012
 * @author James McLurkin
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "roneos.h"

//TODO move as much of this as possible to a different file.

/* Sizes for the GUI command data. */
#define GUI_ARG_COUNT 	8
#define GUI_ARG_SIZE	64

#define CMD_AUDIO	'a' /* Test audio. */
#define CMD_TVRV	'm'	/* Set velocity. */
#define CMD_LEDS	'l' /* Set LED pattern. */
#define CMD_PWM		'p'	/* Set PWM directly. */
#define CMD_VEL		'v' /* Set Velocity directly */
#define CMD_IR		'i'	/* Send IR message. */
#define CMD_RADIO	'r'	/* Send Radio message. */

/*
 * @brief Commands from the GUI
 */
typedef struct {
	uint8 cmd; /* Command id */
	char arg[GUI_ARG_COUNT][GUI_ARG_SIZE];/* Arguments */
} guiCmdData;


// provide these pototypes here because they are internal functions
boolean radioCommandReceiveNonBlocking_internal(RadioCmd* radioCmdPtr, RadioMessage* messagePtr);
void radioCommandXmit_internal(RadioCmd* radioCmdPtr, uint8 destinationID, RadioMessage* messagePtr);
void ledsSetPattern_rc(uint8 color, uint8 pattern, uint8 brightness, uint8 rate);
boolean irCommsSendMessage_internal(IRCommsMessage *irMessagePtr);
boolean irCommsGetMessage_internal(IRCommsMessage* irMessagePtr);

RadioCmd radioCmdGUI;

static guiCmdData cmd = {0};
static int32 pwmL = 0, pwmR = 0;
static int32 velL = 0, velR = 0;
static boolean lightsOn = TRUE;

static SerialCmd serialCmdSY;
static SerialCmd serialCmdSV;
static SerialCmd serialCmdRC;
static SerialCmd serialCmdR1;
static SerialCmd serialCmdRT;
static SerialCmd serialCmdRI;
static SerialCmd serialCmdRR;
static SerialCmd serialCmdBH;
static SerialCmd serialCmdBR;
static SerialCmd serialCmdBX;


// sy = system
void serialCmdSYFunc(char* command) {
	systemPrintMemUsage();
}


// sv = sensor values
void serialCmdSVFunc(char* command) {
	int i;
	//IRCommsMessage ir_msg;
	RadioMessage radioMessage;
	//uint32 msg_size, msg_linkq;
	//char msg[RADIO_MESSAGE_LENGTH_RAW] = {0};

	/* id */
	cprintf("svs %04X\n", roneID);

	/* IR */
	/* Old
	char copy[IR_COMMS_MESSAGE_LENGTH_MAX+1];
	if (irCommsGetMessage_internal(&ir_msg)) {
		for (i = 0; i < IR_COMMS_MESSAGE_LENGTH_MAX; i++) {
			copy[i] = ir_msg.data[i];
		}
		copy[IR_COMMS_MESSAGE_LENGTH_MAX] = '\0';
		cprintf("svi %s,%02X\n", copy, irCommsGetMessageReceiveBits(&ir_msg));
	}
	*/
	uint8 *obstacleMatrix = irObstaclesGetBitMatrix();
	cprintf("svi ");
	for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; i++) {
		cprintf("%02X", obstacleMatrix[i]);

		if (i == IR_COMMS_NUM_OF_RECEIVERS - 1)
			cprintf("\n");
		else
			cprintf(",");
	}

	/* buttons */
	cprintf("svb %01X,%01X,%01X\n",
			buttonsGet(BUTTON_RED),
			buttonsGet(BUTTON_GREEN),
			buttonsGet(BUTTON_BLUE));

#if defined(RONE_V9) || defined(RONE_V12)

	/* radio */
	/* Get message. */
	if (radioCommandReceiveNonBlocking_internal(&radioCmdGUI, &radioMessage)) {//radio_get_message_rc(&radio_msg)) {
		char *msg = radioCommandGetDataPtr(&radioMessage);
		cprintf("svr %s\n", msg);
	}

	/* gyro */
	cprintf("svg %04X,%04X,%04X\n",
			(uint16)(int16)gyroGetValue(GYRO_X_AXIS),
			(uint16)(int16)gyroGetValue(GYRO_Y_AXIS),
			(uint16)(int16)gyroGetValue(GYRO_Z_AXIS));

	/* accelerometer */
	cprintf("sva %04X,%04X,%04X\n",
			(uint16)(int16)accelerometerGetValue(ACCELEROMETER_X),
			(uint16)(int16)accelerometerGetValue(ACCELEROMETER_Y),
			(uint16)(int16)accelerometerGetValue(ACCELEROMETER_Z));

	/* light sensor */
	#if defined(RONE_V9)
	cprintf("svl %04X,%04X,%04X\n",
			lightSensorGetValue(LIGHT_SENSOR_FRONT_LEFT),
			lightSensorGetValue(LIGHT_SENSOR_FRONT_RIGHT),
			lightSensorGetValue(LIGHT_SENSOR_REAR));
	#elif defined(RONE_V12)
	cprintf("svl %04X,%04X,%04X,%04X\n",
			lightSensorGetValue(LIGHT_SENSOR_FRONT_LEFT),
			lightSensorGetValue(LIGHT_SENSOR_FRONT_RIGHT),
			lightSensorGetValue(LIGHT_SENSOR_REAR_LEFT),
			lightSensorGetValue(LIGHT_SENSOR_REAR_RIGHT));
	#endif

	/* encoders */
		/* left */
	cprintf("sve l,%04X,%04X\n",
			encoderGetTicks(ENCODER_LEFT),
			(uint16)(int16)encoderGetVelocity(ENCODER_LEFT));
		/* right */
	cprintf("sve r,%04X,%04X\n",
			encoderGetTicks(ENCODER_RIGHT),
			(uint16)(int16)encoderGetVelocity(ENCODER_RIGHT));

	/* Bump Sensors */
	cprintf("svm %02X\n", bumpSensorsGetBits());
#endif

}

void parseGUIMsg(char *chrPtr, guiCmdData *command) {
	int i, j;
	command->cmd = *chrPtr;
	do {
		chrPtr++; /* ignore whitespace */
	} while (*chrPtr == ' ');

	/* Loop through each command argument. */
	for (i = 0; i < GUI_ARG_COUNT; i++) {
		j = 0;
		/* Fill in each argument. */
		while (*chrPtr != ',') {
			command->arg[i][j] = *chrPtr;
			if (*chrPtr == '\0') {
				chrPtr--;
				/* Get rid of trailing new-lines */
				while (*chrPtr == '\r' || *chrPtr == '\n') {
					*chrPtr = '\0';
					chrPtr--;
				}
				return;
			}
			j++;
			chrPtr++;
		}
		chrPtr++; /* Skip comma */
	}
}


void executeCmd(guiCmdData *command) {
	int i;
	switch (command->cmd) {
	case CMD_LEDS: {
		uint8 color = atoi_hex8(command->arg[0]);
		uint8 pattern = atoi_hex8(command->arg[1]);
		uint8 brightness = atoi_hex8(command->arg[2]);
		uint8 rate = atoi_hex8(command->arg[3]);

		ledsSetPattern_rc(color, pattern, brightness, rate);
		break;
	}
	case CMD_IR: {
		IRCommsMessage irMsg;
		for (i = 0; i < IR_COMMS_MESSAGE_LENGTH_MAX; i++) {
			irMsg.data[i] = '\0';
		}
		for (i = 0; command->arg[0][i] != '\0' && command->arg[0][i] != '\r' && i < IR_COMMS_MESSAGE_LENGTH_MAX; i++) {
			irMsg.data[i] = command->arg[0][i];
		}
		irMsg.data[i] = '\0';

		if (irMsg.data[0] != '\0') {
			irCommsSendMessage_internal(&irMsg);
		}
		break;
	}
#if (defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12))
	case CMD_PWM: {
		if (*command->arg[0] == 'l') {
			pwmL = (int8)atoi_hex8(command->arg[1]);
		} else {
			pwmR = (int8)atoi_hex8(command->arg[1]);
		}
		motorSetPWM_rc(MOTOR_LEFT, pwmL);
		motorSetPWM_rc(MOTOR_RIGHT, pwmR);
		break;
	}
	case CMD_VEL: {
		if (*command->arg[0] == 'l') {
			velL = (int16)atoi_hex16(command->arg[1]);
		} else {
			velR = (int16)atoi_hex16(command->arg[1]);
		}
		motorSetVelocity_rc(MOTOR_LEFT, velL);
		motorSetVelocity_rc(MOTOR_RIGHT, velR);
		break;
	}
	case CMD_TVRV: {
		int32 tv = (int16)atoi_hex16(command->arg[0]);
		int32 rv = (int16)atoi_hex16(command->arg[1]);
		motorSetTVRV_rc(tv, rv);
		break;
	}
	case CMD_RADIO: {
		RadioMessage radioMsg;
		/* Clear the radio message. */
		for (i = 0; i < RADIO_COMMAND_MESSAGE_DATA_LENGTH; i++) {
			radioMsg.raw.data[i] = '\0';
		}

		/* Get the message data pointer. Modify this instead of the
		 * radio message directly because the first two bytes of the
		 * radio message are used for the command type and the destination
		 * ID (which robots listen to the command).
		 */
		char* msgData = radioCommandGetDataPtr(&radioMsg);

		/* Go through what was sent over serial and add it to the radio message. */
		for (i = 0; command->arg[0][i] != '\0' && command->arg[0][i] != '\r' && i < RADIO_COMMAND_MESSAGE_DATA_LENGTH; i++) {
			msgData[i] = command->arg[0][i];
		}
		/* Make sure the message is correctly null terminated. */
		msgData[i] = '\0';

		/* Only broadcast if there is a message. */
		if (msgData[0] != '\0') {
			radioCommandXmit_internal(&radioCmdGUI, ROBOT_ID_ALL, &radioMsg);
		}
		break;
	}
	case CMD_AUDIO: {
		//TODO: Put something here
		break;
	}
#endif

	default: {
		break;
	}
	}
}


// rc = sensor values
void serialCmdRCFunc(char* command) {
	rcMode = RC_MODE_ON;
	rcTimer = 0;
	//TODO Hack warning!
	// move past the command text, skip the 'RI'
	//TODO need structured argument parsing
	command = command + 2;
	parseGUIMsg(command, &cmd);
	executeCmd(&cmd);
	lightsOn = FALSE;
}

uint8 rtRobotID = 46;


/*
 * Strips the leading and trailing spaces of the input
 * string. Used to get the ID of a robot from a ri command.
 */
void stripLeadingAndTrailingSpaces(char* string){

     /* First remove leading spaces */
     const char* firstNonSpace = string;

     while (*firstNonSpace != '\0' && isspace((unsigned char)(*firstNonSpace))) {
          ++firstNonSpace;
     }

     size_t len = strlen(firstNonSpace) + 1;

     memmove(string, firstNonSpace, len);

     /* Now remove trailing spaces */
     char* endOfString = string + len;

     while (string < endOfString  && isspace((unsigned char)(*endOfString))) {
          --endOfString ;
     }

     *endOfString = '\0';
}

#if (defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12))

/*
 * @brief Queries all remote robots for the contents of their cprintf buffer.
 * rt = remote terminal
 */
void serialCmdR1Func(char* command) {
	char string[50];
	float batteryVoltage = systemBatteryVoltageGet();
	sprintf(string, "r1,%d,%1.1f\n", roneID, batteryVoltage);
	cprintf(string);
}

/*
 * @brief Queries all remote robots for the contents of their cprintf buffer.
 * rt = remote terminal
 *
 */
void serialCmdRTFunc(char* command) {
	//TODO print list of connected robots
	//rts,num,id1,id2,...\n
	cprintf("rts,%d", rprintfGetActiveRobotsNum());
	rprintfPrintActiveRobotList();
	cprintf("\n");
	rprintfSetHostMode(RPRINTF_HOST);
}

/*
 * @brief Adds the specified robot to a list of included robots and
 * queries the robots on that list remotely for the contents
 * of their rprintf buffer.
 *
 * ri ## = remote include robot with ID of ##
 */
void serialCmdRIFunc(char* command) {
	stripLeadingAndTrailingSpaces(command);
	//TODO Hack warning!
	// move past the command text, skip the 'RI'
	//TODO need structured argument parsing
	command = command + 2;
	rprintfEnableRobot(atoi(command), TRUE);
	rprintfSetHostMode(RPRINTF_HOST);
}


void serialCmdRRFunc(char* command) {
	cprintf("rr %04X,%04X,%04X\n", roneID, rprintfIsHost(),
		radioCommandGetLocalSubnet());
}


void serialCmdBHFunc() {
	writeBootloaderState(BL_STATE_HOST);
	bootloading();
}


void serialCmdBRFunc() {
	writeBootloaderState(BL_STATE_RECEIVE);
	bootloading();
}


void serialCmdBXFunc() {
	writeBootloaderState(BL_STATE_XMODEM);
	bootloading();
}


#endif /* (defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12)) */

/*
 * @brief Initialize system commands.
 *
 * @returns void
 */
void systemCommandsInit() {
	// system check
	serialCommandAdd(&serialCmdSY, "sy", serialCmdSYFunc);

	// sensor values
	serialCommandAdd(&serialCmdSV, "sv", serialCmdSVFunc);

	// remote control
	serialCommandAdd(&serialCmdRC, "rc", serialCmdRCFunc);

	// RCC
	serialCommandAdd(&serialCmdRR, "rr", serialCmdRRFunc);

	#if (defined(RONE_V6) || defined(RONE_V9) || defined(RONE_V12))
	radioCommandAddQueue(&radioCmdGUI, "GUI", 10);

	// robot status for pinging com posrt
	serialCommandAdd(&serialCmdR1, "r1", serialCmdR1Func);

	// remote terminal
	serialCommandAdd(&serialCmdRT, "rt", serialCmdRTFunc);

	// remote include
	serialCommandAdd(&serialCmdRI, "ri", serialCmdRIFunc);

	// bootloader host
	serialCommandAdd(&serialCmdBH, "bh", serialCmdBHFunc);

	// bootloader remote
	serialCommandAdd(&serialCmdBR, "br", serialCmdBRFunc);

	// bootloader xmodem
	serialCommandAdd(&serialCmdBX, "bx", serialCmdBXFunc);

	#endif
}
