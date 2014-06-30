/*
 * output.c
 *
 *  Created on: Jul 21, 2011
 *      Author: nathan
 */

#include "gui.h"

#define MSG_SIZE		256

/*
 * Conversion ASCII -> uint conversion functions.
 *
 * The GUI receives command arguments from the robot as ASCII hex characters.
 * The integer data types are necessary for proper conversion between
 * signed and unsigned data packets.
 */
uint8 convertASCIIHexNibble(char val)
{
	if (val >= '0' && val <= '9') {
		return (val - '0');
	} else if (val >= 'A' && val <= 'F') {
		return (val - 'A' + 10);
	} else if (val >= 'a' && val <= 'f') {
		return (val - 'a' + 10);
	} else {
		return 0;
	}
}

uint8 convertASCIIHexByte(char *val)
{
	uint8 temp;
	temp = convertASCIIHexNibble(*val) * 16;
	temp = temp + convertASCIIHexNibble(*(val + 1));
	return temp;
}

uint16 convertASCIIHexWord(char *val)
{
	uint16 temp;

	temp = (uint16) (convertASCIIHexByte(val)) * 256;
	temp = temp + (uint16) (convertASCIIHexByte(val + 2));
	return temp;
}

uint32 convertASCIIHexLong(char *val)
{
	uint32 temp;
	temp = (uint32) convertASCIIHexWord(val) * 65536;
	temp = temp + (uint32) convertASCIIHexWord(val + 4);
	return temp;
}

int convertASCIIBool(char val)
{
	switch (val)
	{
	case '0':
	case ' ': {
		return 0;
	}
	default: {
		return 1;
	}
	}
}

/* Initialize members of the output data to zero values */
void outputInit(outputData *out)
{
	int i;
	out->id = 0;
	memset(out->radioMsg, 0, LEN_TXT_RADIO);
	for (i = 0; i < NUM_BUTTONS; i++) {
		out->buttons[i] = 0;
	}
	for (i = 0; i < NUM_LIGHT_SENSORS; i++) {
		out->lightSensors[i] = 0;
	}
	out->gyro.x = 0;
	out->gyro.y = 0;
	out->gyro.z = 0;
	out->accelerometer.x = 0;
	out->accelerometer.y = 0;
	out->accelerometer.z = 0;
	for (i = 0; i < NUM_ENCODERS; i++) {
		out->encoders[i].ticks = 0;
		out->encoders[i].velocity = 0;
	}

	/* Temporary testing data */
	for (i = 0; i < 8; i++)
		out->irData[i] = 0;

	for (i = 0; i < NEIGHBOR_MAX; i++) {
		out->nbrs[i].id = 0;
	}

	out->bumpSensors = 0;
	out->expansionVoltage = 0;
	out->batteryVoltage = 0;

	out->gripper.isOn = 0;
	return;
}

/*
 * Parse a serial message.
 */
void parseMsg(char *chrPtr, guiCmdData *command, int size)
{
	int i, j;
	/* Check that it's a sensor value command. */
	if (chrPtr[0] != 's' || chrPtr[1] != 'v') {
		command->cmd = '\0';
		return;
	}

	chrPtr += 2;

	command->cmd = *chrPtr;
	do {
		chrPtr++; /* ignore whitespace */
	} while (*chrPtr == ' ');

	/* Loop through each command argument. */
	for (i = 0; i < ARG_COUNT; i++) {
		j = 0;
		/* Fill in each argument. */
		while (*chrPtr != ',') {
			/* When the end-line termination sequence is reached,
			 * append a null and quit the function
			 */
			if (*chrPtr == '\r' && *(chrPtr + 1) == '\n') {
				command->arg[i][j] = '\0';
				return;
			} else if (j == ARG_SIZE) {
				/*
				 * Check this to make sure it doesn't try to access
				 * invalid memory space.
				 */
				command->arg[i][j] = '\0';
				return;
			}
			command->arg[i][j] = *chrPtr;

			j++;
			chrPtr++;
		}
		chrPtr++; /* Skip comma */
	}
}

/*
 * Execute a robot command, updating the values in the corresponding data.
 */
void executeCmd(guiCmdData *command, outputData *out)
{
	int i;
	/*
	 * Clear the IR and radio structures so they don't display the last
	 * received message even if it wasn't sent this round.
	 */

	switch (command->cmd)
	{
	case DAT_ID: {
		out->id = convertASCIIHexWord(command->arg[0]);
		break;
	}
	case DAT_BTN: {
		out->buttons[BTN_RED] = *command->arg[0] == '1' ? 1 : 0;
		out->buttons[BTN_GREEN] = *command->arg[1] == '1' ? 1 : 0;
		out->buttons[BTN_BLUE] = *command->arg[2] == '1' ? 1 : 0;
		break;
	}
	case DAT_GYRO: {
		/* Put the new input through a differential so we get a smooth transition.*/
		if (smoothing) {
			differential((int16) convertASCIIHexWord(command->arg[0]),
				&out->gyro.x, ALPHA);
			differential((int16) convertASCIIHexWord(command->arg[1]),
				&out->gyro.y, ALPHA);
			differential((int16) convertASCIIHexWord(command->arg[2]),
				&out->gyro.z, ALPHA);
		} else {
			out->gyro.x = (int16) convertASCIIHexWord(command->arg[0]);
			out->gyro.y = (int16) convertASCIIHexWord(command->arg[1]);
			out->gyro.z = (int16) convertASCIIHexWord(command->arg[2]);
		}
		break;

	}
	case DAT_ACCEL: {
		/* Put the new input through a differential so we get a smooth transition.*/
		if (smoothing) {
			differential((int16) convertASCIIHexWord(command->arg[0]),
				&out->accelerometer.x, ALPHA);
			differential((int16) convertASCIIHexWord(command->arg[1]),
				&out->accelerometer.y, ALPHA);
			differential((int16) convertASCIIHexWord(command->arg[2]),
				&out->accelerometer.z, ALPHA);
		} else {
			out->accelerometer.x = (int16) convertASCIIHexWord(command->arg[0]);
			out->accelerometer.y = (int16) convertASCIIHexWord(command->arg[1]);
			out->accelerometer.z = (int16) convertASCIIHexWord(command->arg[2]);
		}
		break;
	}
	case DAT_LIGHT: {
		out->lightSensors[LS_FRONT_LEFT] = convertASCIIHexWord(command->arg[0]);
		out->lightSensors[LS_FRONT_RIGHT] = convertASCIIHexWord(
			command->arg[1]);
		out->lightSensors[LS_BACK_RIGHT] = convertASCIIHexWord(command->arg[2]);
		out->lightSensors[LS_BACK_LEFT] = convertASCIIHexWord(command->arg[3]);

		break;
	}
	case DAT_ENC: {
		int encoder = (*command->arg[0] == 'r' ? ENC_RIGHT : ENC_LEFT);
		out->encoders[encoder].ticks = convertASCIIHexWord(command->arg[1]);
		out->encoders[encoder].velocity = (int32) (int16) convertASCIIHexWord(
			command->arg[2]);
		break;
	}
	case DAT_RADIO: {
		//memset(out->radioMsg, 0, LEN_TXT_RADIO);
		strcpy(out->radioMsg, command->arg[0]);
		break;
	}
	case DAT_BUMP: {
		out->bumpSensors = convertASCIIHexByte(command->arg[0]);
		break;
	}
	case DAT_NBR: {
		int id;
		float bearing, orientation, range;
		float bearingDir = (command->arg[5][0] == 'F') ? -1 : 1;
		float orientationDir = (command->arg[6][0] == 'F') ? -1 : 1;

		i = convertASCIIHexByte(command->arg[0]);
		id = convertASCIIHexByte(command->arg[1]);
		bearing = 180. / PI * (int16) convertASCIIHexWord(command->arg[2])
			* bearingDir / 1000.;
		orientation = 180. / PI * (int16) convertASCIIHexWord(command->arg[3])
			* orientationDir / 1000.;
		range = (int16) convertASCIIHexWord(command->arg[4]);

		if (smoothing) {
			if (id == out->nbrs[i].id) {
				differentiala(bearing, &out->nbrs[i].bearing);
				differentiala(orientation, &out->nbrs[i].orientation);
				differentialf(range, &out->nbrs[i].range, ALPHA);
			}
		} else {
			out->nbrs[i].id = id;
			out->nbrs[i].bearing = bearing;
			out->nbrs[i].orientation = orientation;
			out->nbrs[i].range = range;
		}
		break;
	}
	case DAT_NBRS: {
		out->numNbrs = convertASCIIHexByte(command->arg[0]);
		break;
	}
	case DAT_IR: {
		for (i = 0; i < NUM_IR_SENSORS; i++)
			out->irData[i] = convertASCIIHexByte(command->arg[i]);
		break;
	}
	case DAT_GRIP: {
		out->gripper.isOn = 1;
		out->gripper.current = convertASCIIHexByte(command->arg[0]);
		out->gripper.servo = convertASCIIHexByte(command->arg[1]);
		out->gripper.gripped = convertASCIIHexByte(command->arg[2]);
		break;
	}
		/*
		 case DAT_BATT: {
		 out->batteryVoltage = convertASCIIHexByte(command->arg[0]);
		 break;
		 }
		 case DAT_EXP: {
		 out->expansionVoltage = convertASCIIHexByte(command->arg[0]);
		 break;
		 }
		 */
	default: {
		break;
	}
	}
}

/*
 * Update output data from serial messages from the robot.
 */
int updateOutput(outputData *output)
{
	char message[MSG_SIZE] = { 0 };
	int gotNewData = 0;

	int x;

	while ((x = serialMessageGet(message, MSG_SIZE)) > 0) {
		guiCmdData cmd = { 0 };
		parseMsg(message, &cmd, x);
		executeCmd(&cmd, output);
		gotNewData = 1;
	}
	return gotNewData;
}

