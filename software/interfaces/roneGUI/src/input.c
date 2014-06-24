/*
 * input.c
 *
 * Methods dealing with input from the GUI to the robot,
 * such as text and buttons (the mouse-clicky type).
 *
 *  Created on: Jun 1, 2011
 *      Author: nathan
 */

#include "gui.h"

/*
 * State variables.
 */
int currentLED;
int motorLeft;
int motorRight;
int currentTextbox;
int motorsSynced;

/* For the transmit button */
int transmit;

/* For the Velocity or PWM button */
int velOrPwm;

/* For the smoothing on the nbr field */
int smoothing;

int autoselect;

textbox textboxes[NUM_TEXTBOXES];

void inputInit() {
	currentLED = -1;
	motorLeft = 0;
	motorRight = 0;
	currentTextbox = -1;
	motorsSynced = 1;
	transmit = 0;
	velOrPwm = 0;
	smoothing = 0;

	/* Initialize text boxes. */
	int i = 0;
	for (i = 0; i < NUM_TEXTBOXES; i++) {
		textboxes[i].index = 0;
		switch (i) {
		case TEXTBOX_COM_PORT: {
			textboxes[i].length = LEN_TXT_COM_PORT;
			// Hack to let COM number be shown
			if (comNumber > 0) {
				sprintf(textboxes[i].message, "%d", comNumber);
			}
			textboxes[i].index = strlen(textboxes[i].message);
			break;
		}
		case TEXTBOX_RADIO: {
			textboxes[i].length = LEN_TXT_RADIO;
			break;
		}
		default: {
			break;
		}
		}
	}
}

/* Scroll to the next text box. */
void textboxScroll() {
	if (currentTextbox >= 0) {
		currentTextbox = (currentTextbox + 1) % NUM_TEXTBOXES;
	}
}

/* Clears the current text box and returns its message. */
void textboxReturn() {
	char inputText[MAX_TEXTBOX_LENGTH];

	textbox *txt = &textboxes[currentTextbox];

	//txt->message[strlen(txt->message)] = '\r\n';

	memset(inputText, 0, MAX_TEXTBOX_LENGTH);
	strcpy(inputText, txt->message);




	switch (currentTextbox) {
	case TEXTBOX_COM_PORT: {

		/* General bookkeeping. Moved from outside the cases and duplicated to
		 * avoid it occurring with the IR messages
		 *
		 * TODO: Make it so that it can go back above.
		 */
		memset(txt->message, 0, MAX_TEXTBOX_LENGTH);
		txt->index = 0;

		sscanf(inputText, "%d", &comNumber);
		CloseHandle(hSerial);
		guiInit();
		break;
	}
	case TEXTBOX_RADIO: {
		memset(txt->message, 0, MAX_TEXTBOX_LENGTH);
		txt->index = 0;

//		pthread_mutex_lock(&serialPortMutex);
		/*
		 * The two irrelevant characters before the actual string are to
		 * pad the message as the current robot API kills those two bytes.
		 */
		cprintf("rcr %s\r", inputText);


//char outMsg[MAX_TEXTBOX_LENGTH + 6] = {0};
//sprintf(outMsg, "rcr %s\r\n", inputText);
//writeToOUTFile(outMsg, MAX_TEXTBOX_LENGTH + 6);


//		pthread_mutex_unlock(&serialPortMutex);
		break;
	}
	default: {
		break;
	}}

}

/* Read a character as it is input into a text box */
void readChar(char character) {
	if (currentTextbox < 0) {
		return;
	}
	textbox* txt = &textboxes[currentTextbox];

	switch (character) {
	case '\r':
	case '\n': {
		textboxReturn();
		break;
	}
	/* Backspace handler */
	case '\b': {
		if (txt->index > 0) {
			strdel(txt->message, --txt->index);
		}
		break;
	}
	case '\t': {
		textboxScroll();
		break;
	}
	/* Delete handler */
	case 127: {
		strdel(txt->message, txt->index);
		break;
	}
	/* Default: insert the character */
	default: {
		if (strlen(txt->message) < txt->length) {
			strins(txt->message, character, txt->index++);
		}
		break;
	}}
}

/* Read a GLUT special character as it is input into a text box */
void readSpecialChar(char character) {
	if (currentTextbox < 0) {
		return;
	}
	textbox* txt = &textboxes[currentTextbox];

	switch (character) {
	case GLUT_KEY_LEFT:
		if (txt->index > 0) {
			txt->index--;
		}
		break;
	case GLUT_KEY_RIGHT:
		if (txt->index < strlen(txt->message)) {
			txt->index++;
		}
		break;
	}
}

/* Handle LED button presses */
void LEDButtonHandler(int index) {
	/* If user clicks the LED that is currently on, turn it off. */

	/*
	 * The setup for the broadcast message is:
	 * 		rcl cc,pp,bb,rr
	 *
	 * 	rc is the command
	 *
	 * 	l means lights
	 *
	 * 	cc is the colour (0 for red, 1 for green, 2 for blue, 3
	 * 	for all)
	 *
	 * 	pp is the pattern (03 for solid)
	 *
	 * 	bb is the brightness
	 *
	 * 	rr is the rate
	 */
	if (index == currentLED) {
		cprintf("rcl 00,00,00,00\r");
		currentLED = -1;
		return;
	}
	switch(index) {
	case BUTTON_LED_RED: {
		cprintf("rcl 00,03,0A,08\r");
		currentLED = BUTTON_LED_RED;
		break;
	}
	case BUTTON_LED_GREEN: {
		cprintf("rcl 01,03,0A,08\r");
		currentLED = BUTTON_LED_GREEN;
		break;
	}
	case BUTTON_LED_BLUE: {
		cprintf("rcl 02,03,0A,08\r");
		currentLED = BUTTON_LED_BLUE;
		break;
	}
	case BUTTON_LED_ALL: {
		cprintf("rcl 03,03,0A,08\r");
		currentLED = BUTTON_LED_ALL;
		break;
	}
	case BUTTON_LED_OFF: {
		cprintf("rcl 00,00,00,00\r");
		currentLED = BUTTON_LED_OFF;
		break;
	}
	default: {
		break;
	}
	}
}

/* Handle PWM control button presses */
void motorButtonHandler(int hitType, int index) {
	/* Assign the motor to change the value of. */
	int *motorPtr;
	motorPtr = (hitType == NAME_MOTOR_LEFT) ? &motorLeft : &motorRight;
	switch (index) {
	case MOTOR_ADD10: {
		*motorPtr += 10;
		break;
	}
	case MOTOR_ADD1: {
		*motorPtr += 1;
		break;
	}
	case MOTOR_OFF: {
		*motorPtr = 0;
		break;
	}
	case MOTOR_SUB1: {
		*motorPtr -= 1;
		break;
	}
	case MOTOR_SUB10: {
		*motorPtr -= 10;
		break;
	}
	default: {
		break;
	}}

	if (velOrPwm == PWM) {
		*motorPtr = clamp(*motorPtr, -MAX_PWM, MAX_PWM);
	} else {
		*motorPtr = clamp(*motorPtr, -MAX_VEL, MAX_VEL);
	}
	//*motorPtr = clamp(*motorPtr, -100, 100);
	//cprintf("rcp l,%02X\r\n", (uint8)(int8)motorLeft);
	//cprintf("rcp r,%02X\r\n", (uint8)(int8)motorRight);
}

/* TODO: COMBINE THESE!!! */

/* Handle motor button presses if the motors are synced. */
void motorButtonSyncedHandler(int index) {

	/*
	 * TODO: Make it depend on
	 */

	/*
	 * Increase or decrease the motor values.
	 */
	switch (index) {
	case MOTOR_ADD10: {
		motorLeft += 10;
		motorRight += 10;
		break;
	}
	case MOTOR_ADD1: {
		motorLeft += 1;
		motorRight += 1;
		break;
	}
	case MOTOR_OFF: {
		motorLeft = 0;
		motorRight = 0;
		break;
	}
	case MOTOR_SUB1: {
		motorLeft -= 1;
		motorRight -= 1;
		break;
	}
	case MOTOR_SUB10: {
		motorLeft -= 10;
		motorRight -= 10;
		break;
	}
	default: {
		break;
	}}

	/* Clamp the motor values based on if PWM or VEL */
	if (velOrPwm == PWM) {
		motorLeft = clamp(motorLeft, -MAX_PWM, MAX_PWM);
		motorRight = clamp(motorRight, -MAX_PWM, MAX_PWM);

		//cprintf("rcp l,%02X\r\n", (uint8)(int8)motorLeft);
		//cprintf("rcp r,%02X\r\n", (uint8)(int8)motorRight);
	} else {
		motorLeft = clamp(motorLeft, -MAX_VEL, MAX_VEL);
		motorRight = clamp(motorRight, -MAX_VEL, MAX_VEL);

		//cprintf("rcv l,%04X\r\n", (uint16)(int16)motorLeft);
		//cprintf("rcv r,%04X\r\n", (uint16)(int16)motorRight);
	}
}

/* Handle the pushing of the motorsSynced toggle. */
void motorSyncHandler() {
	motorsSynced = !motorsSynced;
	if (motorsSynced) {
		motorRight = motorLeft;
	}
}

/* Handle the pushing of the velOrPwm button. */
void velOrPwmHandler() {

}

/* Handle the smoothing button */
void smoothingButtonHandler() {
	if (smoothing == 1)
		smoothing = 0;
	else
		smoothing = 1;
}

/* Handle the select button */
void selectButtonHandler() {
	if (autoselect == 1)
		autoselect = 0;
	else
		autoselect = 1;
}

/* Handle the pushing of the transmit button. */
void transmitButtonHandler()
{
	/* If someone hits the transmit button, turn it off or on. */
	transmit = !transmit;

	if (transmit)
		IRtimer = 500;
//		glutTimerFunc(0, timerTransmitIR, 0);
}

/* Handle pushing the velocity button. */
void velocityButtonHandler()
{
	//velOrPwm = !velOrPwm;

	if (velOrPwm != VEL) {
		velOrPwm = VEL;

		motorLeft = 0;
		motorRight = 0;

		/*
		 * TODO: Motor control functions need a switch based on
		 * if velocity or pwm.
		 */
	}
}

/* Handle pushing the velocity button. */
void pwmButtonHandler()
{
	if (velOrPwm != PWM) {
		velOrPwm = PWM;

		motorLeft = 0;
		motorRight = 0;

		/*
		 * TODO: Motor control functions need a switch based on
		 * if velocity or pwm.
		 */
	}
}

/* Picking processing. */
void processHits(GLint hits, GLuint buffer[])
{
	unsigned int i, j;
	GLuint names, *ptr;

//	printf("hits = %d\r\n", hits);

	/* If there are no hits, set no textbox as active. */
	if (hits == 0)
	{
		currentTextbox = -1;
		return;
	}

	int hitType, hitIndex;
	ptr = (GLuint *) buffer;
	for (i = 0; i < hits; i++) {
		names = *ptr;
		ptr += 3;

		if (names == 0)
			continue;

		for (j = 0; j < names; j++)	{
			if (j == 0) {
				hitType = *ptr;
			} else if (j == 1) {
				hitIndex = *ptr;
			}
			ptr++;
		}
		/* Delegate type of hit. */
		switch (hitType) {
		case NAME_TEXTBOX: {
			currentTextbox = hitIndex;
			break;
		}
		case NAME_LED_BUTTON: {
			LEDButtonHandler(hitIndex);
			break;
		}
		case NAME_MOTOR_LEFT:
		case NAME_MOTOR_RIGHT: {
			/*TODO: COMBINE THESE!!! */
			if (motorsSynced) {
				motorButtonSyncedHandler(hitIndex);
			} else {
				motorButtonHandler(hitType, hitIndex);
			}
			break;
		}
		case NAME_MOTOR_SYNC: {
			motorSyncHandler();
			break;
		}
		case NAME_TRANSMIT: {
			transmitButtonHandler();
			break;
		}
		case NAME_VEL: {
			velocityButtonHandler();
			break;
		}
		case NAME_PWM: {
			pwmButtonHandler();
			break;
		}
		case NAME_SMOOTH: {
			smoothingButtonHandler();
			break;
		}
		case NAME_SELECT: {
			selectButtonHandler();
			break;
		}
		default: {
			currentTextbox = -1;
			break;
		}
		}
	}
}
