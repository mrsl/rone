/*
 * drawinput.c
 *
 *  Created on: Sep 7, 2011
 *      Author: nathan
 */

#include "gui.h"

/*
 * Input Drawing
 *
 * Many of these functions take in a [mode] variable. With OpenGL input, there
 * must be options for selecting and drawing.
 */

/* Used for non-textbox input text (such as for motors and IR). */
void
drawTextOutline(float x, float y, char* msg, int msgSize, int alignment)
{
	char copy[msgSize+1];
	strncpy(copy, msg, msgSize);
	copy[msgSize] = '\0';

	textSetAlignment(alignment);
	textSetSize(TEXT_MED);
	glPushMatrix();
		glTranslatef(x, y, 0);
		glColor3fv(color_black);

		GLfloat l, r, u, d;
		if (alignment == ALIGN_LEFT) {
			l = -TEXTBOX_BORDER;
			r = TEXT_MED * gmf[(int)'m'].gmfCellIncX * msgSize
				+ TEXTBOX_BORDER;
			d = -TEXTBOX_BORDER;
			u = TEXT_MED + TEXTBOX_BORDER;
		} else {
			l = -TEXT_MED * gmf[(int)'m'].gmfCellIncX * msgSize
				- TEXTBOX_BORDER;
			r = TEXTBOX_BORDER;
			d = -TEXTBOX_BORDER;
			u = TEXT_MED + TEXTBOX_BORDER;
		}
		glBegin(GL_LINE_LOOP);
			glVertex2f(l, d);
			glVertex2f(r, d);
			glVertex2f(r, u);
			glVertex2f(l, u);
		glEnd();
		glColor3fv(color_black);
		textPrintf(copy);
	glPopMatrix();
}

/*
 * Draws a button that has text written over it.
 *
 * The button switches from red to black based on the input colourSelect (if the
 * switch is 1, it is red; if 0, black.
 *
 * NOTE: This button is designed in the same way as the text button except the
 * text must be drawn outside of it. This is a workaround for a bug that makes
 * the hit box for the button the entire screen if text is drawn with the
 * button. It doesn't happen if only one thing is using the function, but occurs
 * if the function is called multiple times in one cycle. This is a quick
 * workaround.
 */
void
drawEmptyTextButton(float x, float y,
	int msgSize, int alignment, int colourSelect, GLfloat textSize)
{
	/* Do all of the setup (text size, alignment and moving the draw matrix) */
	glPushMatrix();
		glTranslatef(x, y, 0);

		/* Draw the primary box */
		if (colourSelect)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);

		if (alignment == ALIGN_LEFT) {
			glRectf(-TEXTBOX_BORDER,
				-TEXTBOX_BORDER,
				(textSize * gmf[(int)'m'].gmfCellIncX * msgSize)
					+ TEXTBOX_BORDER,
				textSize + TEXTBOX_BORDER);
		} else {
			glRectf(-(textSize * gmf[(int)'m'].gmfCellIncX * msgSize * .5)
					- TEXTBOX_BORDER,
				-TEXTBOX_BORDER,
				(textSize * gmf[(int)'m'].gmfCellIncX * msgSize * .5)
					+ TEXTBOX_BORDER,
				textSize + TEXTBOX_BORDER);
		}

		/* Draw the outline box */
		glColor3fv(color_black);

		GLfloat l, r, u, d;
		if (alignment == ALIGN_LEFT) {
			l = -TEXTBOX_BORDER;
			r = textSize * gmf[(int)'m'].gmfCellIncX * msgSize
				+ TEXTBOX_BORDER;
			d = -TEXTBOX_BORDER;
			u = textSize + TEXTBOX_BORDER;
		} else {
			l = -(textSize * gmf[(int)'m'].gmfCellIncX * msgSize * .5)
				- TEXTBOX_BORDER;
			r = (textSize * gmf[(int)'m'].gmfCellIncX * msgSize * .5)
				+ TEXTBOX_BORDER;
			d = -TEXTBOX_BORDER;
			u = textSize + TEXTBOX_BORDER;
		}

		glBegin(GL_LINE_LOOP);
			glVertex2f(l, d);
			glVertex2f(r, d);
			glVertex2f(r, u);
			glVertex2f(l, u);
		glEnd();
	glPopMatrix();
}

/* LED around each button. */
void
drawCircleButton(GLfloat x, GLfloat y, const GLfloat color[], GLenum mode)
{
	glPushMatrix();
		glTranslatef(x, y, 0);
		glScalef(LED_SIZE, LED_SIZE, 0);
		glColor3fv(color);

		if (mode == GL_RENDER)
			glCallList(LIST_ANNULUS);

		if (mode == GL_SELECT)
			glCallList(LIST_CIRCLE_FILLED);
	glPopMatrix();
}

/* Input button with circle and text. */
void
drawCircleTextButton(GLfloat x, GLfloat y,
	const GLfloat color[], char* text, GLenum mode)
{
	glPushMatrix();
		glTranslatef(x, y, 0);

		glPushMatrix();
			glScalef(CIRCLE_BTN_SIZE, CIRCLE_BTN_SIZE, 0);
			glColor3fv(color);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();

		if (mode == GL_RENDER) {
			glTranslatef(0, -TEXT_SMALL, 0);
			glColor3fv(color_lightgray);
			textSetAlignment(ALIGN_CENTER);
			textSetSize(TEXT_SMALL);
			textPrintf(text);
		}
	glPopMatrix();
}

/*
 * Requires:
 *      An X and Y location for it to be drawn at and a switch (xSwitch) for
 *      if it is active or not.
 *
 * Effects:
 * 		Draws a circular pushbutton at coordinates x,y. The colour is based
 * 		off of the switch. If the switch is high, the colour is red;
 * 		if low, the colour is black. The exterior ring is always black.
 */
void
drawPushButton(GLfloat x, GLfloat y, GLfloat xSwitch, char *text)
{
	/* Push the current matrix and then move it to draw in the correct place. */
	glPushMatrix();
		glTranslatef(x, y, 0);

		/* Determine the colour of the centre of the button and draw it. */
		if (xSwitch)
			glColor3fv(color_red);
		else
			glColor3fv(color_black);

		glPushMatrix();
			glScalef(PUSHBUTTON_RADIUS, PUSHBUTTON_RADIUS, 0);
			glCallList(LIST_PUSHBUTTON_CENTRE);

			/* Draw the outline of the button. */
			glColor3fv(color_black);
			glCallList(LIST_PUSHBUTTON_OUTLINE);
		glPopMatrix();

		/* Draw the text associated with the pushbutton */
		glTranslatef(PUSHBUTTON_TEXT_OFFSET_X, PUSHBUTTON_TEXT_OFFSET_Y, 0);
		textSetAlignment(ALIGN_LEFT);
		textSetSize(TEXT_SMALL);
		textPrintf(text);
	glPopMatrix();
}

/* Delegating function to buttons. */
void drawLEDButton(int index, GLenum mode) {
	switch (index) {
	case BUTTON_LED_RED: {
		drawCircleButton(
			BTNLED_POS_X - BTNLED_OFFSET_X,
			BTNLED_POS_Y,
			(index == currentLED || currentLED == BUTTON_LED_ALL) ?
				color_red : color_gray,
			mode);
		break;
	}
	case BUTTON_LED_GREEN: {
		drawCircleButton(
			BTNLED_POS_X,
			BTNLED_POS_Y + BTNLED_OFFSET_Y,
			(index == currentLED || currentLED == BUTTON_LED_ALL) ?
				color_green : color_gray,
			mode);
		break;
	}
	case BUTTON_LED_BLUE: {
		drawCircleButton(
			BTNLED_POS_X + BTNLED_OFFSET_X,
			BTNLED_POS_Y,
			(index == currentLED || currentLED == BUTTON_LED_ALL) ?
				color_blue : color_gray,
			mode);
		break;
	}
	default: {
		break;
	}
	}
}

void
drawMotorIncOne(GLfloat x, GLfloat y)
{
	glPushMatrix();
		glColor3fv(color_gray);
		glTranslatef(x, y, 0);
		glScalef(CIRCLE_BTN_SIZE, CIRCLE_BTN_SIZE, 0);
		glCallList(LIST_TRIANGLE);
	glPopMatrix();
}

void
drawMotorIncTen(GLfloat x, GLfloat y)
{
	glPushMatrix();
		glColor3fv(color_gray);
		glTranslatef(x, y, 0);
		glScalef(CIRCLE_BTN_SIZE, CIRCLE_BTN_SIZE, 0);
		glCallList(LIST_TRIANGLE_HALF);
		glTranslatef(0, -1, 0);
		glCallList(LIST_TRIANGLE_HALF);
	glPopMatrix();
}

void
drawMotorStop(GLfloat x, GLfloat y)
{
	glPushMatrix();
		glColor3fv(color_gray);
		glTranslatef(x, y, 0);
		glScalef(CIRCLE_BTN_SIZE, CIRCLE_BTN_SIZE, 0);
		glCallList(LIST_SQUARE);
	glPopMatrix();
}

void
drawMotorDecOne(GLfloat x, GLfloat y)
{
	glPushMatrix();
		glColor3fv(color_gray);
		glTranslatef(x, y, 0);
		glScalef(CIRCLE_BTN_SIZE, -CIRCLE_BTN_SIZE, 0);
		glCallList(LIST_TRIANGLE);
	glPopMatrix();
}

void
drawMotorDecTen(GLfloat x, GLfloat y)
{
	glPushMatrix();
		glColor3fv(color_gray);
		glTranslatef(x, y, 0);
		glScalef(CIRCLE_BTN_SIZE, -CIRCLE_BTN_SIZE, 0);
		glCallList(LIST_TRIANGLE_HALF);
		glTranslatef(0, -1 , 0);
		glCallList(LIST_TRIANGLE_HALF);
	glPopMatrix();
}

/* Delegate to draw motor controls */
void
drawMotorControls(GLfloat x, GLfloat y, int index, GLenum mode)
{
	switch (index) {
	case MOTOR_ADD10: 	{
		drawMotorIncTen(x, y + MOTOR_OFFSET_Y_10);
		break;
	}
	case MOTOR_ADD1: 	{
		drawMotorIncOne(x, y + MOTOR_OFFSET_Y_1);
		break;
	}
	case MOTOR_OFF: 	{
		drawMotorStop(x, y);
		break;
	}
	case MOTOR_SUB1: 	{
		drawMotorDecOne(x, y - MOTOR_OFFSET_Y_1);
		break;
	}
	case MOTOR_SUB10: 	{
		drawMotorDecTen(x, y - MOTOR_OFFSET_Y_10);
		break;
	}
	default: {
		break;
	}
	}
}

void
drawMotorSyncButton(GLfloat x, GLfloat y, GLenum mode)
{
	glPushMatrix();
		glTranslatef(x, y, 0);
		// Draw connecting lines
		if (mode == GL_RENDER) {
			glColor3fv(color_black);
			glBegin(GL_LINES);
			glVertex2f(-MOTOR_SYNC_OFFSET_X, 0);
			glVertex2f(MOTOR_SYNC_OFFSET_X, 0);
			glEnd();
		}

		glScalef(CIRCLE_BTN_SIZE, CIRCLE_BTN_SIZE, 0);
		/* Draw sync button */
		glColor3fv(color_gray);
		glCallList(LIST_CIRCLE_FILLED);

		/* Draw pretty part */
		if (mode == GL_RENDER) {
			glColor3fv(color_black);
			glBegin(GL_LINES);
			if (motorsSynced) {
				glVertex2i(-1, 0);
				glVertex2i(1, 0);
			} else {
				glVertex2i(0, 1);
				glVertex2i(0, -1);
			}
			glEnd();
		}
	glPopMatrix();

}


/* Draw a plain texbox (used for most textboxes)*/
void
drawTextboxPlain(GLfloat x, GLfloat y,
	textbox *box, GLboolean isactive, GLenum mode)
{
	glPushMatrix();
		glTranslatef(x, y, 0);

		glColor3fv(color_black);
		glRectf(-TEXTBOX_BORDER,
			-TEXTBOX_BORDER,
			TEXT_MED * gmf[(int)'m'].gmfCellIncX * box->length
				+ TEXTBOX_BORDER,
			TEXT_MED + TEXTBOX_BORDER);

		if (mode == GL_RENDER) {
			textSetAlignment(ALIGN_LEFT);
			textSetSize(TEXT_MED);
			glColor3fv(color_white);
			textPrintf(box->message);

			if (isactive) {
				float length = 0;
				int loop;
				for (loop = 0; loop < (box->index); loop++)
					length += gmf[(int)box->message[loop]].gmfCellIncX;

				glPushMatrix();
					glTranslatef(length * TEXT_MED, 0, 0);
					glBegin(GL_LINES);
						glVertex2f(0.0, TEXT_MED);
						glVertex2f(0.0, 0.0);
					glEnd();
				glPopMatrix();
			}
		}
	glPopMatrix();
}

/* Draw all the textboxes */
void
drawTextbox(int index, GLenum mode)
{
	switch (index) {
	case TEXTBOX_COM_PORT: {
		if (mode == GL_RENDER) {
			textSetAlignment(ALIGN_RIGHT);
			textSetSize(TEXT_SMALL);

			glColor3fv(color_black);
			glPushMatrix();
				glTranslatef(COMPORT_POS_X, COMPORT_POS_Y, 0);
				textPrintf("COM port ");
			glPopMatrix();
		}
		drawTextboxPlain(COMPORT_POS_X, COMPORT_POS_Y, &textboxes[index],
			(index == currentTextbox), mode);

		break;
	}
	case TEXTBOX_RADIO: {
		if (mode == GL_RENDER) {
			textSetAlignment(ALIGN_RIGHT);
			textSetSize(TEXT_SMALL);

			glColor3fv(color_black);
			glPushMatrix();
				glTranslatef(RADIO_POS_X, RADIOIN_POS_Y, 0);
				textPrintf("Radio ");
			glPopMatrix();
		}
		drawTextboxPlain(RADIO_POS_X, RADIOIN_POS_Y, &textboxes[index],
			(index == currentTextbox), mode);

		break;
	}
	default: {
		break;
	}
	}
}

/* Draws the input to send to the robot. */
void
drawInput(GLenum mode)
{
	GLuint i;
	/* Draw LED buttons */
	if (mode == GL_SELECT) {
		glLoadName(NAME_LED_BUTTON);
	}
	for (i = 0; i < NUM_LED_BUTTONS; i++) {
		if (mode == GL_SELECT)
			glPushName(i);

		drawLEDButton(i, mode);

		if (mode == GL_SELECT)
			glPopName();
	}

	/*
	 * Draw Motor buttons
	 */
	/* Left motor */
	if (mode == GL_SELECT)
		glLoadName(NAME_MOTOR_LEFT);

	for (i = 0; i < NUM_MOTOR_LEFT; i++) {
		if (mode == GL_SELECT)
			glPushName(i);

		drawMotorControls(MOTOR_POS_X - MOTOR_OFFSET_X, MOTOR_POS_Y, i, mode);

		if (mode == GL_SELECT)
			glPopName();
	}

	/* Right motor */
	if (mode == GL_SELECT)
		glLoadName(NAME_MOTOR_RIGHT);

	for (i = 0; i < NUM_MOTOR_RIGHT; i++) {
		if (mode == GL_SELECT)
			glPushName(i);

		drawMotorControls(MOTOR_POS_X + MOTOR_OFFSET_X, MOTOR_POS_Y, i, mode);

		if (mode == GL_SELECT)
			glPopName();
	}

	/* Motor sync button */
	if (mode == GL_SELECT)
		glLoadName(NAME_MOTOR_SYNC);

	drawMotorSyncButton(MOTOR_POS_X, MOTOR_SYNC_POS_Y, mode);

	/* PWM text display. */
	char textMotor[10];
	if (mode == GL_RENDER) {
		sprintf(textMotor, "%d", motorLeft);
		drawTextOutline(-MOTOR_TEXT_OFFSET_X, MOTOR_TEXT_POS_Y,
			textMotor, 4, ALIGN_LEFT);

		sprintf(textMotor, "%d", motorRight);
		drawTextOutline(MOTOR_TEXT_OFFSET_X, MOTOR_TEXT_POS_Y,
			textMotor, 4, ALIGN_RIGHT);
	}

	/*
	 * Draw textboxes.
	 */
	if (mode == GL_SELECT)
		glLoadName(NAME_TEXTBOX);

	for (i = 0; i < NUM_TEXTBOXES; i++) {
		if (mode == GL_SELECT)
			glPushName(i);

		drawTextbox(i, mode);

		if (mode == GL_SELECT)
			glPopName();
	}

	/*
	 * Draw the Velocity or PWM selector.
	 */
	if (mode == GL_SELECT)
		glLoadName(NAME_VEL);

	drawPushButton(VEL_X, VEL_Y, velOrPwm, "Velocity");

	if (mode == GL_SELECT)
		glLoadName(NAME_PWM);

	drawPushButton(PWM_X, PWM_Y, !velOrPwm, "PWM");

	/*
	 * Draw the smoothing buttom
	 */
	if (mode == GL_SELECT)
		glLoadName(NAME_SMOOTH);

	drawPushButton(SMOOTH_POS_X, SMOOTH_POS_Y, smoothing, "Smooth");


	/*
	 * Draw the auto select button
	 */
	if (mode == GL_SELECT)
		glLoadName(NAME_SELECT);

	drawPushButton(SELECT_POS_X, SELECT_POS_Y, autoselect, "Auto-Select");
}
