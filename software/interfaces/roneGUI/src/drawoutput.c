/*
 * drawout.c
 *
 *  Created on: Sep 7, 2011
 *      Author: nathan
 */

#include "gui.h"

/* Draws a 1d grid with a point position as a bar.*/
void
drawGrid1d(float pos)
{
	glPushMatrix();
		/* Draw the base */
		glColor3fv(color_lightgray);
		glCallList(LIST_SQUARE);

		/* Draw the axis */
		glColor3fv(color_black);
		glBegin(GL_LINES);
			glVertex2i(-1, 0);
			glVertex2i(1, 0);
		glEnd();

		/* Clamp the value */
		pos = clamp(pos, -1.0, 1.0);

		/* Draw the value */
		glPushMatrix();
			glColor3fv(color_red);
			glTranslatef(0, pos, 0);
			glBegin(GL_LINES);
				glVertex2i(-1, 0);
				glVertex2i(1, 0);
			glEnd();
		glPopMatrix();
	glPopMatrix();
}

/* Draws a 2d grid with a point position */
void
drawGrid2d(float x, float y)
{
	glPushMatrix();
		/* Draw the axes */
		glColor3fv(color_black);
		glCallList(LIST_AXES2D);

		/* Clamp the values */
		x = clamp(x, -1.0, 1.0);
		y = clamp(y, -1.0, 1.0);

		/* Draw the value */
		glColor3fv(color_red);
		glBegin(GL_POINTS);
			glVertex2f(x, y);
		glEnd();
	glPopMatrix();
}

/* Draws a button that is either on or off. */
void drawButton(const GLfloat color[]) {
	glPushMatrix();
		glColor3fv(color);
		glCallList(LIST_CIRCLE_FILLED);
	glPopMatrix();
}

/* Draws the bump sensor outputs.*/
void
drawBumpSensors(GLfloat x, GLfloat y, const GLfloat color[], int bumpBits)
{
	int i;
	glPushMatrix();
		glTranslatef(x, y, 0);
		glScalef(ROBOT_RADIUS, ROBOT_RADIUS, 0);
		glRotatef(22.5, 0, 0, 1);
		glColor3fv(color);
		for (i=0; i<NUM_BUMP_SENSORS; i++) {
			if ((bumpBits >> i) % 2)
				glCallList(LIST_ANNULUS_SECT);
			glRotatef(45, 0, 0, 1);
		}
	glPopMatrix();
}

/* Draws a circle with a point at pos. */
void
drawCirclePointer(float pos)
{
	glPushMatrix();
		/* Draw axis */
		glColor3fv(color_black);
		glPushMatrix();
			glRotatef(ANGLE_90, 0, 0, 1);
			glCallList(LIST_SEMICIRCLE);
		glPopMatrix();

		pos = clamp(pos, -1.0, 1.0);

		/* Draw value */
		glColor3fv(color_red);
		glPushMatrix();
			glRotatef(pos * ANGLE_90, 0, 0, 1);
			glBegin(GL_POINTS);
				glVertex2i(0, 1);
			glEnd();
		glPopMatrix();
	glPopMatrix();
}

/* Draws a speedometer.*/
void
drawSpeedometer(float velocity, float max_speed)
{
	glPushMatrix();
		/* Draw axis */
		glColor3fv(color_black);
		glPushMatrix();
			glCallList(LIST_SEMICIRCLE);
			glBegin(GL_LINES);
				glVertex2i(0, 0);
				glVertex2i(1, 0);
			glEnd();
		glPopMatrix();

		/* Calculate angle from value */
		float angle = (float) velocity / (max_speed * 2) * ANGLE_90;

		/* Draw pointer */
		glColor3fv(color_red);
		glPushMatrix();
			glRotatef(angle, 0, 0, 1);
			glBegin(GL_LINES);
				glVertex2i(0, 0);
				glVertex2i(1, 0);
			glEnd();
		glPopMatrix();
	glPopMatrix();
}

/* Draws the base of the IR deflection */
void
drawIRRobot(GLfloat x, GLfloat y)
{
	int i;
	glPushMatrix();
		glTranslatef(x, y, 0);

		/* Draw shaded outer area */
		glPushMatrix();
			glScalef(IR_FIELD_RADIUS, IR_FIELD_RADIUS, 0);
			glColor3fv(color_lightgray);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();

		/* Draw robot */
		glPushMatrix();
			glScalef(IR_ROBOT_RADIUS, IR_ROBOT_RADIUS, 0);
			glColor3fv(color_black);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();

		/* Draw marker bar */
		glPushMatrix();
			glScalef(IR_ROBOT_RADIUS / 16., IR_ROBOT_RADIUS, 0);
			glColor3fv(color_lightgray);
			glRecti(-1, 0, 1, 2);
		glPopMatrix();

		glColor3fv(color_black);

		/* Draw transmitters */
		glPushMatrix();
		for (i = 0; i < NUM_IR_SENSORS; i++) {
			glPushMatrix();
				glTranslatef(0, IR_ROBOT_RADIUS + IR_TRIANGLE, 0);
				glScalef(IR_TRIANGLE, IR_TRIANGLE, 0);
				glCallList(LIST_IR_COMPONENTS);
			glPopMatrix();
			glRotatef(45, 0, 0, 1);
		}
		glPopMatrix();

		/* Draw recievers */
		glPushMatrix();
		glRotatef(22.5, 0, 0, 1);
		for (i = 0; i < NUM_IR_SENSORS; i++) {
			glPushMatrix();
				glTranslatef(0, IR_ROBOT_RADIUS + IR_TRIANGLE, 0);
				glScalef(IR_TRIANGLE, -IR_TRIANGLE, 0);
				glCallList(LIST_IR_COMPONENTS);
			glPopMatrix();
			glRotatef(45, 0, 0, 1);
		}
		glPopMatrix();
	glPopMatrix();
}

void
drawIRBounce(GLfloat x, GLfloat y, unsigned int irData[])
{
	int i;
	int j;
	float powIR, powOuter;
	float recvAngle, xmitAngle, min;
	float centerAngle, reflectionAngle;
	float arrowLength;

	/* Exponents to be used in calculations */
	powIR = powf(IR_ARROW_RADIUS, 2.);
	powOuter = powf(IR_FIELD_RADIUS, 2.);

	glPushMatrix();
	glTranslatef(x, y, 0);
	for (i = 0; i < NUM_IR_SENSORS; i++) {
		for (j = 0; j < NUM_IR_SENSORS; j++) {
			/* If Receiver j received Transmitter i */
			if ((irData[j] >> i) % 2) {

				/* Calculate receiver and transmitter angles */
				recvAngle = i * 45 + 22.5;
				xmitAngle = j * 45;

				/* Calculate center between the two */
				centerAngle = (recvAngle + xmitAngle) / 2;
				min = recvAngle >= xmitAngle ? xmitAngle : recvAngle;

				if (abs(recvAngle - xmitAngle) > 180)
					centerAngle -= 180;

				/* Find the length of the reflecting arrow */
				arrowLength = sqrt(powIR + powOuter -
					2. * IR_ARROW_RADIUS * IR_FIELD_RADIUS *
					cos(PI / 180. * abs(centerAngle - min)));

				/* Find reflection angle off the outer radius */
				reflectionAngle = 180. / PI *
					acos((powIR - powf(arrowLength, 2.) - powOuter) /
						(-2 * arrowLength * IR_FIELD_RADIUS));

				glPushMatrix();
					glColor3fv(color_red);

					glRotatef(centerAngle, 0, 0, 1);

					glTranslatef(0, IR_FIELD_RADIUS, 0);
					glRotatef(reflectionAngle, 0, 0, 1);

					/* Draw circle at top */
					glPushMatrix();
						glScalef(IR_TRIANGLE, IR_TRIANGLE, 0);
						glColor3fv(color_red);
						glCallList(LIST_CIRCLE_FILLED);
					glPopMatrix();

					/* Draw reflecting lines */
					glBegin(GL_LINES);
						glVertex2f(0, 0);
						glVertex2f(0, -arrowLength);
					glEnd();

					glRotatef(-2 * reflectionAngle, 0, 0, 1);

					glBegin(GL_LINES);
						glVertex2f(0, 0);
						glVertex2f(0, -arrowLength);
					glEnd();

				glPopMatrix();

				glLineWidth(LINE_WIDTH);
				glColor3fv(color_red);

				/* Draw highlighted transmitter and receiver */
				glPushMatrix();
					glRotatef(recvAngle, 0, 0, 1);
					glTranslatef(0, IR_ROBOT_RADIUS + IR_TRIANGLE, 0);
					glScalef(IR_TRIANGLE, -IR_TRIANGLE, 0);
					glCallList(LIST_IR_COMPONENTS);
				glPopMatrix();

				glPushMatrix();
					glRotatef(xmitAngle, 0, 0, 1);
					glTranslatef(0, IR_ROBOT_RADIUS + IR_TRIANGLE, 0);
					glScalef(IR_TRIANGLE, IR_TRIANGLE, 0);
					glCallList(LIST_IR_COMPONENTS);
				glPopMatrix();
			}
		}
	}
	glPopMatrix();
}

void
drawNBRMap(GLfloat x, GLfloat y)
{
	glPushMatrix();
		glTranslatef(x, y, 0);

		glPushMatrix();
			glScalef(NBR_FIELD_SIZE, NBR_FIELD_SIZE, 0);
			glColor3fv(color_lightgray);
			glCallList(LIST_SQUARE);
			glColor3fv(color_black);
			glCallList(LIST_AXES2D);
		glPopMatrix();
	glPopMatrix();
}

/**
 * Draw a robot on the nbr map
 */
void
drawNBR(nbrData nbr)
{
	glPushMatrix();
		/* Move frame to relative nbr location */
		glRotatef(nbr.bearing, 0, 0, 1);
		/* Scale the range to fit on the map, and use the scaling constant to
		 * adjust given range number
		 */
		if (nbr.range != -1) /* Only translate if not this robot */
			glTranslatef(0,
				clamp(nbr.range * NBR_RANGE_SCALE, 2 * NBR_ROBOT_RADIUS,
				NBR_FIELD_SIZE - NBR_ROBOT_RADIUS - TEXT_SMALL),
				0);

		glRotatef(-nbr.bearing, 0, 0, 1);

		/* Draw robot and marker bar */
		glPushMatrix();
			glRotatef(180 - nbr.orientation + nbr.bearing, 0, 0, 1);
			glPushMatrix();
				glScalef(NBR_ROBOT_RADIUS, NBR_ROBOT_RADIUS, 0);
				glColor3fv(color_black);
				glCallList(LIST_CIRCLE_FILLED);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();

			glTranslatef(0, NBR_ROBOT_RADIUS, 0);
			glColor3fv(color_lightgray);

			glPushMatrix();
				glScalef(0.1, NBR_ROBOT_RADIUS, 0);
				glBegin(GL_LINES);
					glVertex2i(0, 0);
					glVertex2i(0, -1);
				glEnd();
			glPopMatrix();


		glPopMatrix();

		/* Draw ID beneath robot */
		glPushMatrix();
			glTranslatef(2 * NBR_ROBOT_RADIUS, -3 * NBR_ROBOT_RADIUS, 0);
			if (nbr.id > 0) {
				glColor3fv(color_white);
				textSetAlignment(ALIGN_CENTER);
				textSetSize(TEXT_SMALL);
				if (nbr.id < 100)
					textPrintf("%02d", nbr.id);
				else
					textPrintf("%d", nbr.id);
			}
		glPopMatrix();
	glPopMatrix();
}

/* Draw the active nbr map */
void
drawNBRs(GLfloat x, GLfloat y, int idData, int numNbrs, nbrData nbrs[])
{
	int i;

	glPushMatrix();
		glTranslatef(x, y, 0);
		/* Draw a connecting bar */
		for (i = 0; i < numNbrs; i++) {
			if (nbrs[i].id != 0) {
				glPushMatrix();
					glRotatef(nbrs[i].bearing, 0, 0, 1);
					glScalef(0.05,
						clamp(nbrs[i].range * NBR_RANGE_SCALE, 0, 4.25), 0);
					glColor3fv(color_red);
					glBegin(GL_LINES);
						glVertex2i(0, 0);
						glVertex2i(0, 1);
					glEnd();
				glPopMatrix();
			}
		}

		nbrData thisRobot = (nbrData){idData, 0, 180, -1};
		drawNBR(thisRobot);

		/* Draw the robot */
		for (i = 0; i < numNbrs; i++) {
			if (nbrs[i].id != 0)
				drawNBR(nbrs[i]);
		}

	glPopMatrix();
}

/* Draws a robot */
void
drawRobot(GLfloat x, GLfloat y, int idData)
{
	glPushMatrix();
		glTranslatef(x, y, 0);
		glPushMatrix();
			glScalef(ROBOT_RADIUS, ROBOT_RADIUS, 0);
			glColor3fv(color_black);
			glCallList(LIST_CIRCLE_FILLED);
		glPopMatrix();

		if (idData > 0) {
			glTranslatef(0, ROBOT_RADIUS / 2., 0);
			glColor3fv(color_white);
			textSetAlignment(ALIGN_CENTER);
			textSetSize(TEXT_LARGE);
			if (idData < 100)
				textPrintf("%02d", idData);
			else
				textPrintf("%d", idData);
		}
	glPopMatrix();
}

void
drawButtons(GLfloat x, GLfloat y, unsigned int buttons[])
{
	glPushMatrix();
		glTranslatef(x, y, 0);

		glPushMatrix();
			glTranslatef(-BTNLED_OFFSET_X, 0, 0);
			glScalef(BTN_SIZE, BTN_SIZE, 0);
			drawButton(buttons[BTN_RED] ? color_red : color_darkgray);
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0, BTNLED_OFFSET_Y, 0);
			glScalef(BTN_SIZE, BTN_SIZE, 0);
			drawButton(buttons[BTN_GREEN] ? color_green : color_darkgray);
		glPopMatrix();

		glPushMatrix();
			glTranslatef(BTNLED_OFFSET_X, 0, 0);
			glScalef(BTN_SIZE, BTN_SIZE, 0);
			drawButton(buttons[BTN_BLUE] ? color_blue : color_darkgray);
		glPopMatrix();
	glPopMatrix();
}

/* Draws a semi-filled bar. */
void
drawBar(float frac)
{
	glPushMatrix();
		/* Draw the base */
		glColor3fv(color_lightgray);
		glRecti(-1, 0, 1, 1);

		/* Draw the value */
		frac = clamp(frac, 0.0, 1.0);
		glPushMatrix();
			glColor3fv(color_red);
			glScalef(1, frac, 1);
			glRecti(-1, 0, 1, 1);
		glPopMatrix();
	glPopMatrix();
}

/* Draw a light sensor */
void
drawLightSensor(GLfloat x, GLfloat y, char name[], int lsData)
{
	glPushMatrix();
		/* Move to the right position */
		glTranslatef(x, y, 0);

		if (lsData > 1024)
			lsData = 0;

		/* Draw the bar */
		glPushMatrix();
			if (name[1] == 'c') {
				glRotatef(90, 0, 0, 1);
				glTranslatef(LS_WIDTH, -1.6, 0);
			}
			glScalef(LS_WIDTH / 2, LS_HEIGHT, 0);

			drawBar((GLfloat) lsData / MAX_LIGHT_SENSOR_VALUE);
		glPopMatrix();

		/* Align to position */
		if (name[0] == 'f') {
			if (name[1] == 'l')
				glTranslatef(-LS_TEXT_OFFSET, LS_HEIGHT + 0.1, 0);
			else
				glTranslatef(LS_TEXT_OFFSET, LS_HEIGHT + 0.1, 0);
		} else if (name[0] == 'b') {
			if (name[1] == 'l')
				glTranslatef(-LS_TEXT_OFFSET, -TEXT_SMALL, 0);
			else if (name[1] == 'r')
				glTranslatef(LS_TEXT_OFFSET, -TEXT_SMALL, 0);
			else
				glTranslatef(LS_TEXT_OFFSET, -TEXT_SMALL, 0);
		}

		/* Draw the text value */
		glColor3fv(color_black);
		textSetAlignment(ALIGN_CENTER);
		textSetSize(TEXT_SMALL);
		textPrintf("%s:%4d", name, lsData);
	glPopMatrix();
}

/* Draw all the light sensors */
void
drawLightSensors(GLfloat x, GLfloat y, const unsigned int lsData[], int toOffset, int v11)
{
	int lightSensorOffset = (toOffset) ? LS_OFFSET_X_G : LS_OFFSET_X;

	glPushMatrix();
		glTranslatef(x, y - 1.5, 0);
		if (v11) {
			drawLightSensor(-lightSensorOffset, LS_OFFSET_Y, "fl",
				lsData[LS_FRONT_LEFT]);
			drawLightSensor(lightSensorOffset, LS_OFFSET_Y, "fr",
				lsData[LS_FRONT_RIGHT]);
			drawLightSensor((toOffset) ? lightSensorOffset : 0,
				-LS_OFFSET_Y, "bc",
				lsData[LS_BACK_RIGHT]);
		} else {
			drawLightSensor(-lightSensorOffset, LS_OFFSET_Y, "fl", lsData[LS_FRONT_LEFT]);
			drawLightSensor(lightSensorOffset, LS_OFFSET_Y, "fr", lsData[LS_FRONT_RIGHT]);
			drawLightSensor(-lightSensorOffset, -LS_OFFSET_Y, "bl", lsData[LS_BACK_LEFT]);
			drawLightSensor(lightSensorOffset, -LS_OFFSET_Y, "br", lsData[LS_BACK_RIGHT]);
		}
	glPopMatrix();

}

/* Draw an accelerometer, 4x4 x-y plane and 1x4 z axis */
void
drawAccelerometer(GLfloat x, GLfloat y, const axisData aData)
{
	glPushMatrix();
		/* Translate to right position */
		glTranslatef(x, y, 0);

		glPushMatrix();
			/* Draw x-y plane */
			glTranslatef(0, ACC_SCALE, 0);
			glPushMatrix();
				glScalef(ACC_SCALE, ACC_SCALE, 0);
				glColor3fv(color_lightgray);
				glCallList(LIST_SQUARE);
				glRotatef(ANGLE_90, 0, 0, 1); /* Robot x-y axes rotated ccw */
				drawGrid2d((GLfloat) aData.x / MAX_ACCELEROMETER_VALUE,
						(GLfloat) aData.y / MAX_ACCELEROMETER_VALUE);
			glPopMatrix();

			/* Draw z axis */
			glTranslatef(ACC_SCALE * 1.5, 0, 0);
			glPushMatrix();
				glScalef(ACC_SCALE, ACC_SCALE, 0);
				glScalef(.25, 1, 0);
				drawGrid1d((GLfloat) aData.z / MAX_ACCELEROMETER_VALUE);
			glPopMatrix();
		glPopMatrix();

		/* Draw text values */
		glTranslatef(-ACC_SCALE, -TEXT_SMALL, 0);
		glColor3fv(color_black);
		textSetAlignment(ALIGN_LEFT);
		textSetSize(TEXT_SMALL);
		textPrintf("accelerometer");

		glTranslatef(0, -TEXT_SMALL, 0);
		textPrintf("x:%5d, y:%5d, z:%5d", aData.x, aData.y, aData.z);

	glPopMatrix();
}

/* Draw a gyro. */
void
drawGyro(GLfloat x, GLfloat y, const axisData gData)
{
	glPushMatrix();
		glTranslatef(x, y, 0);

		/* Draw data */
		glPushMatrix();
			glScalef(GYRO_RADIUS + 0.25, GYRO_RADIUS + 0.25, 0);
			drawCirclePointer((GLfloat) gData.z/MAX_GYRO_VALUE);
		glPopMatrix();

		glPushMatrix();
			glScalef(GYRO_RADIUS, GYRO_RADIUS, 0);
			glColor3fv(color_lightgray);
			glCallList(LIST_CIRCLE_FILLED);
			drawGrid2d((GLfloat) gData.x / MAX_GYRO_VALUE,
				(GLfloat) gData.y / MAX_GYRO_VALUE);
		glPopMatrix();


		/* Draw text values */
		glTranslatef(GYRO_RADIUS+TEXT_SMALL, GYRO_RADIUS-TEXT_SMALL, 0);
		glColor3fv(color_black);
		textSetAlignment(ALIGN_LEFT);
		textSetSize(TEXT_SMALL);
		textPrintf("gyro");

		glTranslatef(0, -TEXT_SMALL * 2, 0);
		textPrintf("x:%6d", gData.x);

		glTranslatef(0, -TEXT_SMALL, 0);
		textPrintf("y:%6d", gData.y);

		glTranslatef(0, -TEXT_SMALL, 0);
		textPrintf("z:%6d", gData.z);
	glPopMatrix();
}

void
drawGripper(GLfloat x, GLfloat y, const gripData gripper)
{
	int i;

	if (gripper.isOn) {
		glPushMatrix();
			glTranslatef(x, y, 0);

			if (gripper.gripped)
				glColor3fv(color_red);
			else
				glColor3fv(color_black);

			glPushMatrix();
				glScalef(GRIPPER_RADIUS, GRIPPER_RADIUS, 0);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();

			glPushMatrix();
				glPushMatrix();
					glTranslatef(0, GRIPPER_RADIUS - GRIPPER_CLAW_SIZE_Y, 0);
					glScalef(GRIPPER_CLAW_SIZE_X, GRIPPER_CLAW_SIZE_Y, 0);
					glCallList(LIST_GRIPPER_CLAWS);
				glPopMatrix();
				for (i = 1; i < 4; i++) {
					glPushMatrix();
						glRotatef(i * 51.43, 0, 0, 1);
						glTranslatef(0, GRIPPER_RADIUS - GRIPPER_CLAW_SIZE_Y, 0);
						glScalef(GRIPPER_CLAW_SIZE_X, GRIPPER_CLAW_SIZE_Y, 0);
						glCallList(LIST_GRIPPER_CLAWS);
					glPopMatrix();
				}
				for (i = 1; i < 4; i++) {
					glPushMatrix();
						glRotatef(i * -51.43, 0, 0, 1);
						glTranslatef(0, GRIPPER_RADIUS - GRIPPER_CLAW_SIZE_Y, 0);
						glScalef(GRIPPER_CLAW_SIZE_X, GRIPPER_CLAW_SIZE_Y, 0);
						glCallList(LIST_GRIPPER_CLAWS);
					glPopMatrix();
				}
			glPopMatrix();

			glPushMatrix();
				glColor3fv(color_white);
				glScalef(GRIPPER_RADIUS + 0.1, GRIPPER_RADIUS + 0.1, 0);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();

			if (gripper.gripped)
				glColor3fv(color_red);
			else
				glColor3fv(color_black);

			glPushMatrix();
				glScalef(GRIPPER_RADIUS, GRIPPER_RADIUS, 0);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();

			glPushMatrix();
				glRotatef(180 - GRIPPER_ANGLE_MAX / 2, 0, 0, 1);
				glRotatef((GRIPPER_ANGLE_MAX * (float)gripper.servo) / 180.,
						0, 0, 1);

				glPushMatrix();
					glTranslatef(0, GRIPPER_RADIUS - GRIPPER_CLAW_SIZE_Y, 0);
					glRotatef(180, 0, 0, 1);
					glTranslatef(0, -2 * GRIPPER_CLAW_SIZE_Y, 0);
					glScalef(GRIPPER_CLAW_SIZE_X, GRIPPER_CLAW_SIZE_Y, 0);
					glCallList(LIST_SQUARE);
				glPopMatrix();
				for (i = 1; i < 4; i++) {
					glPushMatrix();
						glRotatef(i * 51.43, 0, 0, 1);
						glTranslatef(0, GRIPPER_RADIUS - GRIPPER_CLAW_SIZE_Y, 0);
						glRotatef(180, 0, 0, 1);
						glTranslatef(0, -3 * GRIPPER_CLAW_SIZE_Y, 0);
						glScalef(GRIPPER_CLAW_SIZE_X, GRIPPER_CLAW_SIZE_Y, 0);
						glCallList(LIST_GRIPPER_CLAWS);
					glPopMatrix();
				}
				for (i = 1; i < 4; i++) {
					glPushMatrix();
						glRotatef(i * -51.43, 0, 0, 1);
						glTranslatef(0, GRIPPER_RADIUS - GRIPPER_CLAW_SIZE_Y, 0);
						glRotatef(180, 0, 0, 1);
						glTranslatef(0, -3 * GRIPPER_CLAW_SIZE_Y, 0);
						glScalef(GRIPPER_CLAW_SIZE_X, GRIPPER_CLAW_SIZE_Y, 0);
						glCallList(LIST_GRIPPER_CLAWS);
					glPopMatrix();
				}
			glPopMatrix();

			/*
			glPushMatrix();
				glScalef(GRIPPER_RADIUS, GRIPPER_RADIUS, 0);
				glRotatef(180, 0, 0, 1);
				glCallList(LIST_ANNULUS_SECT);
			glPopMatrix();
			*/

			glPushMatrix();
				glScalef(SEPERATOR_RADIUS, SEPERATOR_RADIUS, 0);
				glColor3fv(color_white);
				glCallList(LIST_CIRCLE_FILLED);
			glPopMatrix();


		glPopMatrix();
	}
}

void
drawEncoders(GLfloat x, GLfloat y, const encoderData eData[])
{
	glPushMatrix();
		glTranslatef(x, y, 0);

		glPushMatrix();
			glTranslatef(ENC_OFFSET_X, ENC_RADIUS, 0);
			glScalef(ENC_RADIUS, ENC_RADIUS, 0);
			drawSpeedometer(eData[ENC_RIGHT].velocity, MAX_SPEED);
		glPopMatrix();

		glPushMatrix();
			glTranslatef(-ENC_OFFSET_X, ENC_RADIUS, 0);
			glScalef(-ENC_RADIUS, ENC_RADIUS, 0);
			drawSpeedometer(eData[ENC_LEFT].velocity, MAX_SPEED);
		glPopMatrix();

		glColor3fv(color_black);
		textSetSize(TEXT_SMALL);
		textSetAlignment(ALIGN_CENTER);

		glTranslatef(0, -TEXT_SMALL * 2, 0);
		textPrintf(" left  encoders  right");

		glTranslatef(0, -TEXT_SMALL, 0);
		textPrintf("%05d    ticks   %05d",
			eData[ENC_LEFT].ticks, eData[ENC_RIGHT].ticks);

		glTranslatef(0, -TEXT_SMALL, 0);
		textPrintf("%5d  velocity  %-5d",
			eData[ENC_LEFT].velocity, eData[ENC_RIGHT].velocity);
	glPopMatrix();
}

void
drawText(float x, float y, char* msg, int size)
{
	char copy[size+1];
	strncpy(copy, msg, size);
	copy[size] = '\0';

	textSetAlignment(ALIGN_LEFT);
	textSetSize(TEXT_MED);
	glPushMatrix();
		glTranslatef(x, y, 0);
		glColor3fv(color_lightgray);
		glRectf(-TEXTBOX_BORDER,
				-TEXTBOX_BORDER,
				TEXT_MED * gmf[(int)'m'].gmfCellIncX * size + TEXTBOX_BORDER,
				TEXT_MED + TEXTBOX_BORDER);
		glColor3fv(color_black);
		textPrintf(copy);
	glPopMatrix();
}

void
drawTextRight(float x, float y, char* msg, int size)
{
	char copy[size+1];
	strncpy(copy, msg, size);
	copy[size] = '\0';

	textSetAlignment(ALIGN_RIGHT);
	textSetSize(TEXT_MED);
	glPushMatrix();
		glTranslatef(x, y, 0);
		glColor3fv(color_lightgray);
		glRectf(-TEXT_MED * gmf[(int)'m'].gmfCellIncX * size - TEXTBOX_BORDER,
				-TEXTBOX_BORDER,
				TEXTBOX_BORDER,
				TEXT_MED + TEXTBOX_BORDER);
		glColor3fv(color_black);
		textPrintf(copy);
	glPopMatrix();
}

/* Draws the data output from the sensors. */
void
drawOutput(outputData *data)
{
	drawGyro(GYRO_POS_X, GYRO_POS_Y, data->gyro);
	drawAccelerometer(ACC_POS_X, ACC_POS_Y, data->accelerometer);
	drawEncoders(ENC_POS_X, ENC_POS_Y, data->encoders);
	drawGripper(ROBOT_POS_X, ROBOT_POS_Y, data->gripper);
	drawRobot(ROBOT_POS_X, ROBOT_POS_Y, data->id);
	drawIRRobot(IR_ROBOT_POS_X, IR_ROBOT_POS_Y);
	drawIRBounce(IR_ROBOT_POS_X, IR_ROBOT_POS_Y, data->irData);
	drawNBRMap(NBR_ROBOT_POS_X, NBR_ROBOT_POS_Y);
	drawNBRs(NBR_ROBOT_POS_X, NBR_ROBOT_POS_Y,
		data->id, data->numNbrs, data->nbrs);
	drawLightSensors(ROBOT_POS_X, ROBOT_POS_Y, data->lightSensors, data->gripper.isOn, data->isv11);
	drawBumpSensors(ROBOT_POS_X, ROBOT_POS_Y, color_red, data->bumpSensors);
	drawButtons(BTNLED_POS_X, BTNLED_POS_Y, data->buttons);
	drawText(RADIO_POS_X, RADIOOUT_POS_Y, data->radioMsg, LEN_RADIO_MESSAGE);
}
