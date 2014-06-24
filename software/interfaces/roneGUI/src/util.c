/*
 * util.c
 *
 * Non-OpenGL basic utility functions.
 *
 *
 *  Created on: May 31, 2011
 *      Author: nathan
 */

#include "gui.h"

/*
 * Clamp a number between min and max .
 */
float clamp(float num, float min, float max)
{
	if (num < min)
		num = min;
	else if (num > max)
		num = max;

	return num;
}

/*
 * Insert character into the string at the position. There should be room in the string.
 */
void strins(char* str, char c, int index)
{
	int i, temp1, temp2;
	temp1 = str[index];
	for (i = index+1; str[i] != '\0'; i++) {
		temp2 = str[i];
		str[i] = temp1;
		temp1 = temp2;
	}
	str[i] = temp1;
	str[i+1] = '\0';
	str[index] = c;
}

/*
 * Delete the character at index and shift everything to the left.
 */
void strdel(char* str, int index)
{
	int i;
	for (i = index; str[i] != '\0'; i++)
	{
		str[i] = str[i+1];
	}
}

/*
 * First order infinite input response filter with specified alpha, 0 < alpha < 1.
 */
void differential(int newData, int *oldDataPtr, float alpha) {
	*oldDataPtr = (int)((float)newData * alpha + (float)*oldDataPtr * (1 - alpha));
}

void differentialf(float newData, float *oldDataPtr, float alpha) {
	*oldDataPtr = (newData * alpha + *oldDataPtr * (1 - alpha));
}

void differentiala(float newData, float *oldDataPtr) {
	float centerAngle = (newData + *oldDataPtr + 360) / 2;

	if (abs(newData - *oldDataPtr) > 180)
		centerAngle -= 180;

	*oldDataPtr = (*oldDataPtr + centerAngle + 180) / 2 - 180;
}
