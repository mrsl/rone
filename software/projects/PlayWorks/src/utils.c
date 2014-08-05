/*
 * utils.c
 *
 *  Created on: Mar 24, 2014
 *      Author: jamesm
 */

#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"


boolean buttonsGetEdge(uint8 button) {
	static uint8 buttonState[BUTTONS_NUM];
	boolean returnVal = FALSE;

	if (button < BUTTONS_NUM) {
		boolean buttonVal = buttonsGet(button);
		if ((buttonState[button] == FALSE) && (buttonVal == TRUE)) {
			returnVal = TRUE;
		}
		buttonState[button] = buttonVal;
	}
	return returnVal;
}


boolean printTimer(uint32* printTimePtr, uint32 printTime) {
	boolean printNow = FALSE;
	if ((osTaskGetTickCount() - *printTimePtr) > printTime) {
		printTime += printTime;
		printNow = TRUE;
	}
	return printNow;
}
