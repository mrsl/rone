/*
 * @file behaviorSystem.c
 *
 * @brief behaviors associated with moving
 * @since Sep 11, 2011
 * @author jmclurkin
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

// how much we modify tv each time before it ramps up to speed
#define BEH_RAMPING_DELTA				50

const Beh behInactive = {0, 0, FALSE};

//void behaviorTask(void* parameters) {
void behaviorSystemInit(void (*behaviorTask)(void* parameters), uint32 stackSize) {
	osTaskCreate(behaviorTask, "behavior", stackSize, NULL, BEHAVIOR_TASK_PRIORITY);
}


/*
 *  @brief Moves the robot based on the input motor behavior.
 *
 *  If the input behavior is inactive, the motors are not turned on.
 *  @param behPtr for desired motor behavior
 *  @returns void
 */
void motorSetBeh(Beh* behPtr) {
	static int32 tvOld = 0;
	int32 tv;

	if (behPtr->tv < (tvOld - BEH_RAMPING_DELTA)) {
		// too big a ramp for one step.  Slow down slowly to prevent tipping
		tv = tvOld - BEH_RAMPING_DELTA;
	} else {
		tv = behPtr->tv;
	}
	tvOld = tv;

	if (behPtr->active) {
		motorSetTVRV(tv, behPtr->rv);
	} else {
		motorSetTVRV(0, 0);
	}
}


/*
 * @brief Gets translational velocity
 *
 * @param behPtr for behavior
 * @returns translational velocity of input behavior
 */
uint8 behGetTv(Beh* behPtr) {
	return behPtr->tv;
}


/*
 *  @brief Sets the input behavior's translational and rotational velocities
 *
 *  @param behPtr for behavior to update
 *  @param tv desired translational velocity
 *  @param rv desired rotational velocity
 *  @returns void
 */
void behSetTvRv(Beh* behPtr, int32 tv, int32 rv) {
	behPtr->tv = tv;
	behPtr->rv = rv;
	behPtr->active = TRUE;
}


/*
 *  @brief Sets the input behavior's translational velocity
 *
 *  @param behPtr for behavior to update
 *  @param tv desired translational velocity
 *  @returns void
 */
void behSetTv(Beh* behPtr, int32 tv) {
	behPtr->tv = tv;
	behPtr->active = TRUE;
}


/*
 *  @brief Sets the input behavior's rotational velocity
 *
 *  @param behPtr for behavior to update
 *  @param rv desired rotational velocity
 *  @returns void
 */
void behSetRv(Beh* behPtr, int32 rv) {
	behPtr->rv = rv;
	behPtr->active = TRUE;
}


/*
 *  @brief Gets the input behavior's rotational velocity.
 *
 *  @param behPtr for behavior
 *  @returns the rotational velocity of the input behavior
 */
uint8 behGetRv(Beh* behPtr) {
	return behPtr->rv;
}

uint8 behIsActive(Beh* behPtr) {
	return behPtr->active;
}


/*
 *  @brief Sets the input behavior to active
 *
 *  @param behPtr for behavior
 *  @returns void
 */
void behSetActive(Beh* behPtr) {
	behPtr->active = TRUE;
}

/*
 *  @brief Sets the input behavior to inactive
 *
 *  @param behPtr for behavior
 *  @returns void
 */
void behSetInactive(Beh* behPtr) {
	behPtr->active = FALSE;
}


Beh* behSubsume(Beh* behOutPtr, Beh* behInLoPtr, Beh* behInHighPtr) {
	if (behIsActive(behInHighPtr)) {
		*behOutPtr = *behInHighPtr;
	} else if (behIsActive(behInLoPtr)){
		*behOutPtr = *behInLoPtr;
	}
	return behOutPtr;
}

Beh* behSubsume2(Beh* behOutPtr, Beh* behIn1Ptr, Beh* behIn2Ptr, Beh* behIn3Ptr) {
	if (behIsActive(behIn3Ptr)) {
		*behOutPtr = *behIn3Ptr;
	} else if (behIsActive(behIn2Ptr)){
		*behOutPtr = *behIn2Ptr;
	} else if (behIsActive(behIn1Ptr)){
		*behOutPtr = *behIn1Ptr;
	}
	return behOutPtr;
}


/*
 *  @brief Determines if the red, green, or blue button has been pressed.
 *
 *  Updates the input modePtr to indicate which color has been pressed.
 *  @param modePtr for the mode to update if a button has been pressed
 *  @param modeRed the mode that should be set if the red button has been pressed
 *  @param modeGreen the mode that should be set if the green button has been pressed
 *  @param modeBlue the mode that should be set if the blue button has been pressed
 *  @returns a boolean TRUE if a button has been pressed (and therefore modePtr updated) and FALSE if not
 */
boolean buttonModeRGB(uint8* modePtr, uint8 modeRed, uint8 modeGreen, uint8 modeBlue) {
	boolean press = FALSE;
	if (buttonsGet(BUTTON_RED)) {
		*modePtr = modeRed;
		press = TRUE;
	}
	if (buttonsGet(BUTTON_GREEN)) {
		*modePtr = modeGreen;
		press = TRUE;
	}
	if (buttonsGet(BUTTON_BLUE)) {
		*modePtr = modeBlue;
		press = TRUE;
	}
	return press;
}
