/*
 * test_1.c
 *
 *  Created on: Oct 11, 2014
 *      Author: Zak
 */

#include "collectivetransport.h"

extern centroidNbrData guidePosition;

state currentState = IDLE;

int32 avoidBearing = 0;
uint8 avoidActive = 0;

NbrData isActive;

/**
 * Initialization of subsystems and such
 */
void behaviorTaskInit() {
	/* Set our radio subnet */
	radioCommandSetSubnet(2);

	/* Initialize neighbor subsystem */
	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	/* Set rprintf time */
	rprintfSetSleepTime(RPRINTF_SLEEP_TIME);

	/* Enable the external pose system */
	externalPoseInit();

	/* Filter really smooth */
	neighborsSetFilterTimeConstants(50, 50);

	centroidLiteInit();

	guideInit();
	leaderInit();
}

/**
 * Main behavior task
 */
void behaviorTask(void* parameters) {
	uint32 lastWakeTime;	// The last time this task was woken

	Beh behOutput;			// Output motion behavior

	boolean buttonOldR;		// Was the button pressed last round?
	boolean buttonOldG;
	boolean buttonOldB;

	/* Initialize variables and subsystems */
	behaviorTaskInit();

	nbrDataCreate(&isActive, "isActive", 1, 0);

	for (;;) {
		lastWakeTime = osTaskGetTickCount();

		/* Disable neighbor comms and just be a radio host */
		if (rprintfIsHost() || externalPoseIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_FAST);
			neighborsDisable();
			/* Delay task until next time */
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
			continue;
		}

		if (currentState == IDLE) {
			nbrDataSet(&isActive, 0);
		} else {
			nbrDataSet(&isActive, 1);
		}

		// Buttons
		uint8 buttonR = buttonsGet(BUTTON_RED);
		if (buttonR && !buttonOldR) {
			switch (currentState) {
			case (IDLE): {
				currentState = FOLLOW;
				break;
			}
			case (LEADER): {
				currentState = IDLE;
				leaderSetLeader(FALSE);
				break;
			}
			case (GUIDE): {
				currentState = IDLE;
				guideSetGuide(FALSE);
				break;
			}
			case (FOLLOW):
			default: {
				currentState = IDLE;
				break;
			}
			}
		}
		buttonOldR = buttonR;

		uint8 buttonG = buttonsGet(BUTTON_GREEN);
		if (buttonG && !buttonOldG) {
			switch (currentState) {
			case (IDLE): {
				currentState = LEADER;
				leaderSetLeader(TRUE);
				break;
			}
			case (LEADER): {
				leaderSetActive((leaderIsActive()) ? FALSE : TRUE);
				break;
			}
			case (GUIDE): {
				uint8 hops = guideGetMyHops();

				guideSetHops((hops + 1) % (MAX_HOPS + 1));

				cprintf("%d\n", guideGetMyHops());
				break;
			}
			default: {
				currentState = IDLE;
				break;
			}
			}
		}
		buttonOldG = buttonG;

		uint8 buttonB = buttonsGet(BUTTON_BLUE);
		if (buttonB && !buttonOldB) {
			switch (currentState) {
			case (IDLE): {
				currentState = GUIDE;
				guideSetGuide(TRUE);
				break;
			}
			case (GUIDE): {
				uint8 type = guideGetType();
				switch (type) {
				case (GUIDE_S): {
					guideSetType(GUIDE_SM);
					break;
				}
				case (GUIDE_SM): {
					guideSetType(GUIDE_G);
					break;
				}
				case (GUIDE_G): {
					guideSetType(GUIDE_S);
					break;
				}
				default: {
					guideSetType(GUIDE_G);
					break;
				}
				}

				cprintf("%d\n", guideGetType());
				break;
			}
			default: {
				currentState = IDLE;
				break;
			}
			}
		}
		buttonOldB = buttonB;

		// LEDs
		switch (currentState) {
		case (IDLE): {
			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			break;
		}
		case (LEADER): {
			if (leaderIsActive()) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_FAST);
			} else {
				ledsSetPattern(LED_GREEN, LED_PATTERN_ON,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			}
			break;
		}
		case (GUIDE): {
			ledsClear(LED_ALL);

			ledsSetPatternSingle(LED_RED, LED_PATTERN_MANUAL,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			ledsSetPatternSingle(LED_GREEN, LED_PATTERN_MANUAL,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);

			uint8 hops = guideGetMyHops();

			uint8 i;
			for (i = 0; i < 16; i++) {
				ledsSetSingle(i, 0);
			}
			for (i = 1; i < hops + 1; i++) {
				ledsSetSingle(15 - i, 66);
			}

			uint8 type = guideGetType();

			switch (type) {
			case (GUIDE_S): {
				ledsSetPatternSingle(LED_BLUE, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
				break;
			}
			case (GUIDE_SM): {
				ledsSetPatternSingle(LED_BLUE, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				break;
			}
			case (GUIDE_G): {
				ledsSetPatternSingle(LED_BLUE, LED_PATTERN_ON,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
				break;
			}
			default: {
				break;
			}
			}
			break;
		}
		case (FOLLOW): {
			if (leaderIsActive()) {
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
						LED_BRIGHTNESS_LOW, LED_RATE_FAST);
			} else {
				ledsSetPattern(LED_RED, LED_PATTERN_ON,
						LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			}
			break;
		}
		default: {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE,
					LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			break;
		}
		}

		// Movement
		switch (currentState) {
		case (LEADER):
		case (FOLLOW): {
			uint8 movementMode = leaderGetMovementMode();

			switch (movementMode) {
			case (MOVEMODE_TRNS): {
				objectTranslateGuide(15, &behOutput);
				break;
			}
			case (MOVEMODE_ROTL): {
				objectRotatePerpendicularLeft(40, &behOutput);
				break;
			}
			case (MOVEMODE_ROTR): {
				objectRotatePerpendicularRight(40, &behOutput);
				break;
			}
			case (MOVEMODE_STOP):
			default: {
				behOutput = behInactive;
				break;
			}
			}
			break;
		}
		case (IDLE): {
			if (avoidActive) {
				behMoveForward(&behOutput, 30);
				rvBearingController(&behOutput, avoidBearing, 50);
			} else {
				behOutput = behInactive;
			}
			break;
		}
		case (GUIDE):
		default: {
			behOutput = behInactive;
			break;
		}
		}


		// Debug
		switch (currentState) {
		case (LEADER):
		case (FOLLOW): {
//			cprintf("pt 0,%d,%d\n", (int32) centroidLiteGetXCoordinate(), (int32) centroidLiteGetYCoordinate());
//			centroidValue gx, gy;
//			centroidNbrDataGet(&gx, &gy, &guidePosition);
//			cprintf("pt 1,%d,%d\n", (int32) gx, (int32) gy);
//			cprintf("pt 2,%d,%d\n", 50 * cosMilliRad(leaderGetObjectOrientation()) / MILLIRAD_TRIG_SCALER,
//					50 * sinMilliRad(leaderGetObjectOrientation()) / MILLIRAD_TRIG_SCALER);
//			cprintf("pt 3,%d,%d\n", 50 * cosMilliRad(leaderGetGuideObjectBearing()) / MILLIRAD_TRIG_SCALER,
//					50 * sinMilliRad(leaderGetGuideObjectBearing()) / MILLIRAD_TRIG_SCALER);
			break;
		}
		case (GUIDE): {
			break;
		}
		case (IDLE):
		default: {
			break;
		}
		}

//
		/* Set motion output */
		motorSetBeh(&behOutput);
		/* Delay task until next time */
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

/**
 * Startup
 */
int main(void) {
  	systemInit();
	behaviorSystemInit(behaviorTask, 2048);
	osTaskStartScheduler();

	return 0;
}
