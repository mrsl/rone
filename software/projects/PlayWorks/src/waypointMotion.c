#if 0
/*
 * debug_navbeacon.c
 *
 *     Created on: 2012-11-05
 *         Author: jmclurkin
 *        Summary: This code is to test detection, orientation, and navigation to the nav towers
 * Current Status: working {unknown, working, defunct, inProgress}
 */
#include "roneos.h"
#include "ronelib.h"
#include <stdlib.h>
#include <math.h>

#define MODE_PROGRAM				0
#define MODE_MOVE					1
#define MODE_SENSE					2
#define MODE_LINE_FOLLOW			3

#define MOTION_TV					60
#define MOTION_TV_RAMP_DISTANCE		11
#define MOTION_TV_MIN				10

#define MOTION_RV					(MILLIRAD_PI / 3)
#define MOTION_RV_RAMP_DISTANCE		(MILLIRAD_PI / 25)
#define MOTION_RV_MIN				100


#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50

#define WAYPOINTS_MAX				20
#define COMMAND_DONE				0
#define COMMAND_FORWARD				1
#define COMMAND_LEFT				2
#define COMMAND_RIGHT				3
#define COMMAND_EXECUTE_TIME		2000
#define BUTTON_PRESS_TIME			300

#define PRINT_TIME	100

#define ROTATE_STRAIGHT				0
#define ROTATE_RIGHT				1
#define ROTATE_LEFT					2
#define RV_RATE						80
#define RV_GAIN						800
#define RV_MIN						100
#define RV_MAX						1000
#define SENSOR_MAX_DIFF				60
#define RV_MAX_DIFF					60
#define TV_MAX_DIFF					1
#define RV_DIRECTION_CHANGE_TIME	1000
#define RVTV_GAIN					50

/******** user code ********/

boolean buttonsGetEdge(uint8 button) {
	//static uint8 buttonState[BUTTONS_NUM];
	static uint8 buttonState[3];
	boolean returnVal = FALSE;

	//if (button < BUTTONS_NUM) {
	if (button < 3) {
		boolean buttonVal = buttonsGet(button);
		if ((buttonState[button] == FALSE) && (buttonVal == TRUE)) {
			returnVal = TRUE;
		}
		buttonState[button] = buttonVal;
	}
	return returnVal;
}

#define TILE_WIDTH		103

uint8 waypointList[WAYPOINTS_MAX];
uint8 waypointListWriteIdx = 0;
uint8 waypointListReadIdx = 0;

uint32 motionOdometerStart;
Pose motionPoseStart;
uint8 currentCommand = COMMAND_DONE;

Pose poseForward = {103, 0, 0};
Pose poseRight = {0, 0, MILLIRAD_DEG_90};
Pose poseLeft = {0, 0, -MILLIRAD_DEG_90};


void commandPrint(uint8 command) {
	switch (command) {
	case COMMAND_FORWARD:
		cprintf("forward");
		break;
	case COMMAND_LEFT:
		cprintf("left");
		break;
	case COMMAND_RIGHT:
		cprintf("right");
		break;
	case COMMAND_DONE:
		cprintf("done");
		break;
	}
}


void waypointListAdd(uint8 command) {
	if (waypointListWriteIdx < WAYPOINTS_MAX) {
		waypointList[waypointListWriteIdx] = command;
		waypointListWriteIdx++;
		cprintf("prog %d. ", waypointListWriteIdx);
		commandPrint(command);
		cprintf("\n");
	} else {
		cprintf("list full\n");
	}
}

int32 computeVelRamp(int32 velMax, int32 velMin, int32 rampDistance, int32 distance, int32 distanceToGoal) {
	int32 tv = velMax;
	if (distanceToGoal < rampDistance) {
		tv = velMax * distanceToGoal / rampDistance;
		tv = bound(tv, velMin, velMax);
	}
	if (distance < rampDistance) {
		tv = velMax * distance / rampDistance;
		tv = bound(tv, velMin, velMax);
	}
	return tv;
}

uint8 waypointReadCommand(void) {
	uint8 command = COMMAND_DONE;
	motionOdometerStart = encoderGetOdometer();
	encoderGetPose(&motionPoseStart);
	if (waypointListReadIdx < waypointListWriteIdx) {
		command = waypointList[waypointListReadIdx];
		waypointListReadIdx++;
		cprintf("run %d: ",waypointListReadIdx);
		commandPrint(command);
		cprintf(" d=");
	}
	return command;
}



// behaviors run every 50ms.  They should be designed to be short, and terminate quickly.
// they are used for robot control.  Watch this space for a simple behavior abstraction
// to appear.
void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	Beh behOutput;
	uint32 neighborRound = 0;
	boolean printNow;
	uint8 mode = MODE_SENSE;
	char string[18];
	static uint32 printTime = 0;
	uint32 buttonTime = 0;
	int32 i;
	uint8 tileIDOld;

    neighborsInit(300);

    // wait for a sec to get initial magnetometer values
    osTaskDelay(500);
    magSetOffset();

    // init the sound task
    playworksSoundInit();
	tileIDOld = bumpSensorsGetBits();

	while (TRUE) {
		behOutput = behInactive;
		neighborsGetMutex();

		if ((osTaskGetTickCount() - printTime) > PRINT_TIME) {
			printTime += PRINT_TIME;
			printNow = TRUE;
		} else {
			printNow = FALSE;
		}

		switch (mode) {
		case MODE_PROGRAM: {
			//ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			if (buttonsGetEdge(BUTTON_RED)) {
				// add a left turn
				waypointListAdd(COMMAND_LEFT);
				ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				buttonTime = osTaskGetTickCount();
			}
			if (buttonsGetEdge(BUTTON_GREEN)) {
				// add a forward motion
				waypointListAdd(COMMAND_FORWARD);
				ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				buttonTime = osTaskGetTickCount();
			}
			if (buttonsGetEdge(BUTTON_BLUE)) {
				// add a right turn
				waypointListAdd(COMMAND_RIGHT);
				ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				buttonTime = osTaskGetTickCount();
			}
			if ((osTaskGetTickCount() - buttonTime) > BUTTON_PRESS_TIME) {
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			}

			if (((osTaskGetTickCount() - buttonTime) > COMMAND_EXECUTE_TIME) &&
					(waypointListWriteIdx > 0)) {
				waypointListReadIdx = 0;
				mode = MODE_MOVE;
				currentCommand = waypointReadCommand();
			}
			break;
		}
		case MODE_MOVE: {
			// read the waypoint list and move to the next waypoint
			switch (currentCommand) {
			case COMMAND_FORWARD: {
				uint32 distance = encoderGetOdometer() - motionOdometerStart;
				uint32 distanceToGoal = TILE_WIDTH - distance;
				int32 tv = computeVelRamp(MOTION_TV, MOTION_TV_MIN, MOTION_TV_RAMP_DISTANCE, distance, distanceToGoal);
				if (distance > TILE_WIDTH) {
					cprintf(",done\n");
					currentCommand = waypointReadCommand();
				} else {
					if (printNow) cprintf(",%d", distance);
					behSetTvRv(&behOutput, tv, 0);
				}
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				break;
			}
			case COMMAND_LEFT: {
				Pose pose;
				encoderGetPose(&pose);
				int32 angle = abs(smallestAngleDifference(motionPoseStart.theta, pose.theta));
				int32 angleToGoal = MILLIRAD_HALF_PI - angle;
				int32 rv = computeVelRamp(MOTION_RV, MOTION_RV_MIN, MOTION_RV_RAMP_DISTANCE, angle, angleToGoal);
				if (angle > MILLIRAD_HALF_PI) {
					cprintf(",done\n");
					currentCommand = waypointReadCommand();
				} else {
					if (printNow) cprintf(",%d", angle);
					behSetTvRv(&behOutput, 0, rv);
				}
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				break;
			}
			case COMMAND_RIGHT: {
				Pose pose;
				encoderGetPose(&pose);
				int32 angle = abs(smallestAngleDifference(motionPoseStart.theta, pose.theta));
				int32 angleToGoal = MILLIRAD_HALF_PI - angle;
				int32 rv = computeVelRamp(MOTION_RV, MOTION_RV_MIN, MOTION_RV_RAMP_DISTANCE, angle, angleToGoal);
				if (angle > MILLIRAD_HALF_PI) {
					cprintf(",done\n");
					currentCommand = waypointReadCommand();
				} else {
					if (printNow) cprintf(",%d", angle);
					behSetTvRv(&behOutput, 0, -rv);
				}
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				break;
			}
			case COMMAND_DONE: {
				cprintf(",done\n");
				waypointListWriteIdx = 0;
				waypointListReadIdx = 0;
				mode = MODE_PROGRAM;
			}
			default:
				break;
			}
			break;
		}
		default:
		case MODE_SENSE:
		case MODE_LINE_FOLLOW: {
			static int32 magMagFilteredLeft = 0;
			static int32 magMagFilteredRight = 0;
			static uint8 turningDirection = ROTATE_RIGHT;
			static int32 rvLineFollow = 0;
			static int32 tvLineFollow = 0;
			static uint32 rvDirectionChangeTime = 0;

			char rvDirChange[] = "   ";

		    if (buttonsGet(BUTTON_RED)) {
		    	magSetOffset();
			}
		    if (buttonsGet(BUTTON_GREEN)) {
		    	mode = MODE_LINE_FOLLOW;
			}

		    int32 magValues[MAG_AXES];
		    int32 x, y, z;
		    for (i = 0; i < MAG_AXES; i++) {
		    	magValues[i] = magGetValueLeftRaw(i) - magOffsetValuesLeft[i];
			}
		    x = magValues[MAG_X_AXIS];
		    y = magValues[MAG_X_AXIS];
		    z = magValues[MAG_X_AXIS];

		    int32 magMagLeft = sqrtInt(x*x + y*y + z*z);
		    magMagLeft = magMagLeft * 100 / 570;
		    int32 magMagFilteredPrevLeft = magMagFilteredLeft;
		    magMagFilteredLeft = filterIIR(magMagFilteredLeft, magMagLeft, MAG_IIR_TIME_CONSTANT);

		    for (i = 0; i < MAG_AXES; i++) {
		    	magValues[i] = magGetValueRightRaw(i) - magOffsetValuesRight[i];
			}
		    x = magValues[MAG_X_AXIS];
		    y = magValues[MAG_X_AXIS];
		    z = magValues[MAG_X_AXIS];

		    int32 magMagRight = sqrtInt(x*x + y*y + z*z);
		    magMagRight = magMagRight * 100 / 7565;
		    int32 magMagFilteredPrevRight = magMagFilteredRight;
		    magMagFilteredRight = filterIIR(magMagFilteredRight, magMagRight, MAG_IIR_TIME_CONSTANT);


		    //if (printNow) cprintf("mag,% 6d,% 6d,% 6d,% 6d,% 6d\n", x, y, z, magMag, magMagFiltered);
			//if (printNow) cprintf("left % 6d,% 6d right % 6d,% 6d\n", magMagLeft, magMagFilteredLeft, magMagRight, magMagFilteredRight);
		    uint8 tileID = bumpSensorsGetBits();
			if (tileIDOld != tileID) {
				playTileSound(tileID);
				tileIDOld = tileID;
			}
			if (printNow) cprintf("tile % 4d, left % 6d right % 6d\n", tileID, magMagFilteredLeft, magMagFilteredRight);

			if (mode == MODE_LINE_FOLLOW) {
				int32 diff = magMagFilteredLeft - magMagFilteredRight;
//				int32 rvDiff = rvLineFollowNew - rvLineFollow;
//				int32 tvLineFollowNew = 0;
//				if (rvDiff > RV_MAX_DIFF) {
//					rvLineFollow += RV_MAX_DIFF;
//					tvLineFollowNew = MOTION_TV/6;
//					ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//				}else if (rvDiff < -RV_MAX_DIFF) {
//					rvLineFollow -= RV_MAX_DIFF;
//					tvLineFollowNew = MOTION_TV/6;
//					ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//				}else {
//					rvLineFollow = rvLineFollowNew;
//					tvLineFollowNew = MOTION_TV/2;
//					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//				}

				int32 rvLineFollowNew = diff * RV_GAIN / 100;
				int32 tvLineFollowNew = 0;
				if (rvLineFollowNew > RV_MAX) {
					//rvLineFollow += RV_MAX_DIFF;
					//rvLineFollow = RV_MAX;
					tvLineFollowNew = MOTION_TV/3;
					ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				}else if (rvLineFollowNew < -RV_MAX) {
					//rvLineFollow -= RV_MAX_DIFF;
					//rvLineFollow = -RV_MAX;
					tvLineFollowNew = MOTION_TV/3;
					ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				} else {
					//rvLineFollow = rvLineFollowNew;
					tvLineFollowNew = MOTION_TV;
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				}
				rvLineFollow = filterIIR(rvLineFollow, rvLineFollowNew, 50);


				tvLineFollowNew = MOTION_TV - MOTION_TV * abs(rvLineFollowNew) / RV_MAX;
				if ((tvLineFollowNew - tvLineFollow) > TV_MAX_DIFF) {
					tvLineFollow += TV_MAX_DIFF;
				}else if ((tvLineFollowNew - tvLineFollow) < -TV_MAX_DIFF) {
					tvLineFollow -= TV_MAX_DIFF;
				} else {
					tvLineFollow = tvLineFollowNew;
				}
				behSetTvRv(&behOutput, tvLineFollowNew, rvLineFollow);
				if (printNow) cprintf("left % 6d | right % 6d | rv % 6d | tv % 6d\n", magMagFilteredLeft, magMagFilteredRight, rvLineFollow, tvLineFollow);
			} else {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
			}
			break;
		}
		}

		//behSetTvRv(&behOutput, -200, 0);

		motorSetBeh(&behOutput);
		neighborsPutMutex();

		osTaskDelayUntil(&lastWakeTime, 20);
	}
}


int main(void) {
	// init the rone hardware and roneos services
	systemInit();
	systemPrintStartup();
	// init the behavior system and start the behavior thread

	behaviorSystemInit(behaviorTask, 2048);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
#endif

