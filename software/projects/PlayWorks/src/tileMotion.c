/*
 * tileMotion.c
 *
 *  Created on: Mar 31, 2014
 *      Author: jamesm
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"

// Tile to tile forward movement state
#define MOTION_STATE_IDLE					0
#define MOTION_STATE_FORWARD_TO_CENTER		1
#define MOTION_STATE_ROTATE					2
#define MOTION_STATE_ROTATE_MAG				3
#define MOTION_STATE_FORWARD_TO_EDGE		4
#define MOTION_STATE_TEST_MAG_CENTER		5

#define MOTION_TV					40
#define MOTION_TV_RAMP_DISTANCE		11
#define MOTION_TV_MIN				10

#define MOTION_RV					(MILLIRAD_PI / 4)
#define MOTION_RV_RAMP_DISTANCE		(MILLIRAD_PI / 25)
#define MOTION_RV_MIN				100

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

// Tile track controller gains
#define MAG_ROTATE_KP				200
#define MAG_ROTATE_KI				10
#define MAG_TRANSLATE_KP			MAG_ROTATE_KP
#define MAG_TRANSLATE_KI			MAG_ROTATE_KI
#define MAGNETOMETER_RV_THRESHOLD	100

#define MAG_ROTATE_TA_KP				15
#define MAG_ROTATE_TA_KI				5

#define ITERM_MAX					1200

#define PRINT_TIME	200


static uint32 motionOdometerStart;
static Pose motionPoseStart;
static osQueueHandle tileMotionMsgQueue;
static uint8 mode = MOTION_STATE_IDLE;
static TileInfo* tileCurrentPtr = NULL;
static boolean motionDone = TRUE;

void tileMotion(uint8 motionCommand) {
	motionDone = FALSE;
	osQueueSend(tileMotionMsgQueue, (void*)(&motionCommand), 0);
}


boolean tileMotionDone(void) {
	return motionDone;
}


TileInfo* tileMotionGetTile(void) {
	return 	tileCurrentPtr;
}


void tileMotionWaitUntilDone(void) {
	while(!motionDone) {
		osTaskDelay(10);
	}
}



int32 computeVelRamp(int32 velMax, int32 velMin, int32 rampDistance, int32 distance, int32 distanceToGoal, boolean rampUp,  boolean rampDown) {
	int32 tv = velMax;
	if (rampDown && (distanceToGoal < rampDistance)) {
		tv = velMax * distanceToGoal / rampDistance;
		tv = bound(tv, velMin, velMax);
	}
	if (rampUp && (distance < rampDistance)) {
		tv = velMax * distance / rampDistance;
		tv = bound(tv, velMin, velMax);
	}
	return tv;
}


int32 magRVController(int32 kp, int32 ki, boolean reset, boolean printNow) {
	static int32 iTerm;
	int32 magLeft = magGetMagnitudeLeft();
	int32 magRight = magGetMagnitudeRight();
	int32 e = magLeft - magRight;
	int32 pTerm = e * kp / 100;
	if (reset) {
		iTerm = 0;
	} else {
		iTerm += e * ki / 1000;
		iTerm = boundAbs(iTerm, ITERM_MAX);
	}
	int32 rv = -(pTerm + iTerm);
	if (printNow) cprintf("L% 6d, R% 6d, e% 6d, P% 6d, I% 6d, rv% 6d\n", magLeft, magRight, e, pTerm, iTerm, rv);
	return rv;
}


#define TA_CONTROLLER_Z_THRESHOLD		150
#define TA_CONTROLLER_MAG_THRESHOLD		600
#define TA_CONTROLLER_RV_MAX			0

int32 magThreeAxisController(int32 kp, int32 ki, boolean reset, boolean printNow) {
	int32 x, y, z, mag;
	static int32 iTerm;
	int32 rv = 0;

	x = magGetValueRight(MAG_X_AXIS);
	y = magGetValueRight(MAG_Y_AXIS);
	z = magGetValueRight(MAG_Z_AXIS);
	mag = magGetMagnitudeRight();

	if (mag > TA_CONTROLLER_MAG_THRESHOLD) {
		// you are in the sweet spot in the basin of the x-axis. Run a p-controller
		int32 e = y;
		int32 pTerm = e * kp / 100;
		if (reset) {
			iTerm = 0;
		} else {
			iTerm += e * ki / 1000;
			iTerm = boundAbs(iTerm, ITERM_MAX);
		}
		rv = -(pTerm + iTerm);
		if (printNow) cprintf("near: x% 6d, y% 6d, z% 6d,  mag% 6d, e% 6d, P% 6d, I% 6d, rv% 6d\n", x, y, z, mag, e, pTerm, iTerm, rv);
	} else {
		// you are far from the center.  move towards the basin with constant velocity
		if (y > 0) {
			rv = TA_CONTROLLER_RV_MAX;
		} else {
			rv = -TA_CONTROLLER_RV_MAX;
		}
		iTerm = 0;
		if (printNow) cprintf(" far: x% 6d, y% 6d, z% 6d, mag% 6d, rv% 6d\n", x, y, z, mag, rv);
	}
	rv = 0;
	return rv;
}


void moveStart(TileInfo* tilePtr, char* motion) {
	motionOdometerStart = encoderGetOdometer();
	encoderGetPose(&motionPoseStart);
	magRVController(0, 0, TRUE, FALSE);
	cprintf("\ntile \"%s\": %s\n", tilePrint(tilePtr), motion);
}


TileInfo* tileMotionReadTile (void) {
	static TileInfo* tileOldPtr = NULL;

	TileInfo* tileTempPtr = tileReadSensor();
	// Detected new tile and it's not the last tile
	if ((tileTempPtr) && (tileTempPtr != tileOldPtr)) {
		tileCurrentPtr = tileTempPtr;
		tileOldPtr = tileCurrentPtr;
		playTileSound(tileGetID(tileCurrentPtr));
		cprintf("\nTP,tag,%d\n", tileGetID(tileCurrentPtr));
	}
	return tileCurrentPtr;
}


void tileMotionTask(void* parameters) {
	uint8 tileMotionMsg;
	uint32 lastWakeTime = osTaskGetTickCount();
	Beh behOutput;
	uint32 printTime = 0;

	tileMotionMsgQueue = osQueueCreate(1, sizeof(uint8));
	int16 tileMotionRotationGoal = 0;

	while (TRUE) {
		behOutput = behInactive;
		boolean printNow = printTimer(&printTime, PRINT_TIME);

		switch (mode) {
//		case MOTION_STATE_TEST_MAG_CENTER: {
//			if (printNow) cprintf("tileID=% 3d ", bumpSensorsGetBits());
//			//int32 rv = magRVController(MAG_TRANSLATE_KP, MAG_TRANSLATE_KI, FALSE, printNow);
//			int32 rv = magThreeAxisController(MAG_ROTATE_TA_KP, MAG_ROTATE_TA_KI, FALSE, printNow);
//			behSetTvRv(&behOutput, 0, rv);
//			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//			break;
//		}
		case MOTION_STATE_IDLE: {
			cprintf("IDLE\n");

			// Idle state waits and takes user motion command inputs
			// TODO change light pattern
			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			portBASE_TYPE val = osQueueReceive(tileMotionMsgQueue, (void *)(&tileMotionMsg), 0);
			if (val == pdPASS) {
				/* New message.  Start the tile track motion controller.
				 * IF we don't have a tile, just do dead reckoning.
				 * We assume that this controller starts at the center of a tile.
				 * All other motions will be relative to this initial placement.
				 * Yes, this is a terrible hack, but we'll make it better later...
				 */
				tileCurrentPtr = tileMotionReadTile();
				moveStart(tileCurrentPtr, "start, align");
				switch (tileMotionMsg) {
				case TILEMOTION_FORWARD:
					mode = MOTION_STATE_FORWARD_TO_EDGE;
					break;
				case TILEMOTION_ROTATE_RIGHT:
					// compute the number of degrees to rotate from the tile geometry
					if (tileCurrentPtr) {
						tileMotionRotationGoal = -tileGetGeometryInfo(tileCurrentPtr)->degreesOfRotation;
					} else {
						// assume the angle of a hex tile
						tileMotionRotationGoal = -MILLIRAD_DEG_60;
					}
					mode = MOTION_STATE_ROTATE;
					break;
				case TILEMOTION_ROTATE_LEFT:
					// compute the number of degrees to rotate from the tile geometry
					if (tileCurrentPtr) {
						tileMotionRotationGoal = tileGetGeometryInfo(tileCurrentPtr)->degreesOfRotation;
					} else {
						// assume the angle of a hex tile
						tileMotionRotationGoal = MILLIRAD_DEG_60;
					}
					mode = MOTION_STATE_ROTATE;
					break;
				default:
					break;
				}
			}
			/* Check of the button is pressed (low to high transition) */
//			if (buttonsGetEdge(BUTTON_GREEN)) {
//				mode = MOTION_STATE_TEST_MAG_CENTER;
//			}
			break;
		}
		case MOTION_STATE_FORWARD_TO_EDGE: {
			cprintf("FORWARD_TO_EDGE\n");
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);

			/* move to the edge of the current tile.  We can't get here unless we have
			 * a valid tilePtr.  Use odometry to measure distance,
			 * and use the magnetometer to help center the robot.
			 */
			int32 distanceTraveled = encoderGetOdometer() - motionOdometerStart;
			int32 distanceToGoal;
			uint32 tv, rv;
			int8 direction;

			// TODO new way to calculate distance to goal using line sensor
			// Calculate distance to goal
			if (tileCurrentPtr) {
				distanceToGoal = tileGetDistanceToCenter(tileCurrentPtr) - distanceTraveled;
//				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
			} else {
				// use the distance of the smallest tile until we get the tile ID
				distanceToGoal = TILE_WIDTH_HEX_LARGE - distanceTraveled;
//				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
			}

			if (distanceToGoal < 0) {
				tileCurrentPtr = NULL;
				moveStart(tileCurrentPtr, "move to center");
				//				mode = MOTION_STATE_FORWARD_TO_CENTER;
				mode = MOTION_STATE_IDLE;
				motionDone = TRUE;
			} else {
				behOutput.tv = computeVelRamp(MOTION_TV, MOTION_TV_MIN, MOTION_TV_RAMP_DISTANCE, distanceTraveled, distanceToGoal, TRUE, FALSE);
				if (reflectiveGetNumActiveSensors(&direction) > 0) {
					// Online, apply rv adjustment to center robot
					// TODO tweak gain
					rvBearingController(&behOutput, REFLECTIVE_SENSOR_SEPARATION * direction, 50);
				} else {
					// No line detected
					behOutput.rv = 0;
				}
				//if (printNow) cprintf(",%d", distanceToGoal);
				behOutput.active = TRUE;
			}
			break;
		}
		case MOTION_STATE_FORWARD_TO_CENTER: {
			/* move to the center of the tile.  initially, you don't know what tile you are on,
			 * so you don't know how far to go.  Read the RFID Tag to figure out how far to travel.
			 */
			int32 distanceToGoal;
			int32 distanceTravelled = encoderGetOdometer() - motionOdometerStart;
			if (tileCurrentPtr) {
				distanceToGoal = tileGetDistanceToCenter(tileCurrentPtr) - distanceTravelled;
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
			} else {
				// move forward until you detect a tile
				tileMotionReadTile();
				// use the distance of the smallest tile until we get the tile ID
				distanceToGoal = TILE_WIDTH_HEX_LARGE - distanceTravelled;
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
			}
			int32 tv = computeVelRamp(MOTION_TV, MOTION_TV_MIN, MOTION_TV_RAMP_DISTANCE, distanceTravelled, distanceToGoal, FALSE, TRUE);
			if (distanceToGoal <= 0) {
				//TODO this code stops theprogram until the robot reads a tag.  Right now, we just continue
//				if (tileCurrentPtr) {
//					mode = MOTION_STATE_IDLE;
//					motionDone = TRUE;
//				} else {
//					tv = 0;
//				}
				mode = MOTION_STATE_IDLE;
				motionDone = TRUE;
			}
			//if (printNow) cprintf(",%d", distanceTravelled);
			//if (printNow) cprintf("%d l=% 6d r=% 6d\n", distanceTravelled, magGetMagnitudeLeft(), magGetMagnitudeRight());
			behSetTvRv(&behOutput, tv, 0);
			break;
		}
		case MOTION_STATE_ROTATE: {
			cprintf("ROTATE\n");
			Pose pose;
			encoderGetPose(&pose);
			int32 angleRotated = abs(smallestAngleDifference(motionPoseStart.theta, pose.theta));
			int32 angleToGoal = abs(tileMotionRotationGoal) - angleRotated;
			int32 rv = computeVelRamp(MOTION_RV, MOTION_RV_MIN, MOTION_RV_RAMP_DISTANCE, angleRotated, angleToGoal, TRUE, TRUE);
			if (angleRotated >= abs(tileMotionRotationGoal)) {
//				// use the magnetometer to center the robot on the tile.
//				moveStart(tileCurrentPtr, "align with magnotometer");
//				mode = TILEMOTION_MODE_MOVE_ROTATE_MAG;
				mode = MOTION_STATE_IDLE;
				motionDone = TRUE;
			} else {
				//if (printNow) cprintf("%d l=% 6d r=% 6d\n", angle, magGetMagnitudeLeft(), magGetMagnitudeRight());
				if (tileMotionRotationGoal > 0) {
					behSetTvRv(&behOutput, 0, rv);
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				} else {
					behSetTvRv(&behOutput, 0, -rv);
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				}
			}
			break;
		}
//		case MOTION_STATE_ROTATE_MAG: {
//			int32 rv = magRVController(MAG_ROTATE_KP, MAG_ROTATE_KI, FALSE, printNow);
//			if (abs(rv) < MAGNETOMETER_RV_THRESHOLD) {
//				//TODO also use the magnetometer to center the robot on the tile.
//				moveStart(tileCurrentPtr, "move to edge");
//				mode = MOTION_STATE_FORWARD_TO_EDGE;
//			} else {
//				behSetTvRv(&behOutput, 0, rv);
//			}
//			ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//			break;
//		}
		}

		// Apply motion
		motorSetBeh(&behOutput);

		osTaskDelayUntil(&lastWakeTime, 10);
	}
}


void tileMotionInit(void) {
    //init the tile motion task
	osTaskCreate(tileMotionTask, "tileMotion", 2048, NULL, BEHAVIOR_TASK_PRIORITY);
}
