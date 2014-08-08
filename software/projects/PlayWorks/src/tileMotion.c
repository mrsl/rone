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
#define MOTION_STATE_PARK		1
#define MOTION_STATE_ROTATE					2
#define MOTION_STATE_ROTATE_MAG				3
#define MOTION_STATE_FORWARD		4
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

// Physical measurements of hardware in mm
#define ROBOT_RADIUS				50
#define TILE_CENTER_RADIUS			0

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


uint32 moveStart(Pose* motionPoseStart, TileInfo* tilePtr, char* motion) {
	encoderGetPose(motionPoseStart);
	magRVController(0, 0, TRUE, FALSE);
	cprintf("\ntile \"%s\": %s\n", tilePrint(tilePtr), motion);

	return encoderGetOdometer();
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
	int32 distanceTraveled;
	int32 distanceToGoal;
	int8 direction;

	tileMotionMsgQueue = osQueueCreate(1, sizeof(uint8));
	int16 tileMotionRotationGoal = 0;
	boolean online = FALSE;
	boolean forwardStageTwo = FALSE;

	while (TRUE) {
		behOutput = behInactive;
		boolean printNow = printTimer(&printTime, PRINT_TIME);

		switch (mode) {
		case MOTION_STATE_IDLE:
			// Idle state makes the robot wait and take user motion command inputs
			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			portBASE_TYPE val = osQueueReceive(tileMotionMsgQueue, (void *)(&tileMotionMsg), 0);
			if (val == pdPASS) {
				/*
				 * New message.  Start the tile track motion controller.
				 * IF we don't have a tile, just do dead reckoning.
				 * We assume that this controller starts at the center of a tile.
				 * All other motions will be relative to this initial placement.
				 * Yes, this is a terrible hack, but we'll make it better later...
				 */
				tileCurrentPtr = tileMotionReadTile();
				motionOdometerStart = moveStart(&motionPoseStart, tileCurrentPtr,  "start new motion");
				switch (tileMotionMsg) {
				case TILEMOTION_FORWARD:
					mode = MOTION_STATE_FORWARD;
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
			break;
		case MOTION_STATE_FORWARD:
			/*
			 * First state of forward translation. The robot moves in the current direction until
			 * it finds the tile or after it has traversed the length of the tile.
			 */
//			cprintf("FORWARD_TO_EDGE\n");

			/*
			 * move to the edge of the current tile.  We can't get here unless we have
			 * a valid tilePtr.  Use odometry to measure distance.
			 */
			distanceTraveled = encoderGetOdometer() - motionOdometerStart;

			// Calculate distance to goal
			if (tileCurrentPtr) {
				// TODO test rfid tilesense, 1 state operation?
				// If the robot started from a tile, use the tile information to help calculate the distance
				distanceToGoal = tileGetDistanceToCenter(tileCurrentPtr) - distanceTraveled;
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
			} else {
				// If the robot started without tile info, use the tile width of the largest tile until we get the tile ID
				distanceToGoal = (2 * TILE_WIDTH_HEX_TEST) - distanceTraveled;
				if (!forwardStageTwo) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				} else {
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				}
			}

			// Determine progress to goal
			if (distanceToGoal < 0) {
				// Goal reached
				tileCurrentPtr = NULL;
				// Store odometer and pose before switching state
				motionOdometerStart = moveStart(&motionPoseStart, tileCurrentPtr, "goal reached");
				mode = MOTION_STATE_IDLE;
				motionDone = TRUE;
			} else {
				// In progress
				behOutput.tv = computeVelRamp(MOTION_TV, MOTION_TV_MIN, MOTION_TV_RAMP_DISTANCE, distanceTraveled, distanceToGoal, TRUE, FALSE);
				// Check line sensors for robot angle adjustment
				if (reflectiveGetNumActiveSensors(&direction) > 0) {
					// Online, apply rv adjustment to center robot
					rvBearingController(&behOutput, REFLECTIVE_SENSOR_SEPARATION * direction, REFLECTIVE_TURING_SPEED);
					// Forward-to-edge stage ends when the robot reaches the border lines (white to black transition)
					if (!online && (tileCurrentPtr == NULL)) {
						motionOdometerStart = moveStart(&motionPoseStart, tileCurrentPtr, "enter new stage");
						if (!forwardStageTwo) {
							// Enter stage 2
							forwardStageTwo = TRUE;
						} else {
							// Enter stage 3
							forwardStageTwo = FALSE;
							mode = MOTION_STATE_PARK;
						}
					}
					// distance traveled when the robot gets back online
//					if (!online) {
//						distanceElapsedBetweenLines = encoderGetOdometer() - lastOnlineOdometerValue;
//					}
					online = TRUE;
				} else {
					// No line detected, no turn
					behOutput.rv = 0;
					// Save the odometer value for the position when the robot exits a line
//					if (online) {
//						lastOnlineOdometerValue = encoderGetOdometer();
//					}
					online = FALSE;
				}
				//if (printNow) cprintf(",%d", distanceToGoal);
				behOutput.active = TRUE;
			}
			break;
		case MOTION_STATE_PARK:
			/*
			 * Second state of forward translation for no tile detection. The robot finds the center of the tile using odometry and with help of line sensors.
			 */
			// If the robot started without tile info, use the tile width of the largest tile until we get the tile ID
			distanceTraveled = encoderGetOdometer() - motionOdometerStart;

			// Calculate distance to goal
			distanceToGoal = ROBOT_RADIUS + TILE_CENTER_RADIUS - distanceTraveled;

			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_TURBO);

			// Determine progress to goal
			if (distanceToGoal < 0) {
				// Goal reached
				tileCurrentPtr = NULL;
				// Store odometer and pose before switching state
				motionOdometerStart = moveStart(&motionPoseStart, tileCurrentPtr, "goal reached");
				mode = MOTION_STATE_IDLE;
				motionDone = TRUE;
			} else {
				// TODO maybe slow down tv
				// In progress
				behOutput.tv = computeVelRamp(MOTION_TV, MOTION_TV_MIN, MOTION_TV_RAMP_DISTANCE, distanceTraveled, distanceToGoal, TRUE, FALSE);
				// Check line sensors for robot angle adjustment
				if (reflectiveGetNumActiveSensors(&direction) > 0) {
					// Online, apply rv adjustment to center robot
					rvBearingController(&behOutput, REFLECTIVE_SENSOR_SEPARATION * direction, REFLECTIVE_TURING_SPEED);
					online = TRUE;
				} else {
					// No line detected, no turn
					behOutput.rv = 0;
					online = FALSE;
				}
				behOutput.active = TRUE;
			}
			break;
//		case MOTION_STATE_FORWARD_TO_CENTER: {
//			/* move to the center of the tile.  initially, you don't know what tile you are on,
//			 * so you don't know how far to go.  Read the RFID Tag to figure out how far to travel.
//			 */
//			int32 distanceToGoal;
//			int32 distanceTravelled = encoderGetOdometer() - motionOdometerStart;
//			if (tileCurrentPtr) {
//				distanceToGoal = tileGetDistanceToCenter(tileCurrentPtr) - distanceTravelled;
//				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//			} else {
//				// move forward until you detect a tile
//				tileMotionReadTile();
//				// use the distance of the smallest tile until we get the tile ID
//				distanceToGoal = TILE_WIDTH_HEX_LARGE - distanceTravelled;
//				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//			}
//			int32 tv = computeVelRamp(MOTION_TV, MOTION_TV_MIN, MOTION_TV_RAMP_DISTANCE, distanceTravelled, distanceToGoal, FALSE, TRUE);
//			if (distanceToGoal <= 0) {
//				//TODO this code stops theprogram until the robot reads a tag.  Right now, we just continue
////				if (tileCurrentPtr) {
////					mode = MOTION_STATE_IDLE;
////					motionDone = TRUE;
////				} else {
////					tv = 0;
////				}
//				mode = MOTION_STATE_IDLE;
//				motionDone = TRUE;
//			}
//			//if (printNow) cprintf(",%d", distanceTravelled);
//			//if (printNow) cprintf("%d l=% 6d r=% 6d\n", distanceTravelled, magGetMagnitudeLeft(), magGetMagnitudeRight());
//			behSetTvRv(&behOutput, tv, 0);
//			break;
//		}
		case MOTION_STATE_ROTATE:;
			Pose pose;
			encoderGetPose(&pose);
			int32 angleRotated = abs(smallestAngleDifference(motionPoseStart.theta, pose.theta));
			int32 angleToGoal = abs(tileMotionRotationGoal) - angleRotated;
			int32 rv = computeVelRamp(MOTION_RV, MOTION_RV_MIN, MOTION_RV_RAMP_DISTANCE, angleRotated, angleToGoal, TRUE, TRUE);
			if (angleRotated >= abs(tileMotionRotationGoal)) {
				// Done rotating
				mode = MOTION_STATE_IDLE;
				motionDone = TRUE;
			} else {
				// Keep rotating
				if (tileMotionRotationGoal > 0) {
					// Rotate left
					behSetTvRv(&behOutput, 0, rv);
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				} else {
					// Rotate right
					behSetTvRv(&behOutput, 0, -rv);
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				}
			}
			break;
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
