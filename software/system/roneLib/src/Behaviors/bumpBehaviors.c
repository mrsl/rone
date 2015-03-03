/*
 * @file bumpBehaviors.c
 *
 * @since Sep 11, 2011
 * @author jamesm
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"


#define BUMPMOVE_IDLE					0
#define BUMPMOVE_REVERSE				1
#define BUMPMOVE_OBS_FRONT				2
#define BUMPMOVE_OBS_FRONT_LEFT			3
#define BUMPMOVE_OBS_LEFT				4
#define BUMPMOVE_OBS_FRONT_RIGHT		5
#define BUMPMOVE_OBS_RIGHT				6
#define BUMPMOVE_REFLECT_ROTATE			7
#define BUMPMOVE_MOVE_FORWARD			8
#define BUMPMOVE_ROTATE					9

#define BUMPTYPE_NONE					19
#define BUMPTYPE_OBS_FRONT				20
#define BUMPTYPE_OBS_FRONT_LEFT			21
#define BUMPTYPE_OBS_LEFT				22
#define BUMPTYPE_OBS_FRONT_RIGHT		23
#define BUMPTYPE_OBS_RIGHT				24

#define MOVEOUT_IDLE					0
#define MOVEOUT_FRONT_HIT				1
#define MOVEOUT_LEFT_HIT				2
#define MOVEOUT_BACK_HIT				3
#define MOVEOUT_RIGHT_HIT				4

#define OBSTACLE_AVOIDANCE_STATE_IDLE   	0
#define OBSTACLE_AVOIDANCE_STATE_BACKUP 	1
#define OBSTACLE_AVOIDANCE_STATE_ROTATE 	2
#define OBSTACLE_AVOIDANCE_STATE_FORWARD	3


#define IROBSTACLE_MOVE_OBS_FRONT_ANGLE		(MILLIRAD_PI * 4/6)
#define IROBSTACLE_MOVE_OBS_FRONT_X_ANGLE	(MILLIRAD_PI * 3/6)
#define IROBSTACLE_MOVE_OBS_X_ANGLE			(MILLIRAD_PI * 2/6)


#define BUMPMOVE_OBS_FRONT_ANGLE			(MILLIRAD_PI / 4)
#define BUMPMOVE_OBS_FRONT_X_ANGLE			(BUMPMOVE_OBS_FRONT_ANGLE / 2)
#define BUMPMOVE_OBS_X_ANGLE				(BUMPMOVE_OBS_FRONT_ANGLE / 4)

#define DEMO_TV								100
#define BUMPMOVE_BACKWARD_TV				100
#define BUMP_ROTATE_RV						(MILLIRAD_PI / 4)
#define BUMP_ROTATE_TV						DEMO_TV

#define BUMP_AVOID_TV_MIN					15
#define BUMPMOVE_REVERSE_DIST				3
//### changed BUMPMOVE_REVERSE_DIST from 50 to 10

// simple behavior:  uses IR sensors to avoid obstacles (hopefully before the robot bumps them).
// If an obstacle is detected in front of the robot, the robot stops and turns away from it.
static Beh* behIRObstacleAvoid_Internal(Beh* behPtr, int32 tv, uint8 irBits, boolean invalidate) {
	// tv = translational velocity
	static uint8 IRObstacleMoveState = BUMPMOVE_IDLE;
	static uint8 IRObstacleMoveStateOld;
	//static uint16 bumpMoveTimer = 0;
	static Pose poseStart;
	Pose pose;
	int32 rv = tv * RVTV_RATIO;

	if ((irBits & IR_COMMS_RECEIVER_FRONT_LEFT_BIT) && (irBits & IR_COMMS_RECEIVER_FRONT_RIGHT_BIT)) {
		// obstacle in front
		IRObstacleMoveState = BUMPMOVE_OBS_FRONT;
	} else if (irBits & IR_COMMS_RECEIVER_FRONT_LEFT_BIT) {
		// obstacle in front left
		IRObstacleMoveState = BUMPMOVE_OBS_FRONT_LEFT;
	} else if (irBits & IR_COMMS_RECEIVER_LEFT_FRONT_BIT) {
		// obstacle in left
		IRObstacleMoveState = BUMPMOVE_OBS_LEFT;
	} else if (irBits & IR_COMMS_RECEIVER_FRONT_RIGHT_BIT) {
		// obstacle in front right
		IRObstacleMoveState = BUMPMOVE_OBS_FRONT_RIGHT;
	} else if (irBits & IR_COMMS_RECEIVER_RIGHT_FRONT_BIT) {
		// obstacle in right
		IRObstacleMoveState = BUMPMOVE_OBS_RIGHT;
	}

	if (IRObstacleMoveStateOld != IRObstacleMoveState) {
		encoderGetPose(&poseStart);
	}
	IRObstacleMoveStateOld = IRObstacleMoveState;

	encoderGetPose(&pose);
	switch (IRObstacleMoveState) {
	case BUMPMOVE_OBS_FRONT: {
		if (abs(poseAngleDiff(&pose, &poseStart)) > IROBSTACLE_MOVE_OBS_FRONT_ANGLE) {
			IRObstacleMoveState = BUMPMOVE_IDLE;
		} else {
			behPtr->tv = 0;
			//behPtr->rv = BUMP_ROTATE_RV * 2;
			behPtr->rv = rv * 2;
			behPtr->active = TRUE;
		}
		break;
	}
	case BUMPMOVE_OBS_FRONT_LEFT:
	case BUMPMOVE_OBS_FRONT_RIGHT: {
		if (abs(poseAngleDiff(&pose, &poseStart)) > IROBSTACLE_MOVE_OBS_FRONT_X_ANGLE) {
			IRObstacleMoveState = BUMPMOVE_IDLE;
		} else {
			//behPtr->tv = BUMP_ROTATE_TV;
			//behPtr->rv = (bumpMoveState == BUMPMOVE_OBS_FRONT_LEFT ? -BUMP_ROTATE_RV * 2 : BUMP_ROTATE_RV * 2);
			behPtr->tv = tv;
			behPtr->rv = (IRObstacleMoveState == BUMPMOVE_OBS_FRONT_LEFT ? -rv * 2 : rv * 2);

			behPtr->active = TRUE;
		}
		break;
	}
	case BUMPMOVE_OBS_LEFT:
	case BUMPMOVE_OBS_RIGHT: {
		if (abs(poseAngleDiff(&pose, &poseStart)) > IROBSTACLE_MOVE_OBS_X_ANGLE) {
			IRObstacleMoveState = BUMPMOVE_IDLE;
		} else {
			//behPtr->tv = BUMP_ROTATE_TV;
			//behPtr->rv = (bumpMoveState == BUMPMOVE_OBS_LEFT ? -BUMP_ROTATE_RV : BUMP_ROTATE_RV);
			behPtr->tv = tv;
			behPtr->rv = (IRObstacleMoveState == BUMPMOVE_OBS_LEFT ? -rv : rv);
			behPtr->active = TRUE;
		}
		break;
	}
	default:
		break;
	}
	// if we are called to invalidate the behavior (like if we got a bump),
	// then switch to idle, regardless of what we just did, and go inactive
	if (invalidate) {
		IRObstacleMoveState = BUMPMOVE_IDLE;
	}

	if (IRObstacleMoveState == BUMPMOVE_IDLE) {
		behPtr->active = FALSE;
	}
	return behPtr;
}

//Uses IR to avoid all obstacles, including robots. Can be invalidated.
Beh* behIRObstacleAvoid(Beh* behPtr, int32 tv, boolean invalidate) {
	Beh* behOutput;

	uint8 irBits = irObstaclesGetBits();
	behOutput = behIRObstacleAvoid_Internal(behPtr, tv, irBits, invalidate);

	return behOutput;
}

//Uses IR to avoid all obstacles, except robots. Can also be invalidated.
Beh* behIRObstacleAvoid_ExcludeRobots(Beh* behPtr, int32 tv, NbrList* nbrListPtr, boolean invalidate) {
	uint8 obstacleBitsGroup, obstacleBitsCount;
	int16 obstacleBearing;

	obstacleExcludeNbrs(nbrListPtr, &obstacleBitsGroup, &obstacleBitsCount, &obstacleBearing);

	static uint32 neighborRound;
	static boolean printNbrs;
	printNbrs = neighborsNewRoundCheck(&neighborRound);
//	if(printNbrs) cprintf("IR vector: %d\n", obstacleBitsCount);

	behIRObstacleAvoid_Internal(behPtr, tv, obstacleBitsGroup, invalidate);

	return behPtr;
}

//FIXME should be small back-up movements.
Beh* behBumpBackoff(Beh* behPtr, int32 tv) {
	static uint8 bumpMoveState = BUMPMOVE_IDLE;
	static uint8 bumpMoveStateOld;
	static Pose poseStart;
	Pose pose;
	int32 rv = tv * RVTV_RATIO * 2;

	if (bumpMoveState == BUMPMOVE_IDLE) {
		uint8 bumpSensorBits = bumpSensorsGetBits();
		int16 bumpSensorAngle = bumpSensorsGetBearing();
		if ((bumpSensorBits & BUMP_FL_BIT) && (bumpSensorBits & BUMP_FR_BIT)) {
			// obstacle in front
			bumpMoveState = BUMPMOVE_OBS_FRONT;
		} else if (bumpSensorBits & BUMP_FL_BIT) {
			// obstacle in front left
			bumpMoveState = BUMPMOVE_OBS_FRONT_LEFT;
		} else if (bumpSensorBits & BUMP_FR_BIT) {
			// obstacle in front right
			bumpMoveState = BUMPMOVE_OBS_FRONT_RIGHT;
		} else if (bumpSensorBits & BUMP_LF_BIT) {
			// obstacle in left
			bumpMoveState = BUMPMOVE_OBS_LEFT;
		} else if (bumpSensorBits & BUMP_RF_BIT) {
			// obstacle in right
			bumpMoveState = BUMPMOVE_OBS_RIGHT;
		}
		//cprintf("\n");
	}

	if (bumpMoveStateOld != bumpMoveState) {
		encoderGetPose(&poseStart);
	}
	bumpMoveStateOld = bumpMoveState;

	encoderGetPose(&pose);
	switch (bumpMoveState) {
	case BUMPMOVE_OBS_FRONT:
	case BUMPMOVE_OBS_FRONT_LEFT: /// added for improving bump behavior
		behPtr->tv = 0;
		behPtr->rv = rv;
		behPtr->active = TRUE;
		break;

	case BUMPMOVE_OBS_FRONT_RIGHT: {
		behPtr->tv = 0;
		behPtr->rv = rv;
		//behPtr->rv = 0;

		behPtr->active = TRUE;
		break;
	}
	case BUMPMOVE_OBS_LEFT:// added for improving bump behavior
		behPtr->tv = 0;
		behPtr->rv = rv;

		//behPtr->rv = 0;
		behPtr->active = TRUE;
		break;
	case BUMPMOVE_OBS_RIGHT: {//FIXME rotate and then move forward
		behPtr->tv = 0;
		behPtr->rv = rv;

		//behPtr->rv = 0;
		behPtr->active = TRUE;
		break;
	}//TODO drive forward if hit from behind
	default:
		break;
	}
	if (bumpMoveState == BUMPMOVE_IDLE) {
		behPtr->active = FALSE;
	}
	return behPtr;
}
////TODO: remove this.  There is a new file with wall behaviors: wallBehaviors.c
////LiTre
//Beh* behWallMove(Beh* behPtr) {
//	int16 bumpBearing = bumpSensorsGetBearing();
//	cprintf("Bump Bearing %d",bumpBearing);
//	if (bumpBearing == -1) {
//		behPtr->tv = 50;
//		behPtr->rv = 0;//move slightly toward wall or along it
//		behPtr->active = TRUE;
//	}
//	else if (bumpBearing < 0 && bumpBearing > -1700){
//		behPtr->tv = 50;
//		behPtr->rv = (1700 + bumpBearing);
//		behPtr->active = TRUE;
//	} else if (bumpBearing > 0 && bumpBearing < 1700) {
//		behPtr->tv = 50;
//		behPtr->rv = (1700 - bumpBearing) * 4;
//		behPtr->active = TRUE;
//	}
//
//	return behPtr;
//	//Rotate along wall
//
//			// Figure out the direction and type of bump and rotate and behave such that the side of the robot
//			//is constantly bumped in parallel with the path of the robot
//		/*static Pose poseStateStart;
//		Pose pose;
//		int32 rv;
//		int32 rotateDist = BUMPMOVE_OBS_FRONT_ANGLE;
//		int8 direction = 1;
//		uint8 bumpType = getBumptypeFront(bumpSensorsGetBits());
//		if (bumpType == BUMPTYPE_OBS_FRONT) {
//			// Obstacle in front
//			rotateDist = BUMPMOVE_OBS_FRONT_ANGLE;
//			direction = 1;
//		} else if (bumpType == BUMPTYPE_OBS_FRONT_LEFT) {
//			// Obstacle on the front left
//			rotateDist = BUMPMOVE_OBS_FRONT_X_ANGLE;
//			direction = -1;
//		} else if (bumpType == BUMPTYPE_OBS_FRONT_RIGHT) {
//			// Obstacle on the front right
//			rotateDist = BUMPMOVE_OBS_FRONT_X_ANGLE;
//			direction = 1;
//		} else if (bumpType == BUMPTYPE_OBS_LEFT) {
//			// Obstacle on the left
//			rotateDist = BUMPMOVE_OBS_X_ANGLE;
//			direction = -1;
//		} else if (bumpType == BUMPTYPE_OBS_RIGHT) {
//			// Obstacle on the right
//			rotateDist = BUMPMOVE_OBS_X_ANGLE;
//			direction = 1;
//		}
//		//TODO: test for corrective wall traversing
//		// Rotate so that bumped bits become are on the side of the robot
//			//this will confirm that we are ready to move along the wayy
//		if (abs(poseAngleDiff(&pose, &poseStateStart)) > rotateDist) {
//			// Forward/backward algorithm for finding a corner??
//				//TODO: to be partially implemented by SKLee
//					//move forward x ticks and check for corner/vertex on wall
//					//then move bawkward 2x ticks... forward 3x ticks. etc.
//
//			behPtr->tv = 50;//default translational velocity?
//			behPtr->rv = 0;
//			behPtr->active = TRUE;
//		} else if() {
//			behPtr->tv = 0;
//			behPtr->rv = 500;
//			behPtr->active = TRUE;
//		} else{
//			behPtr->tv = 0;
//			behPtr->rv = 500;
//			behPtr->active = TRUE;
//		}*/
//}


/*
 * Determine which direction the obstacle seems to be coming from if there
 * is a bump in the front. If there was no front bump, returns BUMPTYPE_NONE.
 */
static uint8 getBumptypeFront(uint8 bumpSensorBits) {
	uint8 bumpType = BUMPTYPE_NONE;

	if ((bumpSensorBits & BUMP_FL_BIT) && (bumpSensorBits & BUMP_FR_BIT)) {
		// obstacle in front
		bumpType = BUMPTYPE_OBS_FRONT;
	} else if (bumpSensorBits & BUMP_FL_BIT) {
		// obstacle in front left
		bumpType = BUMPTYPE_OBS_FRONT_LEFT;
	} else if (bumpSensorBits & BUMP_FR_BIT) {
		// obstacle in front right
		bumpType = BUMPTYPE_OBS_FRONT_RIGHT;
	} else if (bumpSensorBits & BUMP_LF_BIT) {
		// obstacle in left
		bumpType = BUMPTYPE_OBS_LEFT;
	} else if (bumpSensorBits & BUMP_RF_BIT) {
		// obstacle in right
		bumpType = BUMPTYPE_OBS_RIGHT;
	}
	
	return bumpType;
}


// simple behavior that uses bump sensors to determine if robot should avoid an obstacle.
// If obstacle is detected with bump sensors, the robot turns in place.
// After the bump sensor turns off, the robot moves forward <forwardDist> mm.
// If the robot does not bump into anything, this behavior becomes inactive.
//
 Beh* behBumpAvoid(Beh* behPtr, int32 tv, int32 forwardDist) {
	static uint8 bumpMoveState = BUMPMOVE_IDLE;
	static uint8 bumpType = BUMPTYPE_NONE;
	static Pose poseStateStart;
	Pose pose;
	int32 rv;

	if (tv < BUMP_AVOID_TV_MIN) {
		tv = BUMP_AVOID_TV_MIN;
	}
	rv = (tv * RVTV_RATIO * 25)/10;

	// Start off being inactive
	*behPtr = behInactive;

	// Read the bump sensor
	uint8 bumpSensorBits = bumpSensorsGetBits();

	// Record the current position
	encoderGetPose(&pose);

	switch (bumpMoveState) {
	case BUMPMOVE_IDLE: {
		// Look for bumps, otherwise don't do anything
		bumpType = getBumptypeFront(bumpSensorBits);
		if (bumpType != BUMPTYPE_NONE) {
			// If we ran into something on the front, back up
			if (bumpType == BUMPTYPE_OBS_FRONT ||
				bumpType == BUMPTYPE_OBS_FRONT_LEFT ||
				bumpType == BUMPTYPE_OBS_FRONT_RIGHT) {
				// Record the starting pose and start reversing the robot
				bumpMoveState = BUMPMOVE_REVERSE;
			} else {
				// Otherwise if we ran into something just rotate away
				bumpMoveState = BUMPMOVE_ROTATE;
			}
			// Record our current position for the next state
			poseStateStart = pose;
			// Stop so that we can back up or rotate effectively
			behPtr->tv = 0;
			behPtr->rv = 0;
			behPtr->active = TRUE;
		}
		break;
	}
	case BUMPMOVE_REVERSE: {
		// Reverse the robot for a small amount so we can turn (only about 5mm)
		int32 dist = abs(poseDistance(&poseStateStart, &pose));
		if(dist > BUMPMOVE_REVERSE_DIST) {
			// Turn the direction we need to go
			poseStateStart = pose;
			bumpMoveState = BUMPMOVE_ROTATE;
		} else {
			// Reverse a little
			behPtr->tv = -tv;
			behPtr->rv = 0;
			behPtr->active = TRUE;
		}

		break;
	}
	case BUMPMOVE_ROTATE: {
		// Rotate in the direction we need to go to best avoid the obstacle
		// for the distance we need.
		int32 rotateDist = BUMPMOVE_OBS_X_ANGLE;
		int8 direction = 1;

		// Figure out the direction and type of bump
		if (bumpType == BUMPTYPE_OBS_FRONT) {
			// Obstacle in front
			rotateDist = BUMPMOVE_OBS_FRONT_ANGLE;
			direction = 1;
		} else if (bumpType == BUMPTYPE_OBS_FRONT_LEFT) {
			// Obstacle on the front left
			rotateDist = BUMPMOVE_OBS_FRONT_X_ANGLE;
			direction = -1;
		} else if (bumpType == BUMPTYPE_OBS_FRONT_RIGHT) {
			// Obstacle on the front right
			rotateDist = BUMPMOVE_OBS_FRONT_X_ANGLE;
			direction = 1;
		} else if (bumpType == BUMPTYPE_OBS_LEFT) {
			// Obstacle on the left
			rotateDist = BUMPMOVE_OBS_X_ANGLE;
			direction = -1;
		} else if (bumpType == BUMPTYPE_OBS_RIGHT) {
			// Obstacle on the right
			rotateDist = BUMPMOVE_OBS_X_ANGLE;
			direction = 1;
		}

		if (rotateDist > MILLIRAD_HALF_PI){
			rotateDist = MILLIRAD_HALF_PI;
		}

		// Add random factor to avoid synchronized bump beh
		if (rand()%2==1){
			rotateDist += (rand()%15 * rotateDist)/100;
		} else {
			rotateDist -= (rand()%15 * rotateDist)/100;
		}

		// Rotate that way until we are done
		if (abs(poseAngleDiff(&pose, &poseStateStart)) > rotateDist) {
			// And get going!
			bumpMoveState = BUMPMOVE_MOVE_FORWARD;
			poseStateStart = pose;
		} else {
			behPtr->tv = 0;
			behPtr->rv = direction * rv;
			behPtr->active = TRUE;
		}

		break;
	}
	case BUMPMOVE_MOVE_FORWARD: {
		/* Move forward the distance requested, but if there is another bump,
		 * rotate and deal with it, don't sit there like a lump against a wall.
		 */
		bumpType = getBumptypeFront(bumpSensorBits);
		if (bumpType != BUMPTYPE_NONE) {
			// Turn away! Record the starting pose so we know how far
			poseStateStart = pose;
			bumpMoveState = BUMPMOVE_ROTATE;
		} else {
			// We are in the clear to move forward
			int32 dist = abs(poseDistance(&poseStateStart, &pose));
			if (dist > forwardDist) {
				poseStateStart = pose;
				bumpMoveState = BUMPMOVE_IDLE;
			} else {
				behPtr->tv = tv;
				behPtr->rv = 0;
				behPtr->active = TRUE;
			}
		}
		break;
	}
	default: {
		break;
	}
	}

	// Return the behavior we should do
	return behPtr;
}



// simple behavior that uses bump sensors to determine if robot should avoid an obstacle.
// If obstacle is detected with bump sensors, the robot turns in place.
// After the bump sensor turns off, the robot moves forward <forwardDist> mm.
// If the robot does not bump into anything, this behavior becomes inactive.
//
Beh* behBumpAvoidWithRvHistory(Beh* behPtr, int32 tv, int32 forwardDist, int32 previousRv) {
	static uint8 bumpMoveState = BUMPMOVE_IDLE;
	static uint8 bumpType = BUMPTYPE_NONE;
	static Pose poseStateStart;
	static uint32 odometerForwardStart = 0;
	Pose pose;
	int32 rv;

	if (tv < BUMP_AVOID_TV_MIN) {
		tv = BUMP_AVOID_TV_MIN;
	}
	rv = tv * RVTV_RATIO * 2;

	// Start off being inactive
	*behPtr = behInactive;

	// Read the bump sensor
	uint8 bumpSensorBits = bumpSensorsGetBits();

	// Record the current position
	encoderGetPose(&pose);

	switch (bumpMoveState) {
	case BUMPMOVE_IDLE: {
		// Look for bumps, otherwise don't do anything
		bumpType = getBumptypeFront(bumpSensorBits);
		if (bumpType != BUMPTYPE_NONE) {
			// If we ran into something on the front, back up
			if (bumpType == BUMPTYPE_OBS_FRONT ||
				bumpType == BUMPTYPE_OBS_FRONT_LEFT ||
				bumpType == BUMPTYPE_OBS_FRONT_RIGHT) {
				// Record the starting pose and start reversing the robot
				bumpMoveState = BUMPMOVE_REVERSE;
			} else {
				// Otherwise if we ran into something just rotate away
				bumpMoveState = BUMPMOVE_ROTATE;
			}
			// Record our current position for the next state
			poseStateStart = pose;
			// Stop so that we can back up or rotate effectively
			behPtr->tv = 0;
			behPtr->rv = 0;
			behPtr->active = TRUE;
		}
		break;
	}
	case BUMPMOVE_REVERSE: {
		// Reverse the robot for a small amount so we can turn (only about 5mm)
		int32 dist = abs(poseDistance(&poseStateStart, &pose));
		if(dist > forwardDist) {
			// Record our odometer so we know how far to go
			odometerForwardStart = encoderGetOdometer();
			// Turn the direction we need to go
			poseStateStart = pose;
			bumpMoveState = BUMPMOVE_ROTATE;
		} else {
			// Reverse a little
			behPtr->tv = -tv;
			behPtr->rv = 0;
			behPtr->active = TRUE;
		}

		break;
	}
	case BUMPMOVE_ROTATE: {
		// Rotate in the direction we need to go to best avoid the obstacle
		// for the distance we need.
		int32 rotateDist = BUMPMOVE_OBS_X_ANGLE;

		int8 direction = 1;
		if (previousRv < 0){
			direction = -1;
		}

		// Figure out the direction and type of bump
		if (bumpType == BUMPTYPE_OBS_FRONT) {
			// Obstacle in front
			rotateDist = BUMPMOVE_OBS_FRONT_ANGLE;
		} else if (bumpType == BUMPTYPE_OBS_FRONT_LEFT) {
			// Obstacle on the front left
			rotateDist = BUMPMOVE_OBS_FRONT_X_ANGLE;
		} else if (bumpType == BUMPTYPE_OBS_FRONT_RIGHT) {
			// Obstacle on the front right
			rotateDist = BUMPMOVE_OBS_FRONT_X_ANGLE;
		} else if (bumpType == BUMPTYPE_OBS_LEFT) {
			// Obstacle on the left
			rotateDist = BUMPMOVE_OBS_X_ANGLE;
		} else if (bumpType == BUMPTYPE_OBS_RIGHT) {
			// Obstacle on the right
			rotateDist = BUMPMOVE_OBS_X_ANGLE;
		}

		// Rotate that way until we are done
		if (abs(poseAngleDiff(&pose, &poseStateStart)) > rotateDist) {
			// And get going!
			bumpMoveState = BUMPMOVE_MOVE_FORWARD;
		} else {
			behPtr->tv = 0;
			behPtr->rv = direction * rv;
			behPtr->active = TRUE;
		}

		break;
	}
	case BUMPMOVE_MOVE_FORWARD: {
		/*
		 * Move forward the distance requested, but if there is another bump,
		 * rotate and deal with it, don't sit there like a lump against a wall.
		 */
		bumpType = getBumptypeFront(bumpSensorBits);
		if (bumpType != BUMPTYPE_NONE) {
			// Turn away! Record the starting pose so we know how far
			poseStateStart = pose;
			bumpMoveState = BUMPMOVE_ROTATE;
		} else {
			// We are in the clear to move forward
			int32 dist = encoderGetOdometer() - odometerForwardStart;
			if (dist > forwardDist) {
				poseStateStart = pose;
				bumpMoveState = BUMPMOVE_IDLE;
			} else {
				behPtr->tv = tv;
				behPtr->rv = 0;
				behPtr->active = TRUE;
			}
		}
		break;
	}
	default: {
		break;
	}
	}

	// Return the behavior we should do
	return behPtr;
}


/*
 * @brief If bumped, robot will move slightly forward out of way
 *
 * @param behPtr pointer for behavior to be modified
 * @returns updated behPtr
 */
Beh* behBumpMoveOutOfWay(Beh* behPtr)
{
	static int obsAvoidState = MOVEOUT_IDLE;
	static uint32 startTime = 0;
	uint32 currentTime = osTaskGetTickCount();
	uint8 bumpDirection = bumpSensorsWhichHit();
	uint32 moveTime = 1000;

	//cprintf("\n obsAvoidState = %d bumpDirection = %d",obsAvoidState, bumpDirection);

	switch(obsAvoidState)
	{
	case(MOVEOUT_IDLE):
	{
		if (bumpDirection == 0)
		{ //Hit in front
			obsAvoidState = MOVEOUT_FRONT_HIT;
		}
		else if (bumpDirection == 1)
		{ //Hit on left
			obsAvoidState = MOVEOUT_LEFT_HIT;
		}
		else if (bumpDirection == 2)
		{ //Hit from behind
			obsAvoidState = MOVEOUT_BACK_HIT;
		}
		else if (bumpDirection == 3)
		{ //Hit from right
			obsAvoidState = MOVEOUT_RIGHT_HIT;
		}
		else
		{	//NOT hit
			obsAvoidState = MOVEOUT_IDLE;
			behSetActive(behPtr);
			behSetTv(behPtr, 0);
			behSetRv(behPtr, 0);
			//behSetInactive(behPtr);
		}
		startTime = osTaskGetTickCount();
		break;

	}
	case(MOVEOUT_FRONT_HIT):
	{
		if(currentTime - startTime < moveTime)
		{
			behSetActive(behPtr);
			behSetTv(behPtr, -40);
		}
		else
		{
			obsAvoidState = MOVEOUT_IDLE;
			behSetInactive(behPtr);
		}
		break;
	}
	case(MOVEOUT_LEFT_HIT):
	{
		if(currentTime - startTime < moveTime)
		{
			behSetActive(behPtr);
			behSetTv(behPtr, 0);
			behSetRv(behPtr, -400);
		}
		else
		{
			obsAvoidState = MOVEOUT_IDLE;
			behSetInactive(behPtr);
		}
		break;
	}
	case(MOVEOUT_BACK_HIT):
		{
			if(currentTime - startTime < moveTime)
			{
				behSetActive(behPtr);
				behSetTv(behPtr, 40);
				behSetRv(behPtr, 0);
			}
			else
			{
				obsAvoidState = MOVEOUT_IDLE;
				behSetInactive(behPtr);
			}
			break;
		}
	case(MOVEOUT_RIGHT_HIT):
			{
				if(currentTime - startTime < moveTime)
				{
					behSetActive(behPtr);
					behSetTv(behPtr, 0);
					behSetRv(behPtr, 400);
				}
				else
				{
					obsAvoidState = MOVEOUT_IDLE;
					behSetInactive(behPtr);
				}
				break;
			}
		default:
		{
			obsAvoidState = MOVEOUT_IDLE;
			break;
		}
	}//end switch(obsAvoidState)
	return behPtr;
}//behBumpMoveOutOfWay


/*
 *  @brief If bumped, robot will back up, turn in the direction is was traveling, then move forward.
 *
 *  @param behPtr for behavior that is updated by this function
 *  @param headingRobot the heading of the robot before bumping
 *
 *  0 - Straight
 *  1 - Left
 *  2 - Right
 *
 *  @returns updated behPtr
 */
Beh* behBumpNavigate(Beh* behPtr, uint8 HeadingRobot)
{
	static uint8 obstacleAvoidanceState = OBSTACLE_AVOIDANCE_STATE_IDLE;
	static uint32 obstacleAvoidanceStateStartTime = 0;
	const uint32 OBSTACLE_AVOIDANCE_STATE_BACKUP_TIME = 500;
	const uint32 OBSTACLE_AVOIDANCE_STATE_ROTATE_TIME = 1000;
	const uint32 OBSTACLE_AVOIDANCE_STATE_FORWARD_TIME = 400; //800 big angle

	uint32 currentTime = osTaskGetTickCount();
	uint8 bumpSensorBits = bumpSensorsGetBits();
	//cprintf("\n obsAvoidState = %d", obstacleAvoidanceState);

	switch (obstacleAvoidanceState)
	{
		case OBSTACLE_AVOIDANCE_STATE_IDLE:
		{
			if (bumpSensorBits != 0)
			{
				obstacleAvoidanceState = OBSTACLE_AVOIDANCE_STATE_BACKUP;
				obstacleAvoidanceStateStartTime = currentTime;
			}
			else
			{
				behSetInactive(behPtr);
			}
			break;
		}//IDLE
		case OBSTACLE_AVOIDANCE_STATE_BACKUP:
		{
			if ((currentTime - obstacleAvoidanceStateStartTime) < OBSTACLE_AVOIDANCE_STATE_BACKUP_TIME)
			{
				behSetTv(behPtr, -50);
				behSetRv(behPtr, 0);
				behSetActive(behPtr);
				//cprintf("\n I AM BACKING UP");
			}
			else
			{
				obstacleAvoidanceState = OBSTACLE_AVOIDANCE_STATE_ROTATE;
				obstacleAvoidanceStateStartTime = currentTime;
			}
			break;
		}//BACKUP
		case OBSTACLE_AVOIDANCE_STATE_ROTATE:
		{
			if ((currentTime - obstacleAvoidanceStateStartTime) < OBSTACLE_AVOIDANCE_STATE_ROTATE_TIME)
			{

				behSetTv(behPtr, 0);
				if (HeadingRobot == 0)
				{
					behSetRv(behPtr, 600);
				} else if (HeadingRobot == 1) //Left
				{
					behSetRv(behPtr, -600);
				}
				else if (HeadingRobot == 2) //right
				{
					behSetRv(behPtr, 600);
				}
				behSetActive(behPtr);
				//cprintf("\n I AM ROTATING");
			}
			else
			{
				obstacleAvoidanceState = OBSTACLE_AVOIDANCE_STATE_FORWARD;
				obstacleAvoidanceStateStartTime = currentTime;
			}
			break;
		}//ROTATE
		case OBSTACLE_AVOIDANCE_STATE_FORWARD:
		{
			if ((currentTime - obstacleAvoidanceStateStartTime) < OBSTACLE_AVOIDANCE_STATE_FORWARD_TIME)
			{

				behSetTv(behPtr, 50);
				behSetRv(behPtr, 0);
				behSetActive(behPtr);
				//cprintf("\n I AM MOVING FORWARD");
			}
			else
			{
				obstacleAvoidanceState = OBSTACLE_AVOIDANCE_STATE_IDLE;
				obstacleAvoidanceStateStartTime = currentTime;
			}
			break;
		}//FORWARD
		default:
		{
			behSetInactive(behPtr);
			break;
		}//default
	} //switch
	return behPtr;
}//behBumpNavigate


#define BANGLE_MULTIPLIER	3
//TODO: Does this work? What is the desired behavior?
/*
 * @breif
 *
 * @param behPtr
 * @param tv
 * @param reflectDist
 * @returns updated behPtr
 */

Beh* behBumpReflect(Beh* behPtr, int32 tv, int32 reflectDist) {
	static uint8 bumpMoveState = BUMPMOVE_IDLE;
	static Pose poseBumpStart;
	static int32 bumpSensorAngle;
	static int32 bumpRotateAngle;
	Pose pose;
	int32 rv = tv * RVTV_RATIO * 2;

	if(bumpMoveState == BUMPMOVE_IDLE) {
		bumpSensorAngle = bumpSensorsGetBearing();
		if(bumpSensorAngle!=-1) {	//new collision
			bumpSensorAngle = normalizeAngleMilliRad2(bumpSensorAngle);
			if (abs(bumpSensorAngle) < MILLIRAD_DEG_90) {	//new front collision
				encoderGetPose(&poseBumpStart);
				if(abs(bumpSensorAngle) < MILLIRAD_DEG_15) {
					bumpRotateAngle = MILLIRAD_PI - MILLIRAD_DEG_15;	//no perfect perpendicular reflection
				} else {
					bumpRotateAngle = MILLIRAD_PI - abs(2 * bumpSensorAngle);
				}
				bumpMoveState = BUMPMOVE_REFLECT_ROTATE;
			}

		}
	}
	if(bumpMoveState==BUMPMOVE_REFLECT_ROTATE) {	//rotating
		encoderGetPose(&pose);
		int32 theta = poseAngleDiff(&pose, &poseBumpStart);
		if (abs(theta) > abs(bumpRotateAngle)) {
			bumpMoveState = BUMPMOVE_IDLE;
		} else {
			behPtr->tv = 0;
			if (bumpSensorAngle > 0) {
				behPtr->rv = -rv;
			} else {
				behPtr->rv = rv;
			}
			behPtr->active = TRUE;
		}
	}
	if(bumpMoveState == BUMPMOVE_IDLE) {
		behPtr->active = FALSE;
	}
	return behPtr;
}
/*
Beh* behBumpReflect(Beh* behPtr, int32 tv, int32 reflectDist) {
	static uint8 bumpMoveState = BUMPMOVE_IDLE;
	static Pose poseBumpStart;
	static int32 bumpSensorAngle;
	static int32 bumpRotateAngle;
	Pose pose;
	int32 rv = tv * RVTV_RATIO * 2;

	encoderGetPose(&pose);
	bumpSensorAngle = bumpSensorsGetBearing();
	//cprintf("bump sensors % 5d\n", bumpSensorAngle);
	if (bumpSensorAngle > 0) {
		bumpSensorAngle = normalizeAngleMilliRad2(bumpSensorAngle);
		if (abs(bumpSensorAngle) < MILLIRAD_DEG_90) {
			// only deal with forward bumps for now
			// perfect reflection  - doesn't work very well
			bumpRotateAngle = MILLIRAD_PI - abs(2 * bumpSensorAngle);
			if(bumpSensorAngle<0) bumpRotateAngle=-bumpRotateAngle;
			//				if (bangle < MILLIRAD_PI) {
			//					bumpRotateAngle = MILLIRAD_PI - bangle;
			//				} else {
			//					bumpRotateAngle = MILLIRAD_DEG_45;
			//				}

			// scaled reflection - but scale was too large
			//int32 bangle = abs(BANGLE_MULTIPLIER * bumpSensorAngle);

			// rotate the same angle as angle of contact
			//int32 bangle = abs(bumpSensorAngle);

			// rotate a fixed distance
			//int32 bangle = MILLIRAD_DEG_20;
			//bumpRotateAngle = bangle;

			encoderGetPose(&poseBumpStart);
			bumpMoveState = BUMPMOVE_REFLECT_ROTATE;
		}
	}

	bumpSensorAngle = bumpSensorsGetBearing();
	//cprintf("bump sensors % 5d\n", bumpSensorAngle);
	if (bumpSensorAngle > 0) {
		bumpSensorAngle = normalizeAngleMilliRad2(bumpSensorAngle);
		if (abs(bumpSensorAngle) < MILLIRAD_DEG_90) {
			// only deal with forward bumps for now
			// perfect reflection  - doesn't work very well
			//bumpRotateAngle = MILLIRAD_PI - abs(2 * bumpSensorAngle);
			//				if (bangle < MILLIRAD_PI) {
			//					bumpRotateAngle = MILLIRAD_PI - bangle;
			//				} else {
			//					bumpRotateAngle = MILLIRAD_DEG_45;
			//				}

			// scaled reflection - but scale was too large
			//int32 bangle = abs(BANGLE_MULTIPLIER * bumpSensorAngle);

			// rotate the same angle as angle of contact
			//int32 bangle = abs(bumpSensorAngle);

			// rotate a fixed distance
			int32 bangle = MILLIRAD_DEG_20;
			bumpRotateAngle = bangle;

			encoderGetPose(&poseBumpStart);
			bumpMoveState = BUMPMOVE_REFLECT_ROTATE;
		}
	}

	switch (bumpMoveState) {
	case BUMPMOVE_IDLE:
	case BUMPMOVE_MOVE_FORWARD: {
		// used to check sensors here
		if (bumpMoveState == BUMPMOVE_MOVE_FORWARD) {
			int32 dist = poseDistance(&pose, &poseBumpStart);
			//cprintf("bump dist %d\n", dist);
			if (dist > reflectDist) {
				bumpMoveState = BUMPMOVE_IDLE;
			} else {
				behPtr->tv = tv;
				behPtr->rv = 0;
				behPtr->active = TRUE;
			}
		}
		//cprintf("\n");
		break;
	}
	case BUMPMOVE_REFLECT_ROTATE: {
		int32 theta = poseAngleDiff(&pose, &poseBumpStart);
		if (abs(theta) > abs(bumpRotateAngle)) {
			bumpMoveState = BUMPMOVE_MOVE_FORWARD;
		} else {
			behPtr->tv = 0;
			if (bumpSensorAngle > 0) {
				behPtr->rv = -rv;
			} else {
				behPtr->rv = rv;
			}
			behPtr->active = TRUE;
		}
		break;
	}
	default:
		break;
	}

	if (bumpMoveState == BUMPMOVE_IDLE) {
		behPtr->active = FALSE;
	}
	return behPtr;
}
*/

/*
 * @brief If the front bump sensors are activated, the robot will rotate.
 *
 * This is obtained by turning off translational velocity and setting rotational velocity to
 * the input rotate_rv in the behPtr.
 * @param behPtr pointer for the behavior to update
 * @param rotate_rv desired rotational velocity if the robot has encountered an obstacle
 * @returns void
 */
void behBumpReflectDemo(Beh* behPtr, int32 rotate_rv) {
	static uint8 bumpMoveState = BUMPMOVE_IDLE;
	static Pose poseBumpStart;
	static int32 bumpSensorAngle;
	static int32 bumpRotateAngle;
	Pose pose;
	//	int32 rv = tv * RVTV_RATIO * 2;

	behPtr->active = FALSE;
	encoderGetPose(&pose);
	bumpSensorAngle = bumpSensorsGetBearing();
	//cprintf("bump sensors % 5d\n", bumpSensorAngle);
	if (bumpSensorAngle > 0) {
		bumpSensorAngle = normalizeAngleMilliRad2(bumpSensorAngle);
		if (abs(bumpSensorAngle) < MILLIRAD_DEG_90) {
			if (bumpSensorAngle > 0) { //robot hit something on left side, so rotate right.
				behPtr->rv  = -rotate_rv;
				behPtr->tv = 0;
				behPtr->active = TRUE;
			}
			else { //robot hit something on right side, so rotate left.
				behPtr->rv = rotate_rv;
				behPtr->active = TRUE;
				behPtr->tv = 0;
			}
		}

	}
}


