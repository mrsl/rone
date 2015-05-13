/**
 * @file encoder.h
 *
 * @brief Deals with who the encoder ticks and its translation to distance
 *
 * There is an assumption that encoder_pose_update() will be called on a regular basis
 * to ensure accuracy.
 * Keep updating in micrometers.
 * The encoders have a resolution of 0.0625mm. That is, each tick represents
 * a change of 0.0625 mm. Our pose is stored in micrometers and microradians.
 * Each tick corresponds to 62.5 micrometers. Rounding to 63.
 *
 * x,y coordinates are stored in micrometers
 * theta is represented in microradians
 *
 * @since Mar 26, 2011
 * @author James McLurkin
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

/******** Defines ********/

#define ENCODER_LEFT            0
#define ENCODER_RIGHT            1
//Micrometerper Encoder count: for 66, robot does not go far enough (1138 mm / 1200 mm)
//Micrometerper Encoder count: for 62, robot goes too far (1211 mm / 1200 mm)
#define MOTOR_MICROMETERS_PER_ENCODER_COUNT 63 //changed to 63 Um down from 66
// wheel base is in mm.  is actually 78, BUT (TURNS TOO SMALL)                       81, 83, 89 TURNS TOO BIG
#define MOTOR_WHEEL_BASE_MM  78



/******** Functions ********/


/**
 * @brief Initializes the encoder.
 *
 * Enables the peripherals.
 * Sets the state of the odometer to 0,0,0.
 * @returns void
 */
void encoderInit(void);


/**
 * @brief Initializes the encoder.
 *
 * Enables the peripherals.
 * Sets the state of the odometer to 0,0,0.
 * @returns void
 */
int32 encoderGetTicks(uint32 enc);


/**
 * @brief Gets the current rotating direction of the encoder.
 *
 * @param enc specifies which encoder (right or left) to look up
 * @returns the current rotating direction
 */
int32 encoderGetDirection(uint32 enc);


/**
 * @brief Gets the current velocity of the specified encoder.
 *
 * @param enc specifies which encoder (right or left) to look up
 * @returns the current velocity
 */
int32 encoderGetVelocity(uint32 enc);


/**
 * @brief Gets the difference between the two input ticks.
 *
 * @param new is the new encoder position
 * @param old is the old encoder position
 * @returns the difference between old and new position with rollover protection
 */
int32 encoderDeltaTicks(uint32 new, uint32 old);


/**
 * @brief Updates the encoder's position.
 *
 * @returns void
 */
void encoderPoseUpdate(void);


/**
 * @brief Clears the x, y, theta and odometer values of the encoder.
 *
 * @returns void
 */
void encoderPoseClear(void);


/**
 *	@brief Gets the pose of the encoder and stores it in variables (x, y, theta) of where posePtr points to.
 *
 *	@param posePtr points to a Pose structure
 *	@returns void
 */
void encoderGetPose(Pose* posePtr);


/**
 *	@brief Gets the heading of the robot.
 *
 *	@returns the current heading in milliradians
 */
int32 encoderGetHeading(void);


/**
 * 	@brief Sets the pose of the encoder as variables of where posePtr points to.
 *
 * 	@param posePtr points to a Pose structure
 * 	@returns void
 */
void encoderSetPose(Pose* posePtr);


/**
 *	@brief Finds and returns the odometer reading in mm
 *
 *	@returns odometer in mm
 */
uint32 encoderGetOdometer(void);


#endif /* ENCODERS_H_ */
