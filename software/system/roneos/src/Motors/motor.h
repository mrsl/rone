/**
 * @file motor.h
 *
 * @brief Functions that control the two motors on the rone.
 * @since Mar 23, 2011
 * @author James McLurkin
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/******** Defines ********/

#define MOTOR_LEFT                          0
#define MOTOR_RIGHT                         1

#define MOTOR_COMMAND_MODE_PWM				0
#define MOTOR_COMMAND_MODE_VELOCITY			1
#define MOTOR_COMMAND_MODE_WAYPOINT			2
#define MOTOR_COMMAND_MODE_WAYPOINT_THETA	3

#define VELOCITY_MAX 		300

/******** Functions ********/

/**
 * @brief Initializes motor.
 *
 * Enables PWM and initializes the motorVelocityData struct for both left and right motor.
 * Also sets the command timer to 0
 * @returns void
 */
void motorInit(void);


/**
 * @brief Turns off the motors at a low level
 *
 * @returns void
 */
void motorTimeoutStop(void);


/**
 * @brief Updates the motor command timer.
 *
 * If no motor command has been received, timeout and enables the charger.
 * This function should be called at 10 Hz.
 * @returns void
 */
void motorCommandTimerUpdate(void);


/**
 * @brief Brakes one motor with the specified duty cycle.
 *
 * @param motor (left or right)
 * @returns void
 */
void motorBrake(uint32 motor);


/**
 * @brief Sets PWM duty cycle for the specified motor if remote control mode is off.
 *
 * Sets PWM duty cycle for the specified motor for both reverse and forward signals.
 * @param motor left or right motor
 * @param dutyCycle duty cycle of PWM
 * @returns void
 */
void motorSetPWM(uint32 motor, int32 dutyCycle);

//TODO: Not public/?
/**
 * \internal
 * @brief Sets PWM duty cycle for the specified motor. Should not be called by the user.
 *        Call motorSetPWM() instead.
 *
 * Sets PWM duty cycle for the specified motor for both reverse and forward signals.
 * @param motor left or right motor
 * @param dutyCycle duty cycle of PWM
 * @returns void
 * \endinternal
 */
void motorSetPWM_rc(uint32 motor, int32 dutyCycle);


/**
 * @brief Gets the current velocity of the specified motor.
 *
 * @param motor left or right motor
 * @returns the current velocity; 0 if the input parameter is not recognized
 */
int32 motorGetVelocity(uint32 motor);

void motorSetTVRV_NonCmd(int32 tv, int32 rv);


/**
 * @brief Sets the velocity of the specified motor if remote control
 *        mode is off.
 *
 * @param motor left or right motor
 * @param velocity motor velocity to be set in mm/s
 * @returns void
 */
void motorSetVelocity(uint32 motor, int32 velocity);


/**
 * @brief Sets the velocity of the specified motor. Should not be called by user.
 *
 * @param motor left or right motor
 * @param velocity motor velocity to be set in mm/s
 * @returns void
 */
void motorSetVelocity_rc(uint32 motor, int32 velocity);


/**
 * @brief Sets the translation and radial velocity of the motors if remote control
 *        mode is off.
 *
 * @param tv the translational velocity
 * @param rv the rotational velocity
 * @returns void
 */
void motorSetTVRV(int32 tv, int32 rv);

/**
* @brief Sets the translation and radial velocity of the motors. Should not be
*        called by the user. Call motorSetTVRV instead.
*
* @param tv the translational velocity
* @param rv the rotational velocity
* @returns void
*/
void motorSetTVRV_rc(int32 tv, int32 rv);

/**
 * @brief Gets the translational and radial velocity of the motor.
 *
 * @param tvPtr pointer to the desired translational velocity
 * @param rvPtr pointer to the desired rotational velocity
 * @returns void
 */
void motorGetTVRV(int32* tvPtr, int32* rvPtr);

/**
 * @brief Returns whether current waypoint has been reached.
 *
 * @returns waypointDone boolean that keeps track of whether the current waypoint has been reached or not
 */
boolean waypointMoveDone(void);


/**
 * @brief MotorCommandMode set so that waypoint behavior  can be set with absolute coordinates.
 * Sets Goal point and speed according to Parameters
 *
 * @param posePtr - Pointer to position of Goal
 * @param speed
 * @returns void
 */
void waypointMove(Pose* posePtr, int32 speed);

/**
 * @brief MotorCommandMode set so that waypoint behavior can be set with relative coordinates.
 * Sets Goal point and speed according to Parameters
 *
 * @param posePtr - Pointer to position of Goal
 * @param speed
 *
 * @returns void
 */
void waypointMoveRelative(Pose* posePtr, int32 speed);


/**
* @brief MotorCommandMode set so that waypoint behavior can be set with absolute coordinates for theta.
* Sets Goal point and speed according to Parameters
*
* @param posePtr - Pointer to postion of Goal
* @param speed
* @returns void
*/
void waypointMoveTheta(Pose* posePtr, int32 speed);

/**
 * @brief MotorCommandMode set so that waypoint behavior can be set with relative coordinates for theta.
 * Sets Goal point and speed according to Parameters
 *
 * @param posePtr - Pointer to position of Goal
 * @param speed
 *
 * @returns void
 */
void waypointMoveThetaRelative(Pose* posePtr, int32 speed);

void waypointMoveUpdate(void);

/**
 * @brief Updates the velocity data for both motors.
 * Does this for both coordinates and theta
 *
 * @returns void
 */
void motorVelocityUpdate(void);


#endif /* MOTOR_H_ */
