/**
 * @file bumpBehaviors.h
 * @brief Collection of behaviors that use the bump sensor.
 * @details These behaviors include obstacle avoidance and navigation.
 *
 * @since September 28, 2011
 * @author James McLurkin
 */

#ifndef BUMPBEHAVIORS_H_
#define BUMPBEHAVIORS_H_



#define BUMPMOVE_REFLECT_DISTANCE		110
#define BUMPMOVE_REFLECT_DIST_SHORT		20

// obstacle avoidance behaviors

/**
 *  @brief Uses IR to avoid all obstacles, including robots. Can be invalidated.
 *
 *  @param behPtr
 *  @param tv desired translational velocity
 *  @param invalidate
 *  @returns a Beh* for obstacle avoidance
 */
Beh* behIRObstacleAvoid(Beh* behPtr, int32 tv, boolean invalidate);


/**
 * @brief If bumped, robot will move slightly forward out of way
 *
 * @param behPtr pointer for behavior to be modified
 * @returns updated behPtr
 */
Beh* behBumpMoveOutOfWay(Beh* behPtr);


/**
 *  @brief Uses IR to avoid all obstacles, except robots. Can be invalidated.
 *
 *  @param behPtr
 *  @param tv desired translational velocity
 *  @param nbrListPtr for list of neighbors
 *  @param invalidate
 *  @returns a Beh* for obstacle avoidance
 */
Beh* behIRObstacleAvoid_ExcludeRobots(Beh* behPtr, int32 tv, NbrList* nbrListPtr, boolean invalidate);


/**
 *  @brief Simple behavior: Uses bump sensors to determine if robot should avoid an obstacle.
 *
 *  @details If obstacle is detected with bump sensors, the robot turns in place.
 *  After the bump sensor turns off, the robot moves forward forwardDist mm.
 *  If the robot does not bump anything, this behavior becomes inactive.
 *  @param behPtr behavior that is updated by this function
 *  @param tv desired translational velocity
 *  @param forwardDist the desired distance (in mm) the robot should move once a bump sensor turns off
 *  @returns updated behPtr
 */
Beh* behBumpAvoid(Beh* behPtr, int32 tv, int32 forwardDist);

Beh* behBumpAvoidWithRvHistory(Beh* behPtr, int32 tv, int32 forwardDist, int32 previousRv);

/**
 * @brief Turn around if it his running into something
 *
 * @param behPtr behavior that is updated by this function
 * @param tv desired translational velocity
 * @param reflectDist
 * @returns updated behPtr
 */
Beh* behBumpReflect(Beh* behPtr, int32 tv , int32 reflectDist);


/**
 * @brief If the front bump sensors are activated, the robot will rotate.
 *
 * This is obtained by turning off translational velocity and setting rotational velocity to
 * the input rotate_rv in the behPtr.
 * @param behPtr pointer for the behavior to update
 * @param rotate_rv desired rotational velocity if the robot has encountered an obstacle
 * @returns void
 */
void behBumpReflectDemo(Beh* behPtr, int32 rotate_rv);


/**
 * @brief If the front bump sensors are activated, the robot will backup
 * @param behPtr pointer for the behavior to update
 * @param tv desired translational velocity
 * @returns updated behPtr
 */
Beh* behBumpBackoff(Beh* behPtr, int32 tv);


/**
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
Beh* behBumpNavigate(Beh* behPtr, uint8 headingRobot);

static uint8 getBumptypeFront(uint8 bumpSensorBits);


#endif /* BUMPBEHAVIORS_H_ */
