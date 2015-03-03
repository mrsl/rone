/**
 * @file basicBehaviors.h
 *
 * @brief Basic swarm functionality behaviors.
 * @details This includes remote control, flocking, wall following, and follow-the-leader.
 *
 * \internal
 * This section is best for basic *swarm* behaviors.
 * Other things that would fit here is disperse, cluster, sort
 * \endinternal
 *
 * @since September 28, 2011
 * @author James McLurkin
 */

#ifndef BASICBEHAVIORS_H_
#define BASICBEHAVIORS_H_

//#define RVTV_RATIO						((MILLIRAD_PI / 4) / 80)
#define RVTV_RATIO						((MILLIRAD_PI / 4) / 100)


Beh* behFlockNormalToLeader(Beh* behPtr, Nbr* nbrPtr,int32 tv);


Beh* behRotateAroundLeader(Beh* behPtr, int32 leaderDist, int32 leaderBearing, int32 tv);

int32 behAngleNormalToLeader(int32 bearing);

/**
 * @brief Robot rotates in place to face neighbor.
 *
 * @param behPtr behavior that is update by this function
 * @param nbrPtr pointer to neighbor to face
 * @returns updated behPtr
 */
Beh* behFaceNbr(Beh* behPtr, Nbr* nbrPtr);


/**
 *	@brief Stop the robot and do nothing.
 *
 *	@param	behPtr behavior that is updated by this function
 *	@returns updated behPtr
 */
Beh* behStop(Beh* behPtr);


Beh* behAlignWithWall(Beh* behPtr, uint16 bearing, uint16 bits);


/**
 *	@brief Robots flock together.
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrListPtr list of neighbors
 *	@param	tv desired translational velocity
 *	@returns updated behPtr
 */
Beh* behFlock_gain(Beh* behPtr, NbrList* nbrListPtr, int32 tv, int32 rv_gain);	//### Flock with customized rv_gain
Beh* behFlock(Beh* behPtr, NbrList* nbrListPtr, int32 tv);


Beh* behRotateAroundLeader(Beh* behPtr, int32 leaderDist, int32 leaderBearing, int32 tv);



/**
 *	@brief Computes the flock angle
 *
 *	@param	nbrListPtr list of neighbors
 *	@returns flock angle or 0 if there are no neighbors
 */
int32 behFlockAngle(NbrList* nbrListPtr);


/**
 *	@brief Robot orbits the input neighbor.
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrPtr neighbor to be orbited
 *	@param	tv desired translational velocity
 *	@returns updated behPtr
 */
Beh* behOrbit(Beh* behPtr, Nbr* nbrPtr, int32 tv);


/**
 *	@brief Robot orbits the input neighbor within given range (decide clockwise or counterclockwise).
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrPtr neighbor to be orbited
 *	@param	tv desired translational velocity
 *	@param  range desired orbiting range
 *	@returns updated behPtr
 */
Beh* behOrbitRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range);

/**
 *	@brief Robot orbits the input neighbor within given range (clockwise).
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrPtr neighbor to be orbited
 *	@param	tv desired translational velocity
 *	@param  range desired orbiting range
 *	@returns updated behPtr
 */
Beh* behOrbitCWRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range);

/**
 *	@brief Robot orbits the input neighbor within given range (counterclockwise).
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrPtr neighbor to be orbited
 *	@param	tv desired translational velocity
 *	@param  range desired orbiting range
 *	@returns updated behPtr
 */
Beh* behOrbitCCWRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range);

/**
 *	@brief Follow robot whose ID is larger than yours.
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrListPtr list of neighbors
 *	@param	tv desired translational velocity
 *	@returns updated behPtr
 */
Beh* behFollowPredesessor(Beh* behPtr, NbrList* nbrListPtr, int32 tv, int32 range);


//TODO Implement or delete?
//Beh* behFollowPredesessorNew(Beh* behPtr, NbrList* nbrListPtr, uint8* isLeader, int32 tv, BroadcastMessage msg);


//TODO: update brief to be more specific
/**
 *	@brief Robot follows a wall.
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrListPtr for list of neighbors
 *	@param	tv desired translational velocity
 *	@returns updated behPtr
 */
Beh* behWallFollow(Beh* behPtr, NbrList* nbrListPtr, uint32 tv);


/**
 *	@brief Moves robot towards other robots broadcasting a message.
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param nbrListPtr list of neighbors
 *	@param tv desired translational velocity
 *	@param msgPtr
 *	@returns updated behPtr
 */
Beh* behClusterBroadcast(Beh* behPtr, NbrList* nbrListPtr, int32 tv, BroadcastMessage* msgPtr);

/**
 * @brief Rotate the robot to a given bearing using a proportional controller.
 *
 * @param behPtr behavior that is updated by this function
 * @param angle the robot should rotate to
 * @param gain positive value governing how quickly the robot adjusts
 * @returns updated behPtr
 */
Beh* rvBearingController(Beh* behPtr, int32 angle, int32 gain);


/**
 * @brief Simple Behavior: move straight forward (no turning)
 *
 * @param behPtr behavior that is updated by this function
 * @param tv desired translational velocity
 * @returns updated behPtr
 */
Beh* behMoveForward(Beh* behPtr, int32 tv);


/**
 * @brief Rotate the robot to a given bearing using a proportional controller.
 *
 * @param behPtr behavior that is updated by this function
 * @param angle the robot should rotate to
 *
 * @returns updated behPtr
 */
Beh* behBearingController(Beh* behPtr, int32 angle);


//TODO fix code overlap with rvBearingController
Beh* behBearingControllerGain(Beh* behPtr, int32 angle, int32 rvGain);

/**
 * @brief Stops the robot if it is sitting on it's butt. (x
 *
 * @param behPtr behavior that is updated by this function
 * @returns updated behPtr
 */
Beh* behChargeStop(Beh* behPtr);
void behChargeStopLights(Beh* behPtr);


/**
 * @brief Moves robot towards specified neighbor.
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr for neighbor
 * @param tv desired translational velocity
 * @returns updated behPtr
 */
Beh* behMoveToNbr(Beh* behPtr, Nbr* nbrPtr, int32 tv);

/**
 * @brief Moves robot towards specified neighbor until given range.
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr for neighbor
 * @param tv desired translational velocity
 * @param range given minimum distance
 * @returns updated behPtr
 */
Beh* behMoveToNbrRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range);

/**
 * @brief Moves robot towards specified neighbor until given range with P control.
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr for neighbor
 * @param range given minimum distance
 * @returns updated behPtr
 */
Beh* behMoveToNbrPRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range);

/**
 * @brief Moves robot between specified neighbors.
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr1 for neighbor
 * @param nbrPtr2 for neighbor
 * @param tv desired translational velocity
 * @returns updated behPtr
 */
Beh* behMoveBetweenNbr(Beh* behPtr, Nbr* nbrPtr1, Nbr* nbrPtr2, int32 tv);

/**
 * @brief Moves robot between specified neighbors (range based).
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr1 for neighbor
 * @param nbrPtr2 for neighbor
 * @param tv desired translational velocity
 * @returns updated behPtr
 */
Beh* behMoveBetweenNbrRange(Beh* behPtr, Nbr* nbrPtr1, Nbr* nbrPtr2, int32 tv);

/**
 * @brief Moves robot away from specified neighbor
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr for neighbor
 * @param tv desired translational velocity
 * @returns updated behPtr
 */
Beh* behMoveFromNbr(Beh* behPtr, Nbr* nbrPtr, int32 tv);

/**
 * @brief Moves robot away from specified neighbor until given range
 *
 * @param behPtr behavior that is updated by this function
 * @param nbrPtr for neighbor
 * @param tv desired translational velocity
 * @param range given minimum distance
 * @returns updated behPtr
 */
Beh* behMoveFromNbrRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range);

//TODO: implement or delete?
Beh* behBumpRotate(Beh* behPtr, int32 tv);

#endif
