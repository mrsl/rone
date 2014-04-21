/**
 * @file Navigation-midangle.h
 *
 * @brief Used to steer a robot through a network of robots without crashing.
 * @details
 * @since Sep 23, 2011
 * @author Golnaz Habibi
 */

#ifndef NAVIGATION_MIDANGLE_H_
#define NAVIGATION_MIDANGLE_H_


/**
 *  @brief
 *
 *  @param nbrListGuidesPtr
 *  @param msgPtr
 *  @returns void
 */
void nbrListPickGuides(NbrList* nbrListGuidesPtr, BroadcastMessage* msgPtr);


/**
 *  @brief
 *
 *  @param behPtr for behavior that is updated by this function
 *  @param nbrListPtr for list of neighbors
 *  @param tv desired translational velocity
 *  @returns updated behPtr
 */
Beh* midAngleNavigation(Beh* behPtr, NbrList* nbrListPtr, int32 tv);

//TODO: implement or delete?
Beh* broadcastMessageNavigate(Beh* behPtr, BroadcastMessage* msgPtr, int32 tv);


#endif /* NAVIGATION_MIDANGLE_H_ */
