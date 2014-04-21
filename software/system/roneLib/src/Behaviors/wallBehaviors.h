/*
 * WallMotion.h
 *
 *  Created on: Aug 1, 2013
 *      Author: jamesm
 */

#ifndef WALLMOTION_H_
#define WALLMOTION_H_

#define WALL_FOLLOW_LEFT	1
#define WALL_FOLLOW_RIGHT	-1

Beh* behWallMove(Beh* behPtr, int32 tv, int8 direction);
void splitBearingGroup (int16 * bearGroups);

#endif /* WALLMOTION_H_ */
