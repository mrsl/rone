/*
 * tileMotion.h
 *
 *  Created on: Mar 31, 2014
 *      Author: jamesm
 */

#ifndef TILEMOTION_H_
#define TILEMOTION_H_

// User motion command inputs
#define TILEMOTION_IDLE				0
#define TILEMOTION_FORWARD			1
#define TILEMOTION_ROTATE_RIGHT		2
#define TILEMOTION_ROTATE_LEFT		3

void tileMotionInit(void);
void tileMotion(uint8 motionCommand);
boolean tileMotionDone(void);
TileInfo* tileMotionGetTile(void);
void tileMotionWaitUntilDone(void);


#endif /* TILEMOTION_H_ */
