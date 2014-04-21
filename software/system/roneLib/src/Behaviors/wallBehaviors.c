/*
 * @file WallMotion.c
 *
 * @brief Wall following and orientating behaviors
 * This includes wall following, and others
 *
 * @since Aug 01, 2013
 * @author: jamesm
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define WALL_FOLLOW_RV	500

Beh* behWallMove(Beh* behPtr, int32 tv, int8 direction) {
	NbrList nbrList;
	uint8 obstacleBitsGroup;
	uint8 obstacleBitsCount;
	int16 obstacleBearing, obstacleBearingError;

	// find the normal to the wall from the IR system
	nbrListCreate(&nbrList);
	obstacleExcludeNbrs(&nbrList, &obstacleBitsGroup, &obstacleBitsCount, &obstacleBearing);

	if(obstacleBitsGroup == 0) {
		// we have no obstacle bits.  Nothing to do here.
		if (direction == WALL_FOLLOW_LEFT) {
			// point your left side towards the wall
			behSetRv(behPtr, WALL_FOLLOW_RV);
		} else {
			// point your right side towards the wall
			behSetRv(behPtr, -WALL_FOLLOW_RV);
		}
	} else {
		// add pi/2 to the obsracle heading
		if (direction == WALL_FOLLOW_LEFT) {
			// point your left side towards the wall
			obstacleBearingError = normalizeAngleMilliRad2(obstacleBearing - MILLIRAD_HALF_PI);
		} else {
			// point your right side towards the wall
			obstacleBearingError = normalizeAngleMilliRad2(obstacleBearing + MILLIRAD_HALF_PI);
		}
		behBearingController(behPtr, obstacleBearingError);
	}
	behSetTv(behPtr, tv);
	return behPtr;
}


void splitBearingGroup (int16 * bearGroups){
	int8 obsBits = irObstaclesGetBits();
	int8 bitMask;
	int8 tempBit = 0;
	int8 countGroup = 0;
	int16 obstacleBearing;
	bearGroups[0] = 0;
	bearGroups[1] = 0;
	bearGroups[2] = 0;
	bearGroups[3] = 0;

	//cprintf("bearingBits = %d", obsBits);

	if (obsBits != 0){
		for (bitMask = 1; bitMask != 0; bitMask = bitMask << 1) {
			if(obsBits & bitMask){
				tempBit |= bitMask;
			}
			else{
				if (tempBit != 0){
					obstacleBearing = angleFromBitVectorOffset(tempBit);
					bearGroups[countGroup] = normalizeAngleMilliRad(obstacleBearing);
					tempBit = 0;
					countGroup++;
				}

			}
		}
	}

}
