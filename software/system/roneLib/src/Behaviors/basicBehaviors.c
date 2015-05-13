/*
 * @file basicBehaviors.c
 *
 * @brief Basic swarm functionality behaviors.
 * This includes remote control, flocking, wall following, follow-the-leader
 *
 * \internal
 * This section is best for basic *swarm* behaviors.
 * Other things that would fit here is disperse, cluster, sort
 * \endinternal
 *
 * @since Sep 28, 2011
 * @author: jamesm
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define OBS_FRONT_ANGLE				0
#define OBS_FRONT_LEFT_ANGLE		0.3927
#define OBS_LEFT_ANGLE				1.1781
#define OBS_FRONT_RIGHT_ANGLE		5.8905
#define OBS_RIGHT_ANGLE				5.1051



#define FRONT_BUMP					0
#define FRONT_LEFT_BUMP				1
#define LEFT_BUMP					2
#define	FRONT_RIGHT_BUMP			3
#define RIGHT_BUMP					4

#define	RV_GAIN						50
#define TV_GAIN				        1


Beh* behStop(Beh* behPtr) {
	behPtr->tv = 0;
	behPtr->rv = 0;
	behPtr->active = FALSE;
	return behPtr;
}

Beh* behMoveForward(Beh* behPtr, int32 tv) {
	behPtr->tv = tv;
	behPtr->rv = 0;
	behPtr->active = TRUE;
	return behPtr;
}



Beh* rvBearingController(Beh* behPtr, int32 angle, int32 gain) {
	int32 q = smallestAngleDifference(0, angle);
	behPtr->rv = -q * gain / 100;
	behPtr->active = TRUE;
	return behPtr;
}


Beh* behBearingController(Beh* behPtr, int32 angle) {
	int32 q = smallestAngleDifference(0, angle);
	behPtr->rv = -q * RV_GAIN / 100;
	behPtr->active = TRUE;
	return behPtr;
}

//TODO: Doesn't this do the same thing as rvBearingController with a different parameter name?
Beh* behBearingControllerGain(Beh* behPtr, int32 angle, int32 rvGain) {
	int32 q = smallestAngleDifference(0, angle);
	behPtr->rv = -q * rvGain / 100;
	behPtr->active = TRUE;
	return behPtr;
}

#define CHARGE_STOP_ACCEL_VALUE		500
#define CHARGE_STOP_IIR_ALPHA		5
#define CHARGE_STOP_COUNTER_MAX		16


Beh* behChargeStop(Beh* behPtr) {
	static int32 accelAvg = 0;
	static int32 chargeStopCounter = 0;
	int32 accelX = accelerometerGetValue(ACCELEROMETER_X);

	*behPtr = behInactive;
	accelAvg = filterIIR(accelX, accelAvg, CHARGE_STOP_IIR_ALPHA);
	//cprintf("accelAvg %d count %d ", accelAvg, chargeStopCounter);
	if (abs(accelAvg) > CHARGE_STOP_ACCEL_VALUE) {
		if (chargeStopCounter < CHARGE_STOP_COUNTER_MAX * 2) {
			chargeStopCounter++;
		}
	} else {
		if (chargeStopCounter > 0) {
			chargeStopCounter-=2;
		}
	}
	if (chargeStopCounter > CHARGE_STOP_COUNTER_MAX) {
		behPtr->active = TRUE;
		//cprintf("stop");
	}
	//cprintf("\n");
	return behPtr;
}

void behChargeStopLights(Beh* behPtr) {
	if(behIsActive(behPtr)) {
		// show different lights for charge statis
		//TODO: right now, we don't have access to the charge status, so we just show red circle
		ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
	}
}



// this is for a neighbor period of 150ms
//#define MOVE_TO_NEIGHBOR_RV_GAIN	120

// this is for a neighbor period of 600ms
#define MOVE_TO_NEIGHBOR_RV_GAIN	60


Beh* behMoveToNbr(Beh* behPtr, Nbr* nbrPtr, int32 tv) {
	if (nbrPtr) {
		rvBearingController(behPtr, nbrPtr->bearing, MOVE_TO_NEIGHBOR_RV_GAIN);
		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}

Beh* behFaceNbr(Beh* behPtr, Nbr* nbrPtr) {
	if (nbrPtr) {
		int32 q = smallestAngleDifference(0, nbrPtr->bearing);
		int32 rv = -q * 50 / 100;
		if(abs(rv)<100){
			behPtr->rv = 0;
		}
		behPtr->tv = 0;
		behPtr->rv = rv;
		behPtr->active = TRUE;
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}

Beh* behMoveToNbrRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range) {	//### move to distance
	if (nbrPtr) {
		if(nbrPtr->range > range) {
			int32 theta=nbrPtr->bearing;
			rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);
			behPtr->tv = tv;
			behPtr->active = TRUE;
		} else {
			behPtr->active = FALSE;
		}
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}

#define MOVE_TO_NBR_VEL_BOUND		25
//#define MOVE_TO_NBR_KP			500
#define MOVE_TO_NBR_KP				100
#define MOVE_TO_NBR_KI				4
#define MOVE_TO_NBR_KI_NEG_GAIN		2
#define MOVE_TO_NBR_KI_BOUND		10000


Beh* behMoveToNbrPRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range) {	//### move to distance
	static int32 iTermState = 0;

	if (nbrPtr) {
		int32 e = nbrPtr->range - range;
		if (e > 0) {
			iTermState += e;
		} else {
			iTermState += MOVE_TO_NBR_KI_NEG_GAIN * e;
		}
		iTermState = bound(iTermState, -MOVE_TO_NBR_KI_BOUND, MOVE_TO_NBR_KI_BOUND);
		int32 iTerm = MOVE_TO_NBR_KI * iTermState / 1000;
		int32 pTerm = MOVE_TO_NBR_KP * e / 1000;
		int32 tvTemp = tv + pTerm + iTerm;
		tvTemp = bound(tvTemp, tv * (100 - MOVE_TO_NBR_VEL_BOUND) / 100, tv * (100 + MOVE_TO_NBR_VEL_BOUND) / 100);

		int32 theta=nbrPtr->bearing;
		rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);

		//cprintf("rng=%d, goal_rng=%d, e=%d, istate=%d, iterm=%d, pterm=%d\n",nbrPtr->range, range, e, iTermState, iTerm, pTerm);
		behPtr->active = TRUE;
		if(tvTemp>0) {
			behPtr->tv = tvTemp;
		} else {
			behPtr->tv=0;
		}
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}

Beh* behMoveBetweenNbr(Beh* behPtr, Nbr* nbrPtr1, Nbr* nbrPtr2, int32 tv) {	//### move between neighbors (angle based) for bubble sort
	if (nbrPtr1 && nbrPtr2) {
		int32 theta=averageAngles(nbrPtr1->bearing,nbrPtr2->bearing);
		rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);
		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}


Beh* behAlignWithWall(Beh* behPtr, uint16 bearing, uint16 bits){
	uint32 newRV = 0;
	uint32 bearingInside = irObstaclesGetBearing();
	cprintf("startingRV %d bearing %d newRVstart %d ",behPtr->rv, bearing, newRV);
	//if wall is sensed
	if(bits != 0){
		bearing = bearing - 3141;
		bearingInside = bearingInside - 3141;
		cprintf("bearing after math %d ",bearing);
		//if wall is to the right
		if(bearing >= 0){
			newRV = (bearing - 1571)/ 1.5;
			cprintf(" rv = %d\n",newRV);
			behSetRv(behPtr,newRV);

			//if wall is normal to robot (within 10 degrees), stop rotating
			if (abs(newRV) < 100){
				behSetTvRv(behPtr, 0, 0);
			}
		}//end right wall

		//if wall is to the left
		else{
				newRV = (bearing + 1571) / 1.5;
				cprintf(" rv = %d\n",newRV);
				behSetTvRv(behPtr, 0, newRV);
				//if wall is normal to robot, stop rotating
				if (abs(newRV) < 100){
					behSetTvRv(behPtr, 0, 0);
				}
		} // end left wall
	} // end if wall
	else{
			behSetTvRv(behPtr, 0, 0);
	} //End else no wall
	cprintf("RV: %d active: %d\n",newRV,behPtr->active);
	return behPtr;
}
Beh* behMoveBetweenNbrRange(Beh* behPtr, Nbr* nbrPtr1, Nbr* nbrPtr2, int32 tv) {	//### move between neighbors (range based) for bubble sort
	static uint8 stable=0;
	if (nbrPtr1 && nbrPtr2) {
		int32 C=(nbrPtr1->range * cosMilliRad(nbrPtr1->bearing) + nbrPtr2->range * cosMilliRad(nbrPtr2->bearing))/32768;
		int32 S=(nbrPtr1->range * sinMilliRad(nbrPtr1->bearing) + nbrPtr2->range * sinMilliRad(nbrPtr2->bearing))/32768;
		int32 dist2=(C*C+S*S)/4;
		int32 theta;
		if((stable==0 && dist2>1600) || (stable==1 && dist2>19600)) {	//### move to middle if distance is large
			if(nbrPtr1->range * 2 > nbrPtr2->range * 3) {
				theta=nbrPtr1->bearing;
			} else if(nbrPtr2->range * 2 > nbrPtr1->range * 3) {
				theta=nbrPtr2->bearing;
			} else {
				theta=normalizeAngleMilliRad2(atan2MilliRad(S,C));
			}
			rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);
			if(theta<523&&theta>-523) {	//### between minus and plus 30 degrees
				behPtr->tv = tv;
			} else {
				behPtr->tv = 0;
			}
			stable=0;
			behPtr->active = TRUE;
		} else {
			theta=smallestAngleDifference(nbrPtr1->bearing,nbrPtr2->bearing);
			if(theta<2790 && theta>-2790) {	//### move to middle if angle is large (more than 20 degrees from straight)
				theta=normalizeAngleMilliRad2(atan2MilliRad(S,C));
				rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);
				if(theta<523&&theta>-523) {	//### between minus and plus 30 degrees
					behPtr->tv = tv;
				} else {
					behPtr->tv = 0;
				}
				stable=0;
				behPtr->active = TRUE;
			} else {	//### already there
	//			theta=averageAngles(nbrPtr1->orientation,nbrPtr2->orientation);
	//			rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);
	//			behPtr->tv = 0;
				stable=1;
				behPtr->active = FALSE;
			}

		}
		cprintf("~MBNR\t%d/%d\t%d@%d\t%d@%d\t%d\t%d(%d)\n",nbrPtr1->ID,nbrPtr2->ID,nbrPtr1->range,nbrPtr1->bearing,nbrPtr2->range,nbrPtr2->bearing,theta,dist2,stable);
		//cprintf("C=%d\tS=%d\tC1=%d\tS2=%d\n",C,S,cosMilliRad(nbrPtr1->bearing),sinMilliRad(nbrPtr2->bearing));
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}

Beh* behMoveFromNbr(Beh* behPtr, Nbr* nbrPtr, int32 tv) {
	if (nbrPtr) {
		rvBearingController(behPtr, normalizeAngleMilliRad2(nbrPtr->bearing + MILLIRAD_PI), MOVE_TO_NEIGHBOR_RV_GAIN);
		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}

Beh* behMoveFromNbrRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range) {	//### move away from neighbor
	if (nbrPtr) {
		if(nbrPtr->range < range) {
			int32 theta=normalizeAngleMilliRad2(nbrPtr->bearing + MILLIRAD_PI);
			rvBearingController(behPtr, theta, MOVE_TO_NEIGHBOR_RV_GAIN);
			behPtr->tv = tv;
			behPtr->active = TRUE;
		} else {
			behPtr->active = FALSE;
		}
	} else {
		behPtr->active = FALSE;
	}
	return behPtr;
}


#define FLOCK_RV_GAIN	70

//TODO: What is flock angle in relation to the other robots?

int32 behFlockAngle(NbrList* nbrListPtr) {
	int32 i, x, y, alpha;
	Nbr* nbrPtr;

	x = 0;
	y = 0;
	for (i = 0; i < nbrListPtr->size; ++i) {
		nbrPtr = nbrListPtr->nbrs[i];
		if(nbrPtr->orientationValid) {
			alpha = normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation);
			x += cosMilliRad(alpha);
			y += sinMilliRad(alpha);
		}
	}
	if (nbrListPtr->size > 0) {
		alpha = normalizeAngleMilliRad2(atan2MilliRad(y, x));
	} else {
		alpha = 0;
	}
	return alpha;
}

/*int32 behFlockAngleNormalToLeader(NbrList* nbrListPtr) {
	int32 i, x, y, alpha;
	Nbr* nbrPtr;

	x = 0;
	y = 0;
	for (i = 0; i < nbrListPtr->size; ++i) {
		nbrPtr = nbrListPtr->nbrs[i];
		if(nbrPtr->orientationValid) {
			alpha = normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation);
			x += cosMilliRad(alpha);
			y += sinMilliRad(alpha);
		}
	}
	if (nbrListPtr->size > 0) {
		alpha = normalizeAngleMilliRad2(atan2MilliRad(y, x));
	} else {
		alpha = 0;
	}
	return alpha + MILLIRAD_DEG_90;
}

*/
int32 behFlockAngleNormalToLeader( Nbr* nbrPtr){
	int32 alpha;

	alpha =nbrPtr->bearing;

	if (alpha > 0)
		return normalizeAngleMilliRad2(alpha - (int32)MILLIRAD_DEG_90);
		else

		return normalizeAngleMilliRad2(alpha + (int32)MILLIRAD_DEG_90);
}


int32 behAngleNormalToLeader(int32 bearing){
	//int32 alpha;

	//alpha =nbrPtr->bearing;
	if (bearing > 0)
	return normalizeAngleMilliRad2(bearing - (int32)MILLIRAD_DEG_90);
	else

	return normalizeAngleMilliRad2(bearing + (int32)MILLIRAD_DEG_90);

}


Beh* behFlockNormalToLeader(Beh* behPtr, Nbr* nbrPtr, int32 tv) {
	int32 alpha;

	if (nbrPtr!= NULL) {
		alpha = behFlockAngleNormalToLeader(nbrPtr);
		rvBearingController(behPtr, alpha, FLOCK_RV_GAIN);

		//behPtr->rv = alpha * ROTATION-GAIN;

		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->tv = tv;
		behPtr->rv = 0;
		behPtr->active = TRUE;
	}
	return behPtr;
}

Beh* behRotateAroundLeader(Beh* behPtr, int32 leaderDist, int32 leaderBearing, int32 tv_gain)
 {
	//int32 alpha;

	//if (nbrListPtr->size > 0) {
		int32 alpha = behAngleNormalToLeader(leaderBearing);
		//FLOCK GAIN = 150
		rvBearingController(behPtr, alpha, 160);
		//used to be leaderDist/10
		behPtr->tv = tv_gain* leaderDist/5;

		//behPtr->rv = alpha * ROTATION-GAIN;


		behPtr->active = TRUE;
//	} else {
//		behPtr->tv = tv;
//		behPtr->rv = 0;
//		behPtr->active = TRUE;
//	}
	return behPtr;
}

//### Flock with customized rv_gain
Beh* behFlock_gain(Beh* behPtr, NbrList* nbrListPtr, int32 tv, int32 rv_gain) {
	int32 alpha;

	if (nbrListPtr->size > 0) {
		alpha = behFlockAngle(nbrListPtr);
		rvBearingController(behPtr, alpha, rv_gain);
		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->tv = tv;
		behPtr->rv = 0;
		behPtr->active = TRUE;
	}
	return behPtr;
}

Beh* behFlock(Beh* behPtr, NbrList* nbrListPtr, int32 tv) {
	return behFlock_gain(behPtr, nbrListPtr, tv, FLOCK_RV_GAIN);
}


Beh* behClusterBroadcast(Beh* behPtr, NbrList* nbrListPtr, int32 tv, BroadcastMessage* msgPtr) {
	NbrList nbrListParents;
	if (msgPtr) {
		nbrListGetParents(&nbrListParents, nbrListPtr, msgPtr);
		behBearingController(behPtr, nbrListAverageBearing(&nbrListParents));
		behSetTv(behPtr, tv);
	}
	return behPtr;
}

Beh* behClusterBroadcastStop(Beh* behPtr, NbrList* nbrListPtr, BroadcastMessage* msgPtr, uint16 minRange) {
	NbrList nbrListParents;
	if (msgPtr) {
		uint8 i;
		nbrListGetParents(&nbrListParents, nbrListPtr, msgPtr);
		for (i = 0; i < nbrListGetSize(nbrListPtr); ++i) {
			Nbr* nbrPtr = nbrListGetNbr(nbrListPtr, i);
			if(nbrGetRange(nbrPtr) < minRange) {
				behSetTv(behPtr, 0);
				break;
			}
		}
	}
	return behPtr;
}



#define ORBIT_RV_GAIN	40
// should be MILLIRAD_HALF_PI, but need to add lead compensation
#define ORBIT_MAGIC_ANGLE 970

Beh* behOrbit(Beh* behPtr, Nbr* nbrPtr, int32 tv) {
	int32 theta;
	theta = normalizeAngleMilliRad(nbrGetBearing(nbrPtr) + ORBIT_MAGIC_ANGLE);
	rvBearingController(behPtr, theta, ORBIT_RV_GAIN);
	behSetTv(behPtr, tv);
	return behPtr;
}

Beh* behOrbitRangeRaw(Beh* behPtr, int32 bearing, int32 range, int32 tv, uint16 desiredRange) {
	int32 theta = normalizeAngleMilliRad(
		bearing + 1570 - bound((range - desiredRange),-1000, 1000)
	);

	rvBearingController(behPtr, theta, ORBIT_RV_GAIN);
	behPtr->tv = tv;
	behPtr->active = TRUE;

	return behPtr;
}

Beh* behOrbitRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range) {	//### orbit
	static int oldID=0;
	static uint8 CCW=0;
	int32 theta;
	if (nbrPtr) {
		if(nbrPtr->ID==oldID) {
			if(CCW==0) {	//### CW
				theta = normalizeAngleMilliRad((int32)nbrPtr->bearing + 1570 - bound((nbrPtr->range - range),-1000,1000));
			} else {	//### CCW
				theta = normalizeAngleMilliRad((int32)nbrPtr->bearing - 1570 + bound((nbrPtr->range - range),-1000,1000));
			}
		} else {
			if(nbrPtr->bearing<0) {	//### CW
				theta = normalizeAngleMilliRad((int32)nbrPtr->bearing + 1570 - bound((nbrPtr->range - range),-1000,1000));
				CCW=0;
			} else {	//### CCW
				theta = normalizeAngleMilliRad((int32)nbrPtr->bearing - 1570 + bound((nbrPtr->range - range),-1000,1000));
				CCW=1;
			}
		}
		rvBearingController(behPtr, theta, ORBIT_RV_GAIN);
		behPtr->tv = tv;
		behPtr->active = TRUE;
		cprintf("~OR\t%d/%d\t%d@%d\t%d@%d\t%d\t%d(%d)\n",nbrPtr->ID,0,nbrPtr->range,nbrPtr->bearing,0,0,theta,40000,CCW);
	} else {
		behPtr->active = FALSE;
		oldID=0;
		cprintf("~ORNL\t%d/%d\t%d@%d\t%d@%d\t%d\t%d(%d)\n",0,0,0,0,0,0,0,0,0);
	}

	return behPtr;
}

Beh* behOrbitCWRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range) {	//### orbit
	if (nbrPtr) {
		int32 theta = normalizeAngleMilliRad((int32)nbrPtr->bearing + 1570 - bound((nbrPtr->range - range),-1570,1570));
		rvBearingController(behPtr, theta, ORBIT_RV_GAIN);

		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->active = FALSE;
	}

	return behPtr;
}

Beh* behOrbitCCWRange(Beh* behPtr, Nbr* nbrPtr, int32 tv, uint16 range) {	//### orbit CCW
	if (nbrPtr) {
		int32 theta = normalizeAngleMilliRad((int32)nbrPtr->bearing - 1570 + bound((nbrPtr->range - range),-1570,1570));
		rvBearingController(behPtr, theta, ORBIT_RV_GAIN);

		behPtr->tv = tv;
		behPtr->active = TRUE;
	} else {
		behPtr->active = FALSE;
	}

	return behPtr;
}


//TODO depricated - remove it
//Beh* behFollowPredesessorNew(Beh* behPtr, NbrList* nbrListPtr, uint8* isLeader, int32 tv, BroadcastMessage msg) {
//	int32 i, x, y, avgOrientation, theta;
//	Nbr* nbrPtr;
//	Nbr* leaderNbrPtr;
//	uint8 leaderID = 0;
//
//	leaderNbrPtr = NULL;
//	for (i = 0; i < nbrListPtr->size; ++i) {
//		nbrPtr = nbrListPtr->nbrs[i];
//		if((nbrPtr->ID < roneID) && (nbrPtr->ID > leaderID)) {
//			leaderID = nbrPtr->ID;
//			leaderNbrPtr = nbrPtr;
//		}
//	}
//	if (leaderNbrPtr) {
//		behMoveToNbr(behPtr, leaderNbrPtr, tv);
//	} else {
//		*isLeader = 1;
//		//*behPtr = behInactive;
//	}
//	return behPtr;
//}

//TODO: Not in h file...but should it be? Seems like it could be useful for other users
/*
 *	@brief Follow robot whose ID is larger than yours
 *
 *	@param	behPtr behavior that is updated by this function
 *	@param	nbrListPtr list of neighbors
 *	@param	tv desired translational velocity
 *	@returns updated behPtr
 */
Beh* behFollowPredesessor(Beh* behPtr, NbrList* nbrListPtr, int32 tv, int32 range) {
	int32 i, x, y, avgOrientation, theta;
	Nbr* nbrPtr;
	Nbr* leaderNbrPtr;
	uint8 leaderID = 0;

	leaderNbrPtr = NULL;
	for (i = 0; i < nbrListGetSize(nbrListPtr); ++i) {
		nbrPtr = nbrListGetNbr(nbrListPtr, i);
		if((nbrPtr->ID < roneID) && (nbrPtr->ID > leaderID)) {
			leaderID = nbrPtr->ID;
			leaderNbrPtr = nbrPtr;
		}
	}
	if (leaderNbrPtr) {
		behMoveToNbrPRange(behPtr, leaderNbrPtr, tv, range);
	} else {
		*behPtr = behInactive;
	}
	return behPtr;
}


Beh* behWallFollow(Beh* behPtr, NbrList* nbrListPtr, uint32 tv){
	static uint8 prevBumpType;
	static uint8 prevPrevBumpType;
	uint8 irBits;
	uint8 bumpType;
	uint32 bumpTheta = 0;
	uint32 thetaDelta = 0;

	irBits = irObstaclesGetBits();

	if ((irBits & IR_COMMS_RECEIVER_FRONT_LEFT_BIT) && (irBits & IR_COMMS_RECEIVER_FRONT_RIGHT_BIT)) {
		// obstacle in front
		bumpType = FRONT_BUMP;
		bumpTheta = OBS_FRONT_ANGLE * 1000;
	} else if (irBits & IR_COMMS_RECEIVER_FRONT_LEFT_BIT) {
		// obstacle in front left
		bumpType = FRONT_LEFT_BUMP;
		bumpTheta = OBS_FRONT_LEFT_ANGLE * 1000;
	} else if (irBits & IR_COMMS_RECEIVER_LEFT_FRONT_BIT) {
		// obstacle in left
		bumpType = LEFT_BUMP;
		bumpTheta = OBS_LEFT_ANGLE * 1000;
	} else if (irBits & IR_COMMS_RECEIVER_FRONT_RIGHT_BIT) {
		// obstacle in front right
		bumpType = FRONT_RIGHT_BUMP;
		bumpTheta = OBS_FRONT_RIGHT_ANGLE * 1000;
	} else if (irBits & IR_COMMS_RECEIVER_RIGHT_FRONT_BIT) {
		// obstacle in right
		bumpType = RIGHT_BUMP;
		bumpTheta = OBS_RIGHT_ANGLE * 1000;
	}

	prevBumpType = bumpType;
	prevPrevBumpType = bumpType;

	if (bumpTheta > 0) {
		if (bumpType == FRONT_LEFT_BUMP) {
			behSetTvRv(behPtr, tv, -RV_GAIN);
		}
		else if (bumpType == FRONT_RIGHT_BUMP) {
			behSetTvRv(behPtr, tv, RV_GAIN);
		}
		else if (bumpType == FRONT_BUMP){
			if (prevPrevBumpType == FRONT_LEFT_BUMP) {
				behSetTvRv(behPtr, tv, -RV_GAIN);
			}
			else if (prevPrevBumpType == FRONT_RIGHT_BUMP) {
				behSetTvRv(behPtr, tv, RV_GAIN);
			}
			else {
				behSetTvRv(behPtr, tv, RV_GAIN);
			}
		}
	}

	if (bumpType != prevBumpType) {
		prevPrevBumpType = prevBumpType;
	}

	prevBumpType = bumpType;

	return behPtr;
}
