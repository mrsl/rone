/*
 * globalTreeCOM.c
 *
 *  Created on: Jun 27, 2014
 *      Author: MaJellins
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"
#include "globalTreeCOM.h"

#define PI				3147


void GlobalTreeCOMListCreate(PosistionCOM * posListPtr){
	int i;
	for(i = 0; i <GLOBAL_ROBOTLIST_MAX_SIZE + 1; i++){
		nbrDataCreate16(&posListPtr[i].X_H,&posListPtr[i].X_L,"X_H", "X_L", 0);
		nbrDataCreate16(&posListPtr[i].Y_H,&posListPtr[i].Y_L,"Y_H", "Y_L", 0);
	}

}

/*
 * @brief Updates the center of mass for each tree
 * @param 	globalRobotList - list of robotTrees
 * 			nbrList - list of nbrs
 * 			posListPtr - list center of mass positions for each robot
 * 			Range - defualt range until new way of finding range is created
 * @return void
 */
void GlobalTreeCOMUpdate(GlobalRobotList globalRobotList, NbrList nbrList, PosistionCOM* posListPtr, int Range, NbrData* LeaderHeading_H, NbrData* LeaderHeading_L){
	Nbr* nbrPtr;
	int j,i;
	//For Every tree in the list
	for (j = 0; j < globalRobotList.size; j++) {
		int32 xtot = 0;
		int32 ytot = 0;
		uint8 weight = 0;
		//For ever nbr
		for (i = 0; i < nbrList.size; i++){
			nbrPtr = nbrList.nbrs[i];
			uint8 nbrTreeParentId = nbrDataGetNbr(&(globalRobotList.list[j].ParentID), nbrPtr);
			//If parent of nbr is me, Convert Center of mass to my cordinate frame average with other robots center of mass
			if(nbrTreeParentId == roneID){
				int16 x,y,xprime,yprime;
				nbrPtr = nbrListGetNbr(&nbrList, i);
				int32 nbrOrient = nbrGetOrientation(nbrPtr);
				int32 nbrBear = nbrGetBearing(nbrPtr);

				x = nbrDataGetNbr16(&posListPtr[j].X_H,&posListPtr[j].X_L,nbrPtr);
				y = nbrDataGetNbr16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,nbrPtr);

				xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
				yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
				x = xprime;

				//Range = nbrRangeLookUp(roneID, nbrGetID(nbrPtr));
				y = yprime + Range;

				xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
				yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;

				xtot +=  xprime;
				ytot +=  yprime;
				weight++;
			}
		}
		if(weight == 0){
		//nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,0);
		//nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,0);
		}else{
			int16 xave = xtot/(weight+1);
			int16 yave = ytot/(weight+1);
			nbrDataSet16(&posListPtr[j].X_H,&posListPtr[j].X_L,xave);
			nbrDataSet16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,yave);
			//rprintf("TrID %d XA %d YA %d\n", nbrDataGet(&(globalRobotList.list[j].ID)),xave,yave);
		}
	}

	//Create Pivot Tree

	uint8 lowestTreeID = nbrDataGet(&globalRobotList.list[0].ID);
	uint8 lowestPivotHops = 0;
	uint8 lowestPivotParent = 0;
	if(lowestTreeID == roneID){
		nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,0);
		nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,0);
	} else{
		uint8 pivotHops;
		for (i = 0; i < nbrList.size; i++){
			nbrPtr = nbrList.nbrs[i];
			if(lowestTreeID == nbrGetID(nbrPtr)){
				int32 nbrBear = nbrGetBearing(nbrPtr);
				int16 pivot_X = Range * sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
				int16 pivot_Y = Range * cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;

				nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,pivot_X);
				nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,pivot_Y);
				//rprintf("TrID %d NbrID %d Hops %d\n",lowestTreeID,nbrGetID(nbrPtr),0);
				nbrDataSet16(LeaderHeading_H,LeaderHeading_L,normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation));
				return;
			}
			pivotHops = nbrDataGetNbr(&(globalRobotList.list[0].Hops), nbrPtr);
			if((lowestPivotHops == 0) || (pivotHops<lowestPivotHops) || ( (pivotHops==lowestPivotHops) && (lowestPivotParent < nbrGetID(nbrPtr)))){
				int16 x,y,xprime,yprime;
				int32 nbrOrient = nbrGetOrientation(nbrPtr);
				int32 nbrBear = nbrGetBearing(nbrPtr);
				lowestPivotParent = nbrGetID(nbrPtr);
				lowestPivotHops = pivotHops;
				//rprintf("TrID %d NbrID %d Hops %d\n",lowestTreeID,lowestPivotParent,lowestPivotHops);
				x = nbrDataGetNbr16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,nbrPtr);
				y = nbrDataGetNbr16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,nbrPtr);

				xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
				yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
				x = xprime;

				y = yprime + Range;

				xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
				yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;

				nbrDataSet16(&posListPtr[j].X_H,&posListPtr[j].X_L,xprime);
				nbrDataSet16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,yprime);
				nbrDataSet16(LeaderHeading_H,LeaderHeading_L,normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation)+nbrDataGetNbr16(LeaderHeading_H,LeaderHeading_L,nbrPtr));
			}
		}
	}
}

/*
 * @brief Orbits the given point in X,Y coordinates
 * @param 	COMX - x position of point to orbit
 * 			COMY - y position of point to orbit
 * 			Behrotate - output of orbit behavior
 * 			TV - intended TV of a robot that is orbiting i.e 0 TV means robots stay in place and turn
 * @return void
 */
void GlobalTreePointOrbit(int16 COMX, int16 COMY, Beh* BehRotate, int32 TV){
	int32 bearing = atan2MilliRad((int32)COMY,(int32)COMX) - 3141;
	int32 distance = vectorMag((int32)COMY,(int32)COMX);
	int32 newRv = 0;
	TV = TV * distance / 100;
	if(COMX == 0 && COMY == 0){
		behSetTvRv(BehRotate, 0, 0);
		return;
	}
	if(abs(bearing) > 100){
		if(bearing < 0){
			newRv = bearing/ 1.5;
			behSetTvRv(BehRotate, TV/2, newRv);
		} else{
			newRv = bearing/ 1.5;
			behSetTvRv(BehRotate, TV/2, newRv);
		}
	}else{
		behSetTvRv(BehRotate, TV, 0);
	}
	//rprintf("X%d Y%d b%d RV%d\n",COMX,COMY,bearing,newRv);

}


void GlobalTreeCycloidMotrion(uint cycloidTime, uint32 cycloidPeriod, Beh*behOutput ){
	int32 angleT = 0;
	int32 theta = 0;
	int32 maxSpeed = 200;
	int32 r = 250; //radius of imaginary rotating circle - play with this
	int32 curX = 0;
	int32 curY = 0;

	if(cycloidTime % 10 == 0){ //only sample certain times
		//calculate angleT based on time and period
		angleT = 10*MILLIRAD_2PI * cycloidTime / cycloidPeriod; //time-dependent variable in cycloid eqns (the angle the circle has rotated through)
		//calculate desired angle to axis of translation
		theta = atan2MilliRad(sinMilliRad(angleT), MILLIRAD_TRIG_SCALER - cosMilliRad(angleT)); //arctan(vy/vx) = arctan(sint/1-cost)
		theta = normalizeAngleMilliRad(theta);
		//calculate new x and y from cylcoid equations
		curX =r*(angleT*MILLIRAD_TRIG_SCALER/1000 - sinMilliRad(angleT))/MILLIRAD_TRIG_SCALER;
		curY = r*(MILLIRAD_TRIG_SCALER - cosMilliRad(angleT))/MILLIRAD_TRIG_SCALER;

		//create goal Pose to use for setting tv and rv via waypoint
		Pose waypointGoalPose;
		waypointGoalPose.x = curX;
		waypointGoalPose.y = curY;
		waypointGoalPose.theta = theta;
		waypointMove(&waypointGoalPose, maxSpeed); //set goal pose
	}
	if (cycloidTime < cycloidPeriod) {
		//perform cycloid motion during the cycloid period
		encoderPoseUpdate();
		waypointMoveUpdate(); //use waypoint to set tv and rv

	} else {
		//stop moving when cycloid period has ended
		int32 tv = 0;
		int32 rv = 0;
		behOutput->tv = tv;
		behOutput->rv = rv;
		motorSetTVRV_NonCmd(tv, rv);
		cycloidTime = cycloidTime + 1; //time variable
	}
}

int GlobalTreeCycloidStartPos(int16 COMX, int16 COMY){

	return 0;
}

int nbrRangeLookUp(uint8 myID, uint8 nbrID){
	int Range = 500;
	switch(myID){
	case 8:{
		switch(nbrID){
			case 9:{
				Range = 300;
				break;
			}
			case 10:{
				Range = 600;
				break;
			}
			case 17:{
				Range = 900;
				break;
			}
			case 19:{
				Range = 1200;
				break;
			}
			case 23:{
				Range = 1500;
				break;
			}
			case 24:{
				Range = 1800;
				break;
			}
			case 27:{
				Range = 300;
				break;
			}
		}
		break;
	}
	case 9:{
		switch(nbrID){
			case 8:{
				Range = 300;
				break;
			}
			case 10:{
				Range = 300;
				break;
			}
			case 17:{
				Range = 600;
				break;
			}
			case 19:{
				Range = 900;
				break;
			}
			case 23:{
				Range = 1200;
				break;
			}
			case 24:{
				Range = 1500;
				break;
			}
			case 27:{
				Range = 600;
				break;
			}
		}
		break;
	}
	case 10:{
		switch(nbrID){
			case 8:{
				Range = 600;
				break;
			}
			case 9:{
				Range = 300;
				break;
			}
			case 17:{
				Range = 300;
				break;
			}
			case 19:{
				Range = 600;
				break;
			}
			case 23:{
				Range = 900;
				break;
			}
			case 24:{
				Range = 1200;
				break;
			}
			case 27:{
				Range = 900;
				break;
			}
		}
		break;
	}
	case 17:{
		switch(nbrID){
			case 8:{
				Range = 900;
				break;
			}
			case 9:{
				Range = 600;
				break;
			}
			case 10:{
				Range = 300;
				break;
			}
			case 19:{
				Range = 300;
				break;
			}
			case 23:{
				Range = 600;
				break;
			}
			case 24:{
				Range = 900;
				break;
			}
			case 27:{
				Range = 1200;
				break;
			}
		}
		break;
	}
	case 19:{
		switch(nbrID){
			case 8:{
				Range = 1200;
				break;
			}
			case 9:{
				Range = 900;
				break;
			}
			case 10:{
				Range = 600;
				break;
			}
			case 17:{
				Range = 300;
				break;
			}
			case 23:{
				Range = 300;
				break;
			}
			case 24:{
				Range = 600;
				break;
			}
			case 27:{
				Range = 1500;
				break;
			}
		}
		break;
	}
	case 23:{
		switch(nbrID){
			case 8:{
				Range = 1500;
				break;
			}
			case 9:{
				Range = 1200;
				break;
			}
			case 10:{
				Range = 900;
				break;
			}
			case 17:{
				Range = 600;
				break;
			}
			case 19:{
				Range = 300;
				break;
			}
			case 24:{
				Range = 300;
				break;
			}
			case 27:{
				Range = 1800;
				break;
			}
		}
		break;
	}
	case 24:{
		switch(nbrID){
			case 8:{
				Range = 1800;
				break;
			}
			case 9:{
				Range = 1500;
				break;
			}
			case 10:{
				Range = 1200;
				break;
			}
			case 17:{
				Range = 900;
				break;
			}
			case 19:{
				Range = 600;
				break;
			}
			case 23:{
				Range = 300;
				break;
			}
			case 27:{
				Range = 2100;
				break;
			}
		}
		break;
	}
	case 27:{
		switch(nbrID){
			case 8:{
				Range = 300;
				break;
			}
			case 9:{
				Range = 600;
				break;
			}
			case 10:{
				Range = 900;
				break;
			}
			case 17:{
				Range = 1200;
				break;
			}
			case 19:{
				Range = 1500;
				break;
			}
			case 23:{
				Range = 1800;
				break;
			}
			case 24:{
				Range = 2100;
				break;
			}
		}
		break;
	}
	}
	return Range+50;
}
