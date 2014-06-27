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


void creatGlobalTreeCOMList(PosistionCOM * posListPtr){
	int i;
	for(i = 0; i <GLOBAL_ROBOTLIST_MAX_SIZE; i++){
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
void updateGlobalTreeCOM(GlobalRobotList globalRobotList, NbrList nbrList, PosistionCOM* posListPtr, int Range){
	Nbr* nbrPtr;
	int j,i;
	//For Every tree in the list
	for (j = 0; j < globalRobotList.size; j++) {
		int32 xtot = 0;
		int32 ytot = 0;
		uint8 wieght = 0;
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

				//if(printNow){rprintf("ID %d TrID %d J%d - Nbr %d: X%d Y%d\n", roneID, nbrDataGetNbr(&(globalRobotListPtr.list[j].ID),nbrPtr), j, nbrGetID(nbrPtr), x,y);}

				xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
				yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
				x = xprime;
				switch(roneID){
				case 53:{
					switch(nbrGetID(nbrPtr)){
						case 42:{
							Range = 400;
							break;
						}
						case 2:{
							Range = 570;
							break;
						}
						case 31:{
							Range = 500;
							break;
						}
					}
					break;
				}
				case 42:{
					switch(nbrGetID(nbrPtr)){
						case 53:{
							Range = 400;
							break;
						}
						case 2:{
							Range = 310;
							break;
						}
						case 31:{
							Range = 670;
							break;
						}
					}
					break;

				}
				case 2:{
					switch(nbrGetID(nbrPtr)){
						case 53:{
							Range = 570;
							break;
						}
						case 42:{
							Range = 310;
							break;
						}
						case 31:{
							Range = 450;
							break;
						}
					}
					break;

				}
				case 31:{
					switch(nbrGetID(nbrPtr)){
						case 53:{
							Range = 500;
							break;
						}
						case 42:{
							Range = 670;
							break;
						}
						case 2:{
							Range = 450;
							break;
						}
					}
					break;
				}
				}
				y = yprime + Range;

				xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
				yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;

				//if(printNow){rprintf("Nbr %d: O%d B%d X''%d Y''%d XCOM%d YCOM%d\n",nbrGetID(nbrPtr),nbrOrient,nbrBear,x,y, xprime,yprime);}
				xtot +=  xprime;
				ytot +=  yprime;
				wieght++;
			}
		}
		if(wieght == 0){
		//nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,0);
		//nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,0);
		}else{
			int16 xave = xtot/wieght;
			int16 yave = ytot/wieght;
			nbrDataSet16(&posListPtr[j].X_H,&posListPtr[j].X_L,xave);
			nbrDataSet16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,yave);
			//if(printNow){rprintf("TrID %d XA %d YA %d\n", nbrDataGet(&(globalRobotList.list[j].ID)),xave,yave);}
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
void orbitGlobalTreePoint(int16 COMX, int16 COMY, Beh* BehRotate, int32 TV){
	int32 bearing = atan2MilliRad((int32)COMY,(int32)COMX) - 3141;
	int32 newRv = 0;
	if(abs(bearing) > 100){
		if(bearing < 0){
			newRv = bearing/ 2;
			behSetTvRv(BehRotate, 0, newRv);
		} else{
			newRv = bearing/ 2;
			behSetTvRv(BehRotate, 0, newRv);
		}
	}else{
		behSetTvRv(BehRotate, TV, 0);
	}
	rprintf("X%d Y%d b%d RV%d\n",COMX,COMY,bearing,newRv);

}
