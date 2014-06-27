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
		//nbrDataCreate32(&treeGuessCOM[i].X_HH,&treeGuessCOM[i].X_HL,&treeGuessCOM[i].X_LH,&treeGuessCOM[i].X_LL,"X_HH", "X_HL","X_LH", "X_LL", 0);
		//nbrDataCreate32(&treeGuessCOM[i].Y_HH,&treeGuessCOM[i].Y_HL,&treeGuessCOM[i].Y_LH,&treeGuessCOM[i].Y_LL,"Y_HH", "Y_HL","Y_LH", "Y_LL", 0);
		nbrDataCreate16(&posListPtr[i].X_H,&posListPtr[i].X_L,"X_H", "X_L", 0);
		nbrDataCreate16(&posListPtr[i].Y_H,&posListPtr[i].Y_L,"Y_H", "Y_L", 0);
	}

}

void updateGlobalTreeCOM(GlobalRobotList globalRobotList, NbrList nbrList, PosistionCOM* posListPtr, int Range){
	Nbr* nbrPtr;
	int j,i;
	for (j = 0; j < globalRobotList.size; j++) {
		int32 xtot = 0;
		int32 ytot = 0;
		uint8 wieght = 0;
		for (i = 0; i < nbrList.size; i++){
			nbrPtr = nbrList.nbrs[i];
			uint8 nbrTreeParentId = nbrDataGetNbr(&(globalRobotList.list[j].ParentID), nbrPtr);
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
