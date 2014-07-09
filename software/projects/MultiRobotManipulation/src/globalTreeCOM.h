/*
 * globalTreeCOM.h
 *
 *  Created on: Jun 27, 2014
 *      Author: MaJellins
 */

#ifndef GLOBALTREECOM_H_
#define GLOBALTREECOM_H_

typedef struct posCOM {
	NbrData X_H;
	NbrData X_L;
	NbrData Y_H;
	NbrData Y_L;
} PosistionCOM;


void GlobalTreeCOMListCreate(PosistionCOM* posListPtr);
void GlobalTreeCOMUpdate(GlobalRobotList globalRobotList, NbrList nbrList, PosistionCOM* posListPtr, int Range,  NbrData* LeaderHeading_H, NbrData* LeaderHeading_L);
void GlobalTreePointOrbit(int16 COMX, int16 COMY, Beh* BehRotate, int32 TV);
void GlobalTreeCycloidMotrion(uint cycloidTime, uint32 cycloidPeriod, uint16 distCOM ,int32 maxSpeed, int32 radius, Beh*behOutput);
//void GlobalTreeCycloidMotrion(uint cycloidTime, int16 COMX, int16 COMY ,int32 maxSpeed, int32 radius, Beh*behOutput);
int GlobalTreeCycloidStartPos(int16 COMX, int16 COMY);
int nbrRangeLookUp(uint8 myID, uint8 nbrID);
#endif /* GLOBALTREECOM_H_ */
