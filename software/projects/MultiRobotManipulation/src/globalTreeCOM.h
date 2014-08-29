/*
 * globalTreeCOM.h
 *
 *  Created on: Jun 27, 2014
 *      Author: MaJellins
 */

#ifndef GLOBALTREECOM_H_
#define GLOBALTREECOM_H_

#include "roneos.h"
#include "ronelib.h"
#include "scaleCoordinate.h"
#include "lookupTable.h"

#define PI				3147

#define GUIDE_ROBOT_ID		55
#define HOST_ROBOT_ID		19

typedef struct posCOM {
	NbrData X_H;
	NbrData X_L;
	NbrData Y_H;
	NbrData Y_L;
} PositionCOM;

struct {
	int16 centroidX;
	int16 centroidY;
	int16 guideX;
	int16 guideY;
	int16 pivotX;
	int16 pivotY;
	uint8 childCountSum;
} typedef navigationData;

extern boolean isPivot;


void GlobalTreeCOMListCreate(PositionCOM* posListPtr);
void GlobalTreeCOMUpdate(GlobalRobotList globalRobotList, NbrList nbrList, PositionCOM* posListPtr, int Range,  NbrData* LeaderHeading_H, NbrData* LeaderHeading_L);
void GlobalTreePointOrbit(int16 COMX, int16 COMY, Beh* BehRotate, int32 TV);
//void GlobalTreeCycloidMotrion(uint cycloidTime, uint32 cycloidPeriod, uint16 distCOM ,int32 maxSpeed, int32 radius, Beh*behOutput);
//void GlobalTreeCycloidMotrion(uint cycloidTime, int16 COMX, int16 COMY ,int32 maxSpeed, int32 radius, Beh*behOutput);
int nbrRangeLookUp(uint8 myID, uint8 nbrID);

void createGRLscaleCoordinates(scaleCoordinate scaleCoordinateArray[]);

void centroidGRLUpdate(navigationData *navDataPtr,
					   GlobalRobotList globalRobotList,
					   NbrList *nbrListPtr,
					   scaleCoordinate scaleCoordinateArray[]);

void centroidGRLListUpdate(navigationData *navDataPtr,
						   GlobalRobotListElement *grlElement,
						   NbrList *nbrListPtr,
						   scaleCoordinate *centroidEstimate);
void CentroidGRLPrintAllTrees(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr, PositionCOM* posListPtr);
void CentroidGRLPrintSelfTree(GlobalRobotList* globalRobotListPtr, PositionCOM* posListPtr);
void CentroidGRLPrintNbrTree(GlobalRobotList* globalRobotListPtr, Nbr* nbrptr, PositionCOM* posListPtr);
void CentroidGRLPrintEstimate(GlobalRobotList* globalRobotList, PositionCOM* posListPtr);
#endif /* GLOBALTREECOM_H_ */
