/*
 * globalTreeCOM.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Zak
 */

#ifndef GLOBALTREECOM_H_
#define GLOBALTREECOM_H_

#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"
#include "scaleCoordinate.h"

#define BEHAVIOR_TASK_PERIOD	50
#define NEIGHBOR_ROUND_PERIOD	1500
#define RPRINTF_SLEEP_TIME		100

#define CHECKVAL		0xDADA

#define MSG_TYPE_LT		0
#define MSG_TYPE_ST		1

#define STATE_IDLE		0
#define STATE_CGUESS	1
#define STATE_RALIGN	2
#define STATE_ROTATE	3
#define STATE_PALIGN	4
#define STATE_PIVOT 	5
#define STATE_TALIGN	6
#define STATE_FTRANS 	7
#define STATE_BTRANS 	8

#define STATE_MAX		STATE_BTRANS

#define REST			0
#define CNTCLK			1
#define CLKWISE			2
#define ATTEMPTING		3

#define RAVG_SIZE		50

#define MRM_ROTATE_TV_GAIN		5
#define MRM_PIVOT_TV_GAIN		5
#define MRM_TRANS_TV_GAIN		40
#define MRM_MAX_TV				60

typedef struct posCOM {
	NbrData X_H;
	NbrData X_L;
	NbrData Y_H;
	NbrData Y_L;
} PositionCOM;

// Data structure to contain all our current location estimations
struct {
	int16 centroidX;
	int16 centroidY;
	int16 guideX;
	int16 guideY;
	int16 pivotX;
	int16 pivotY;
	uint8 childCountSum;
} typedef navigationData;

// Tree and coordinate operations and update functions
void createGRLscaleCoordinates(scaleCoordinate scaleCoordinateArray[]);
void createGRLpivotCoordinate(scaleCoordinate *pivot);
void createGRLguideCoordinate(scaleCoordinate *guide);
void createStateInformation();
void setGRLpivot(uint8 id);
void setGRLguide(uint8 id);

uint8 getPivotRobot();
uint8 getGuideRobot();

void updateDistributedInformation(NbrList *nbrListPtr);


void centroidGRLUpdate(navigationData *navDataPtr,
					   GlobalRobotList *globalRobotList,
					   NbrList *nbrListPtr,
					   scaleCoordinate scaleCoordinateArray[]);


void centroidGRLListUpdate(navigationData *navDataPtr,
						   GlobalRobotListElement *grlElement,
						   NbrList *nbrListPtr,
						   scaleCoordinate *centroidEstimate);

void pivotGRLUpdate(navigationData *navDataPtr,
				    GlobalRobotList *globalRobotList,
				    NbrList *nbrListPtr,
				    scaleCoordinate *pivotCoordinate);

void guideGRLUpdate(navigationData *navDataPtr,
				    GlobalRobotList *globalRobotList,
				    NbrList *nbrListPtr,
				    scaleCoordinate *guideCoordinate);

void rootedLocationTreeUpdate(GlobalRobotList *globalRobotList,
				    		  NbrList *nbrListPtr,
				    		  uint8 rootRobot,
				    		  scaleCoordinate *sc,
				    		  int16 *x, int16 *y);

// Controllers
void mrmOrbitCentroid(navigationData *navData, Beh *beh, int32 tvModifier);
void mrmOrbitPivot(navigationData *navData, Beh *beh, int32 tvModifier);
void mrmPointOrbit(Beh *beh, int32 x, int32 y, int32 tvModifier);
void mrmTranslateLeaderToGuide(navigationData *navDataPtr, NbrList *nbrListPtr,
									Beh *behPtr, int32 tvModifier);

// Input callbacks
void mrmInitCallbacks();

// Helpers
void setState(uint8 newState);
uint8 getState();

void setStartNbrRound(uint32 nbrRound);
boolean isInitStartNbrRound();
uint32 getDeltaStartNbrRound(uint32 nbrRound);

int32 mrmIIR(int32 currentVal, int32 newVal, int32 alpha);
void navDataInit(navigationData *navData);
void copyNavData(navigationData *toCopy, navigationData *toMe);
void rollingAverageNavData(navigationData *new, navigationData *avg);


//void GlobalTreeCOMListCreate(PositionCOM* posListPtr);
//void GlobalTreeCOMUpdate(GlobalRobotList globalRobotList, NbrList nbrList, PositionCOM* posListPtr, int Range,  NbrData* LeaderHeading_H, NbrData* LeaderHeading_L);
//void GlobalTreePointOrbit(int16 COMX, int16 COMY, Beh* BehRotate, int32 TV);
////void GlobalTreeCycloidMotrion(uint cycloidTime, uint32 cycloidPeriod, uint16 distCOM ,int32 maxSpeed, int32 radius, Beh*behOutput);
////void GlobalTreeCycloidMotrion(uint cycloidTime, int16 COMX, int16 COMY ,int32 maxSpeed, int32 radius, Beh*behOutput);
//int nbrRangeLookUp(uint8 myID, uint8 nbrID);
//
//void CentroidGRLPrintAllTrees(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr, PositionCOM* posListPtr);
//void CentroidGRLPrintSelfTree(GlobalRobotList* globalRobotListPtr, PositionCOM* posListPtr);
//void CentroidGRLPrintNbrTree(GlobalRobotList* globalRobotListPtr, Nbr* nbrptr, PositionCOM* posListPtr);
//void CentroidGRLPrintEstimate(GlobalRobotList* globalRobotList, PositionCOM* posListPtr);
#endif /* GLOBALTREECOM_H_ */
