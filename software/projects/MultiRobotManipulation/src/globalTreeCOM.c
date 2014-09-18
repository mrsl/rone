/*
 * globalTreeCOM.c
 *
 *  Created on: Jun 27, 2014
 *      Author: Zak
 */

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"
#include "globalTreeCOM.h"

#define ROTATE_STEPS	PI/20

NbrData pivotRobot;
NbrData pivotNonce;

NbrData guideRobot;
NbrData guideNonce;

NbrData stateInfo;
NbrData stateNonce;

/**
 * Creates the scale coordinate array for centroid locations
 */
void createGRLscaleCoordinates(scaleCoordinate scaleCoordinateArray[]){
	int i;

	for (i = 0; i < GLOBAL_ROBOTLIST_MAX_SIZE; i++) {
		createScaleCoordinate(&scaleCoordinateArray[i]);
	}
}

/**
 * Creates the scale coordinate for the pivot
 */
void createGRLpivotCoordinate(scaleCoordinate *pivot) {
	createScaleCoordinate(pivot);
	nbrDataCreate(&pivotRobot, "pivot", 8, 0);
	nbrDataCreate(&pivotNonce, "pivotNonce", 8, 0);
}

/**
 * Creates the scale coordinate for the guide
 */
void createGRLguideCoordinate(scaleCoordinate *guide) {
	createScaleCoordinate(guide);
	nbrDataCreate(&guideRobot, "guide", 8, 0);
	nbrDataCreate(&guideNonce, "guideNonce", 8, 0);
}

/**
 * Creates the scale coordinate for the guide
 */
void createStateInformation() {
	nbrDataCreate(&stateInfo, "state", 8, 0);
	nbrDataCreate(&stateNonce, "stateNonce", 8, 0);
}

/**
 * Sets a new pivot robot for the network
 */
void setGRLpivot(uint8 id) {
	nbrDataSet(&pivotRobot, id);
	nbrDataSet(&pivotNonce, nbrDataGet(&pivotNonce) + 1);
}

/**
 * Sets a new guide robot for the network
 */
void setGRLguide(uint8 id) {
	nbrDataSet(&guideRobot, id);
	nbrDataSet(&guideNonce, nbrDataGet(&guideNonce) + 1);
}

uint8 getPivotRobot() {
	return nbrDataGet(&pivotRobot);
}

uint8 getGuideRobot() {
	return nbrDataGet(&guideRobot);
}

/**
 * Sets our state for the FSM
 */
void setState(uint8 newState) {
	nbrDataSet(&stateInfo, newState);
	nbrDataSet(&stateNonce, nbrDataGet(&stateNonce) + 1);

	if (newState == STATE_IDLE) {
		setStartNbrRound(0);
	}
}

uint8 getState() {
	return nbrDataGet(&stateInfo);
}

/**
 * Updates pivot and guide robot knowledge
 */
void updateDistributedInformation(NbrList *nbrListPtr) {
	int i;
	Nbr *nbrPtr;

	for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
		nbrPtr = nbrListGetNbr(nbrListPtr, i);

		// Check for new pivot
		if (nbrDataGetNbr(&pivotRobot, nbrPtr) != nbrDataGet(&pivotRobot)) {
			if (nbrDataGetNbr(&pivotNonce, nbrPtr) >= nbrDataGet(&pivotNonce)) {
				nbrDataSet(&pivotRobot, nbrDataGetNbr(&pivotRobot, nbrPtr));
				nbrDataSet(&pivotNonce, nbrDataGetNbr(&pivotNonce, nbrPtr));
			}
		}

		// Check for new guide
		if (nbrDataGetNbr(&guideRobot, nbrPtr) != nbrDataGet(&guideRobot)) {
			if (nbrDataGetNbr(&guideNonce, nbrPtr) >= nbrDataGet(&guideNonce)) {
				nbrDataSet(&guideRobot, nbrDataGetNbr(&guideRobot, nbrPtr));
				nbrDataSet(&guideNonce, nbrDataGetNbr(&guideNonce, nbrPtr));
			}
		}

		// Check for new guide
		if (nbrDataGetNbr(&stateInfo, nbrPtr) != nbrDataGet(&stateInfo)) {
			if (nbrDataGetNbr(&stateNonce, nbrPtr) >= nbrDataGet(&stateNonce)) {
				nbrDataSet(&stateInfo, nbrDataGetNbr(&stateInfo, nbrPtr));
				nbrDataSet(&stateNonce, nbrDataGetNbr(&stateNonce, nbrPtr));
			}
		}
	}
}

/**
 * @brief Updates the GRL's centroid estimate
 */
void centroidGRLUpdate(navigationData *navDataPtr,
					   GlobalRobotList *globalRobotList,
					   NbrList *nbrListPtr,
					   scaleCoordinate scaleCoordinateArray[]) {
	// Variables
	int i;
	navigationData tempNavData;			// Temporary coordinates
	GlobalRobotListElement* grlEltPtr;	// Pointer to a GRL Tree (element)
	uint16 treeId,						// Id of the Tree
		   treeParentId;				// Id of the root of the Tree

	// For every tree in the GRL
	for (i = 0; i < globalRobotList->size; i++) {
		grlEltPtr = globalRobotListGetElt(globalRobotList, i);
		treeId = grlEltGetID(grlEltPtr);
		treeParentId = grlEltGetParentID(grlEltPtr);

		// If regular manipulator robot in the tree
		if (treeId != getGuideRobot()) {
			// Update the tree and find the centroid sums
			centroidGRLListUpdate(&tempNavData,
								  &globalRobotList->list[i],
								  nbrListPtr,
								  &scaleCoordinateArray[i]);

			// If we are the root of the tree, set our estimate
			if (treeParentId == 0) {
				navDataPtr->centroidX = tempNavData.centroidX;
				navDataPtr->centroidY = tempNavData.centroidY;
				navDataPtr->childCountSum = tempNavData.childCountSum;
			}
		}
	}
}

/**
 * @brief Updates a single GRL tree (element) with new neighbor data
 */
void centroidGRLListUpdate(navigationData *navDataPtr,
						   GlobalRobotListElement *grlElement,
						   NbrList *nbrListPtr,
						   scaleCoordinate *centroidEstimate) {
	// Variables
	int i;
	int16 x,					// X and Y of a child
		  y,
		  xSum = 0,				// Summed X and Y of children
		  ySum = 0;
	uint8 childCount,			// Child count from a child (Includes child)
		  childCountSum = 0;	// Total amount of children from children
	Nbr *nbrPtr;
	uint8 nbrId,				// Neighbor's ID
		  nbrTreeParentId;		// ID of the neighbor's parent

	for (i = 0; i < nbrListPtr->size; i++){
		nbrPtr = nbrListPtr->nbrs[i];

		//uint8 nbrTreeId = nbrDataGetNbr(&(grlElement->ID), nbrPtr);
		nbrId = nbrGetID(nbrPtr);

		if (nbrId != roneID) {
			nbrTreeParentId = nbrDataGetNbr(&(grlElement->ParentID), nbrPtr);

			// If we are the parent node in the tree, count and sum child nodes
			// Also, ignore the navigational guide robot
			if (nbrTreeParentId == roneID && nbrId != getGuideRobot()) {
				// Transform remote coordinate to local reference frame and add to sums
				transformScaleCoordinate(centroidEstimate, nbrPtr, &x, &y,
					&childCount);
				xSum += x;
				ySum += y;
				childCountSum += childCount;
			}
		}
	}
	// Update our knowledge of centroid sums
	updateScaleCoordinate(centroidEstimate, xSum, ySum, ++childCountSum);

	// Set given data structure values to current centroid estimate
	navDataPtr->centroidX = xSum / childCountSum;
	navDataPtr->centroidY = ySum / childCountSum;
	navDataPtr->childCountSum = childCountSum;
}

void pivotGRLUpdate(navigationData *navDataPtr,
				    GlobalRobotList *globalRobotList,
				    NbrList *nbrListPtr,
				    scaleCoordinate *pivotCoordinate) {
	int16 x = 0,
		  y = 0;

	rootedLocationTreeUpdate(globalRobotList,
							 nbrListPtr,
							 getPivotRobot(),
							 pivotCoordinate,
							 &x, &y);

	navDataPtr->pivotX = x;
	navDataPtr->pivotY = y;
}

void guideGRLUpdate(navigationData *navDataPtr,
				    GlobalRobotList *globalRobotList,
				    NbrList *nbrListPtr,
				    scaleCoordinate *guideCoordinate) {
	int16 x = 0,
		  y = 0;

	rootedLocationTreeUpdate(globalRobotList,
							 nbrListPtr,
							 getGuideRobot(),
							 guideCoordinate,
							 &x, &y);

	navDataPtr->guideX = x;
	navDataPtr->guideY = y;
}

void rootedLocationTreeUpdate(GlobalRobotList *globalRobotList,
				    		  NbrList *nbrListPtr,
				    		  uint8 rootRobot,
				    		  scaleCoordinate *sc,
				    		  int16 *x, int16 *y) {
	// Variables
	int i;
	GlobalRobotListElement* grlEltPtr = NULL; // Pointer to a GRL Tree (element)

	// If we are the root set location to ourselves
	*x = 0;
	*y = 0;

	// Find the rooted tree
	for (i = 0; i < globalRobotList->size; i++) {
		grlEltPtr = globalRobotListGetElt(globalRobotList, i);
		if (grlEltGetID(grlEltPtr) == rootRobot) {
			break;
		}
	}

	if (grlEltPtr == NULL) {
		return; // error
	}

	Nbr *nbrPtr;
	uint8 myParent = nbrDataGet(&(grlEltPtr->ParentID));

	for (i = 0; i < nbrListPtr->size; i++){
		nbrPtr = nbrListPtr->nbrs[i];

		if (nbrGetID(nbrPtr) == myParent) {
			scShiftNbrReferenceFrame(sc, nbrPtr, x, y);
			break;
		}
	}

	updateScaleCoordinate(sc, *x, *y, 0);
}

//void pivotGRLUpdate(navigationData *navDataPtr,
//				    GlobalRobotList *globalRobotList,
//				    NbrList *nbrListPtr,
//				    scaleCoordinate *pivotCoordinate) {
//	// Variables
//	int i;
//	GlobalRobotListElement* grlEltPtr = NULL; // Pointer to a GRL Tree (element)
//	int16 x = 0, y = 0;
//
//	// If we are the pivot set location to ourselves
//	if (isPivot) {
//		updateScaleCoordinate(pivotCoordinate, 0, 0, 0);
//
//		navDataPtr->pivotX = x;
//		navDataPtr->pivotY = y;
//
//	// Use the tree rooted on the provided robot
//	} else {
//		// Find the pivot tree
//		for (i = 0; i < globalRobotList->size; i++) {
//			grlEltPtr = globalRobotListGetElt(globalRobotList, i);
//			if (grlEltGetID(grlEltPtr) == nbrDataGet(&pivotRobot)) {
//				break;
//			}
//		}
//		if (grlEltPtr == NULL) {
//			return; // error
//		}
//
//		Nbr *nbrPtr;
//		uint8 nbrId;
//		uint8 myParent = nbrDataGet(&(grlEltPtr->ParentID));
//
//		for (i = 0; i < nbrListPtr->size; i++){
//			nbrPtr = nbrListPtr->nbrs[i];
//			nbrId = nbrGetID(nbrPtr);
//
//			if (nbrId == myParent) {
//				scShiftNbrReferenceFrame(pivotCoordinate, nbrPtr, &x, &y);
//				updateScaleCoordinate(pivotCoordinate, x, y, 0);
//
//				navDataPtr->pivotX = x;
//				navDataPtr->pivotY = y;
//				return;
//			}
//		}
//	}
//}

////////////////////////////////////////////////////////////////////////////////
//void CentroidGRLPrintAllTrees(GlobalRobotList* globalRobotListPtr, NbrList* nbrListPtr, PositionCOM* posListPtr){
//	int8 i;
//	int16 xchd;
//	Nbr* nbrPtr;
//	CentroidGRLPrintSelfTree(globalRobotListPtr, posListPtr);
//	for (i = 0; i < nbrListGetSize(nbrListPtr); i++) {
//		// get a nbrptr from my neighbor list
//		nbrPtr = nbrListGetNbr(nbrListPtr, i);
//		if(nbrPtr!= NULL){
//			CentroidGRLPrintNbrTree(globalRobotListPtr, nbrPtr, posListPtr);
//		}
//	}
//	rprintf("\n");
//}
//
//void CentroidGRLPrintSelfTree(GlobalRobotList* globalRobotListPtr, PositionCOM* posListPtr) {
//	int j;
//	GlobalRobotListElement* grlEltPtr;
//	rprintf("%d", roneID);
//	for (j = 0; j < globalRobotListPtr->size; j++) {
//		grlEltPtr = globalRobotListGetElt(globalRobotListPtr, j);
//		uint8 nbrTotalRobotListRobotID = grlEltGetID(grlEltPtr);
//		rprintf(",%d,%d,%d", nbrTotalRobotListRobotID, grlEltGetParentID(grlEltPtr), grlEltGetHops(grlEltPtr));
//		rprintf("\n");
//
//	}
//	rprintf("\n");
//} // todo: print our guess
//
//void CentroidGRLPrintNbrTree(GlobalRobotList* globalRobotListPtr, Nbr* nbrptr, PositionCOM* posListPtr){
//	int j;
//	rprintf("%d", nbrGetID(nbrptr));
//	for (j = 0; j < GLOBAL_ROBOTLIST_MAX_SIZE; j++) {
//		uint8 nbrRobotListID = nbrDataGetNbr(&(globalRobotListPtr->list[j].ID), nbrptr);
//		uint8 nbrRobotListParentID = nbrDataGetNbr(&(globalRobotListPtr->list[j].ParentID), nbrptr);
//		if (nbrRobotListID == ROBOT_ID_NULL) {
//			rprintf("\n");
//			return;
//		}
//		rprintf(",%d,%d", nbrRobotListID, nbrRobotListParentID);
//		rprintf("\n");
//
//	}
//	//rprintf(",%d", nbrDataGetNbr16(&posListPtr[j].X_H,&posListPtr[j].X_L, nbrptr));
//	//rprintf("\n");
//}
//
//void CentroidGRLPrintEstimate(GlobalRobotList* globalRobotList, PositionCOM* posListPtr){
//		int  j;
//		//For Every tree in the list
//		for (j = 0; j < globalRobotList->size; j++) {
//
//				rprintf(",%d,%d,%d,%d ", &posListPtr[j].X_H, &posListPtr[j].X_L, &posListPtr[j].Y_H, &posListPtr[j].Y_L);
//				rprintf("\n");
//
//				}
//	rprintf("\n");
//}
//
//void GlobalTreeCOMListCreate(PositionCOM * posListPtr){
//	int i;
//
//	for(i = 0; i <GLOBAL_ROBOTLIST_MAX_SIZE + 1; i++){
//		nbrDataCreate16(&posListPtr[i].X_H,&posListPtr[i].X_L,"X_H", "X_L", 0);
//		nbrDataCreate16(&posListPtr[i].Y_H,&posListPtr[i].Y_L,"Y_H", "Y_L", 0);
//
//		nbrDataSet16(&posListPtr[i].X_H, &posListPtr[i].X_L, 0);
//		nbrDataSet16(&posListPtr[i].Y_H, &posListPtr[i].Y_L, 0);
//	}
//
//}
//
///*
// * @brief Updates the center of mass for each tree
// * @param 	globalRobotList - list of robotTrees
// * 			nbrList - list of nbrs
// * 			posListPtr - list center of mass positions for each robot
// * 			Range - defualt range until new way of finding range is created
// * @return void
// */
//void GlobalTreeCOMUpdate(GlobalRobotList globalRobotList, NbrList nbrList, PositionCOM* posListPtr, int Range, NbrData* LeaderHeading_H, NbrData* LeaderHeading_L){
//	Nbr* nbrPtr;
//	int j,i;
//	//For Every tree in the list
//	for (j = 0; j < globalRobotList.size; j++) {
//		int32 xtot = 0;
//		int32 ytot = 0;
//		uint8 weight = 0; // counts the number of children
//		//For ever nbr
//		for (i = 0; i < nbrList.size; i++){
//			nbrPtr = nbrList.nbrs[i];
//			uint8 nbrTreeParentId = nbrDataGetNbr(&(globalRobotList.list[j].ParentID), nbrPtr);
//			//If parent of nbr is me, Convert Center of mass to my coordinate frame average with other robots center of mass
//			if(nbrTreeParentId == roneID){
//				int16 x,y,xprime,yprime;
//				nbrPtr = nbrListGetNbr(&nbrList, i);
//				int32 nbrOrient = nbrGetOrientation(nbrPtr);
//				int32 nbrBear = nbrGetBearing(nbrPtr);
//
//				x = nbrDataGetNbr16(&posListPtr[j].X_H,&posListPtr[j].X_L,nbrPtr);
//				y = nbrDataGetNbr16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,nbrPtr);
//
//				xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
//				yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
//				x = xprime;
//
//				//Range = nbrRangeLookUp(roneID, nbrGetID(nbrPtr));
//				y = yprime + Range;
//
//				xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
//				yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
//
//				xtot +=  xprime;
//				ytot +=  yprime;
//				weight++;
//			}
//		}
//		if(weight == 0){
//		//nbrDataSet16(&treeGuessCOM[j].X_H,&treeGuessCOM[j].X_L,0);
//		//nbrDataSet16(&treeGuessCOM[j].Y_H,&treeGuessCOM[j].Y_L,0);
//		}else{
//			int16 xave = xtot/(weight+1);
//			int16 yave = ytot/(weight+1);
//			nbrDataSet16(&posListPtr[j].X_H,&posListPtr[j].X_L,xave);
//			nbrDataSet16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,yave);
//			//rprintf("TrID %d XA %d YA %d\n", nbrDataGet(&(globalRobotList.list[j].ID)),xave,yave);
//		}
//	}
//
//	//Create Pivot Tree
//
//	uint8 lowestTreeID = nbrDataGet(&globalRobotList.list[0].ID);
//	uint8 lowestPivotHops = 0;
//	uint8 lowestPivotParent = 0;
//	if(lowestTreeID == roneID){
//		nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,0);
//		nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,0);
//	} else{
//		uint8 pivotHops;
//		for (i = 0; i < nbrList.size; i++){
//			nbrPtr = nbrList.nbrs[i];
//			if(lowestTreeID == nbrGetID(nbrPtr)){
//				int32 nbrBear = nbrGetBearing(nbrPtr);
//				int16 x,y,xprime,yprime;
//				x = 0;
//				y = Range;
//				xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
//				yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
//
//				nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,xprime);
//				nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,yprime);
//				//rprintf("TrID %d NbrID %d Hops %d\n",lowestTreeID,nbrGetID(nbrPtr),0);
//				nbrDataSet16(LeaderHeading_H,LeaderHeading_L,normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation));
//				return;
//			}
//			pivotHops = nbrDataGetNbr(&(globalRobotList.list[0].Hops), nbrPtr);
//			if((lowestPivotHops == 0) || (pivotHops<lowestPivotHops) || ( (pivotHops==lowestPivotHops) && (lowestPivotParent < nbrGetID(nbrPtr)))){
//				int16 x,y,xprime,yprime;
//				int32 nbrOrient = nbrGetOrientation(nbrPtr);
//				int32 nbrBear = nbrGetBearing(nbrPtr);
//				lowestPivotParent = nbrGetID(nbrPtr);
//				lowestPivotHops = pivotHops;
//				//rprintf("TrID %d NbrID %d Hops %d\n",lowestTreeID,lowestPivotParent,lowestPivotHops);
//				x = nbrDataGetNbr16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,nbrPtr);
//				y = nbrDataGetNbr16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,nbrPtr);
//
//				xprime = x*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
//				yprime = x*sinMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrOrient)/MILLIRAD_TRIG_SCALER;
//				x = xprime;
//
//				y = yprime + Range;
//
//				xprime = x*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER - y*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
//				yprime = x*sinMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER + y*cosMilliRad(nbrBear)/MILLIRAD_TRIG_SCALER;
//
//				nbrDataSet16(&posListPtr[j].X_H,&posListPtr[j].X_L,xprime);
//				nbrDataSet16(&posListPtr[j].Y_H,&posListPtr[j].Y_L,yprime);
//				nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].X_L,xprime);
//				nbrDataSet16(&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_H,&posListPtr[GLOBAL_ROBOTLIST_MAX_SIZE].Y_L,yprime);
//
//				nbrDataSet16(LeaderHeading_H,LeaderHeading_L,normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation)+nbrDataGetNbr16(LeaderHeading_H,LeaderHeading_L,nbrPtr));
//			}
//		}
//	}
//
//}
//
//
//
///*
// * @brief Orbits the given point in X,Y coordinates
// * @param 	COMX - x position of point to orbit
// * 			COMY - y position of point to orbit
// * 			Behrotate - output of orbit behavior
// * 			TV - intended TV of a robot that is orbiting i.e 0 TV means robots stay in place and turn
// * @return void
// */
//void GlobalTreePointOrbit(int16 COMX, int16 COMY, Beh* BehRotate, int32 TV){
//	int32 bearing = atan2MilliRad((int32)COMY,(int32)COMX) - MILLIRAD_PI;
//	int32 distance = vectorMag((int32)COMY,(int32)COMX);
//	int32 newRv = 0;
//	TV = TV * distance / 100;
//	if(COMX == 0 && COMY == 0){
//		behSetTvRv(BehRotate, 0, 0);
//		return;
//	}
//
//	if(abs(bearing) > 100){
//		if(bearing < 0){
//			newRv = bearing/ 1.5;
//			behSetTvRv(BehRotate, TV/2, newRv);
//		} else{
//			newRv = bearing/ 1.5;
//			behSetTvRv(BehRotate, TV/2, newRv);
//		}
//	}else{
//		behSetTvRv(BehRotate, TV, 0);
//	}
//
//	//rprintf("X%d Y%d b%d RV%d\n",COMX,COMY,bearing,newRv);
//
//}
//
///**
// * @brief A path for a single robot cycliod path.
// * @param	cycloidTime	- Time sent out by clock robot
// * 			cycloidPeriod - Period for full cyliod rotation
// * 			distCOM - distance to center of mass
// * 			maxSpeed - maxSpeed allowed of cyliod motion
// * 			radius - Radius of object
// */
//void GlobalTreeCycloidMotrion(uint cycloidTime, uint32 cycloidPeriod, uint16 distCOM ,int32 maxSpeed, int32 radius, Beh*behOutput){
//	int32 angleT = 0;
//	int32 theta = 0;
//	int32 curX = 0;
//	int32 curY = 0;
//	cycloidTime = cycloidTime%cycloidPeriod;
//	distCOM = distCOM / 100;
//	if(cycloidTime % 10 == 0){ //only sample certain times
//		//calculate angleT based on time and period
//		angleT = 10*MILLIRAD_2PI * cycloidTime / cycloidPeriod; //time-dependent variable in cycloid eqns (the angle the circle has rotated through)
//		//calculate desired angle to axis of translation
//		theta = atan2MilliRad(sinMilliRad(angleT), MILLIRAD_TRIG_SCALER - cosMilliRad(angleT)); //arctan(vy/vx) = arctan(sint/1-cost)
//		theta = normalizeAngleMilliRad(theta);
//
//		//calculate new x and y from cylcoid equations
//		curX =radius*(angleT*MILLIRAD_TRIG_SCALER/1000 - distCOM * sinMilliRad(angleT))/MILLIRAD_TRIG_SCALER;
//		curY = radius*(MILLIRAD_TRIG_SCALER - distCOM * cosMilliRad(angleT))/MILLIRAD_TRIG_SCALER;
//		//curX =radius*(angleT*MILLIRAD_TRIG_SCALER/1000 - sinMilliRad(angleT))/MILLIRAD_TRIG_SCALER;
//		//curY = radius*(MILLIRAD_TRIG_SCALER - cosMilliRad(angleT))/MILLIRAD_TRIG_SCALER;
//
//		//create goal Pose to use for setting tv and rv via waypoint
//		Pose waypointGoalPose;
//		waypointGoalPose.x = curX;
//		waypointGoalPose.y = curY;
//		waypointGoalPose.theta = theta;
//		waypointMove(&waypointGoalPose, maxSpeed); //set goal pose
//		//rprintf("T %d D %d MS%d R %d\n",cycloidTime,distCOM,maxSpeed,radius);
//	}
//	encoderPoseUpdate();
//	waypointMoveUpdate(); //use waypoint to set tv and rv
//
//}
//
///*void GlobalTreeCycloidMotrion(uint cycloidTime, int16 COMX, int16 COMY ,int32 maxSpeed, int32 radius, Beh*behOutput){
//	if(cycloidTime % 10 == 0){ //only sample certain times
//		int16 newRotatePos = ROTATE_STEPS * cycloidTime / 10;
//		int16 newX =  (((-COMX)* (cosMilliRad(newRotatePos))) + ((-COMY) * (-sinMilliRad(newRotatePos))))/MILLIRAD_TRIG_SCALER;
//		int16 newY =  (((-COMX)* (sinMilliRad(newRotatePos))) + ((-COMY) * (cosMilliRad(newRotatePos))))/MILLIRAD_TRIG_SCALER;
//		Pose waypointGoalPose;
//		waypointGoalPose.x = newX;
//		waypointGoalPose.y = newY;
//		waypointGoalPose.theta = 0;
//		waypointMove(&waypointGoalPose, maxSpeed); //set goal pose
//		rprintf("T %d RP %d RS%d C %d %d N %d %d\n",(uint32)cycloidTime, newRotatePos,ROTATE_STEPS,COMX,COMY,newX,newY);
//	}
//	//perform cycloid motion during the cycloid period
//	encoderPoseUpdate();
//	waypointMoveUpdate(); //use waypoint to set tv and rv
//}*/
//
///*
// * For hard coding distance
// */
//int nbrRangeLookUp(uint8 myID, uint8 nbrID){
//	int Range = 500;
//	switch(myID){
//	case 8:{
//		switch(nbrID){
//			case 9:{
//				Range = 300;
//				break;
//			}
//			case 10:{
//				Range = 600;
//				break;
//			}
//			case 11:{
//				Range = 900;
//				break;
//			}
//			case 15:{
//				Range = 1200;
//				break;
//			}
//			case 17:{
//				Range = 1500;
//				break;
//			}
//			case 20:{
//				Range = 1800;
//				break;
//			}
//			case 24:{
//				Range = 2100;
//				break;
//			}
//		}
//		break;
//	}
//	case 9:{
//		switch(nbrID){
//			case 8:{
//				Range = 300;
//				break;
//			}
//			case 10:{
//				Range = 300;
//				break;
//			}
//			case 11:{
//				Range = 600;
//				break;
//			}
//			case 15:{
//				Range = 900;
//				break;
//			}
//			case 17:{
//				Range = 1200;
//				break;
//			}
//			case 20:{
//				Range = 1500;
//				break;
//			}
//			case 24:{
//				Range = 1800;
//				break;
//			}
//		}
//		break;
//	}
//	case 10:{
//		switch(nbrID){
//			case 8:{
//				Range = 600;
//				break;
//			}
//			case 9:{
//				Range = 300;
//				break;
//			}
//			case 11:{
//				Range = 300;
//				break;
//			}
//			case 15:{
//				Range = 600;
//				break;
//			}
//			case 17:{
//				Range = 900;
//				break;
//			}
//			case 20:{
//				Range = 1500;
//				break;
//			}
//			case 24:{
//				Range = 1800;
//				break;
//			}
//		}
//		break;
//	}
//	case 11:{
//		switch(nbrID){
//			case 8:{
//				Range = 900;
//				break;
//			}
//			case 9:{
//				Range = 600;
//				break;
//			}
//			case 10:{
//				Range = 300;
//				break;
//			}
//			case 15:{
//				Range = 300;
//				break;
//			}
//			case 17:{
//				Range = 900;
//				break;
//			}
//			case 20:{
//				Range = 1200;
//				break;
//			}
//			case 24:{
//				Range = 1500;
//				break;
//			}
//		}
//		break;
//	}
//	case 15:{
//		switch(nbrID){
//			case 8:{
//				Range = 1200;
//				break;
//			}
//			case 9:{
//				Range = 900;
//				break;
//			}
//			case 10:{
//				Range = 600;
//				break;
//			}
//			case 11:{
//				Range = 300;
//				break;
//			}
//			case 17:{
//				Range = 300;
//				break;
//			}
//			case 20:{
//				Range = 600;
//				break;
//			}
//			case 24:{
//				Range = 900;
//				break;
//			}
//		}
//		break;
//	}
//	case 17:{
//		switch(nbrID){
//			case 8:{
//				Range = 1500;
//				break;
//			}
//			case 9:{
//				Range = 1200;
//				break;
//			}
//			case 10:{
//				Range = 900;
//				break;
//			}
//			case 11:{
//				Range = 600;
//				break;
//			}
//			case 15:{
//				Range = 300;
//				break;
//			}
//			case 20:{
//				Range = 300;
//				break;
//			}
//			case 24:{
//				Range = 600;
//				break;
//			}
//		}
//		break;
//	}
//	case 20:{
//		switch(nbrID){
//			case 8:{
//				Range = 1800;
//				break;
//			}
//			case 9:{
//				Range = 1500;
//				break;
//			}
//			case 10:{
//				Range = 1200;
//				break;
//			}
//			case 11:{
//				Range = 900;
//				break;
//			}
//			case 15:{
//				Range = 600;
//				break;
//			}
//			case 17:{
//				Range = 300;
//				break;
//			}
//			case 24:{
//				Range = 300;
//				break;
//			}
//		}
//		break;
//	}
//	case 27:{
//		switch(nbrID){
//			case 8:{
//				Range = 2100;
//				break;
//			}
//			case 9:{
//				Range = 1800;
//				break;
//			}
//			case 10:{
//				Range = 1500;
//				break;
//			}
//			case 11:{
//				Range = 1200;
//				break;
//			}
//			case 15:{
//				Range = 900;
//				break;
//			}
//			case 17:{
//				Range = 600;
//				break;
//			}
//			case 20:{
//				Range = 300;
//				break;
//			}
//		}
//		break;
//	}
//	}
//	return Range+50;
//}
