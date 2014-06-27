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


void creatGlobalTreeCOMList(PosistionCOM* posListPtr);
void updateGlobalTreeCOM(GlobalRobotList globalRobotList, NbrList nbrList, PosistionCOM* posListPtr, int Range);

#endif /* GLOBALTREECOM_H_ */
