/*
 * robotList.h
 *
 *  Created on: Apr 14, 2014
 *      Author: mrsl
 */

#ifndef ROBOTLIST_H_
#define ROBOTLIST_H_

#define ROBOTLIST_MAX_SIZE 10


typedef struct nbrDataRobotElement {
	NbrData ID;
	NbrData nonce;
	uint32 nonceTimestamp;
} nbrDataRobotElement;


typedef struct nbrDataRobotList {
	uint8 size;
	uint8 manipulationRobotCount;
	nbrDataRobotElement robotList[ROBOTLIST_MAX_SIZE];
}nbrDataRobotList;

#endif /* ROBOTLIST_H_ */
