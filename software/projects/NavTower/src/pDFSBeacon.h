/*
 * ir_test.h
 *
 *  Created on: Jun 26, 2012
 *      Author: Divya
 */

#ifndef IR_TEST_H_
#define IR_TEST_H_

#define NEIGHBOR_ROUND_PERIOD		200

#define ROOT_ROBOT					0
#define JUNCTION_ROBOT 				1
#define	EXPLORER_ROBOT				2
#define	DEAD_END_ROBOT				3
#define JUNCTION_BEACON             4
#define DEAD_END_BEACON             5

#define MAX_HOPS                    7                // max number that the number of bits for hops can hold


typedef struct robotMessage {

	boolean transitioning;

	NbrMsgField msgSourceIDJunction;
	NbrMsgField msgHopsJunction;
	NbrMsgField msgJunctionTimeStamp;

   // NbrMsgField msgSourceIDRoot;
   // NbrMsgField msgHopsRoot;

	NbrMsgField msgRobotType;

	NbrMsgField deadEndID;
	NbrMsgField deadEndValid;
} robotMessage;


#endif /* IR_TEST_H_ */
