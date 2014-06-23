/*
 * NbrDataCoM.c
 *
 *  Created on: Dec 23, 2013
 *      Author: Golnaz Habibi
 */


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "roneos.h"
#include "ronelib.h"

#include "nbrDataCoM.h"


 ////    Functions for CoM Data  /////////////


/*
 * @brief Create nbrdata for the shared data of CoM , x position of CoM.
*/

void nbrDataCoMCreate(NbrDataCoM* nbrCoMDataPtr, const char * name) {
	nbrDataCreateFloat(&nbrCoMDataPtr->xpos, name);
	nbrDataCreateFloat(&nbrCoMDataPtr->ypos, name);
	nbrDataCreate(&nbrCoMDataPtr->nonce, name, 8, 0);
	nbrDataCreate(&nbrCoMDataPtr->req_ID, name, 8, ROBOT_ID_NULL);
	nbrDataCreate(&nbrCoMDataPtr->ack_ID, name, 8, ROBOT_ID_NULL);
}



/*
 * @brief send x position of CoM.
*/
void nbrDataCoMSetPos(NbrDataCoM* nbrCoMDataPtr, float xvalue, float yvalue ){
	 nbrDataSetFloat(&nbrCoMDataPtr->xpos, xvalue);
	 nbrDataSetFloat(&nbrCoMDataPtr->ypos, yvalue);


}




/*
 * @brief send  ID of receiver.
*/
void nbrDataCoMSetAckID(NbrDataCoM* nbrCoMDataPtr, uint8 ackValue){
	nbrDataSet(&nbrCoMDataPtr->ack_ID, ackValue);
}

/*
 * @brief send ID of sender.
*/
void nbrDataCoMSetReqID(NbrDataCoM* nbrCoMDataPtr, uint8 reqValue){
	nbrDataSet(&nbrCoMDataPtr->req_ID, reqValue);

}


void nbrDataCoMSetNonce(NbrDataCoM* nbrCoMDataPtr, uint8 nonceValue){
	nbrDataSet(&nbrCoMDataPtr->nonce, nonceValue);

}

/*
 * @brief receive x position of CoM.
*/
float nbrDataCoMGetx(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr){
	  return	nbrDataGetNbrFloat(&nbrCoMDataPtr->xpos, nbrPtr);

}


/*
 * @brief receive y position of CoM.
*/
float nbrDataCoMGety(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr){
	  return	nbrDataGetNbrFloat(&nbrCoMDataPtr->ypos, nbrPtr);
}

/*
 * @brief receive ID of the sender.
*/
uint8 nbrDataCoMGetReqID(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr){

	return	nbrDataGetNbr(&nbrCoMDataPtr->req_ID, nbrPtr);

}

/*
 * @brief receive ID of the target receiver.
*/
uint8 nbrDataCoMGetAckID(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr){
	return	nbrDataGetNbr(&nbrCoMDataPtr->ack_ID, nbrPtr);

}


uint8 nbrDataCoMGetNonce(NbrDataCoM* nbrCoMDataPtr, Nbr* nbrPtr){

	return	nbrDataGetNbr(&nbrCoMDataPtr->nonce, nbrPtr);

	}




			/*const uint16 RangLookupData[11][11] = {
		{0  , 125, 248, 297, 363, 441, 521, 542, 408, 275, 142   },
		{125, 0  , 145, 229, 335, 438, 545, 602, 449, 302, 172   },
		{248, 145, 0  , 112,  245,362 , 492, 587, 424  , 282 , 203},
		{297, 229, 112,  0 , 136,  255, 391, 503, 339, 213 , 195 },
		{363, 335, 245, 136,  0 ,  119, 258, 387, 229, 152, 226  },
		{441, 438 ,362 , 255, 119  ,  0 , 143, 292, 159,176, 299 },
		{521, 545, 492, 391 , 258,  143,  0, 171, 126, 248, 385},
		{542, 602, 587, 503, 387,  292, 171 , 0, 164, 308,  430},
		{408, 449, 424, 339 , 229, 159 ,126, 164,  0 ,  148, 280},
		{275, 302, 282, 213 ,152,176 , 248, 308  , 148 ,  0, 137},
		{142, 172, 203, 195, 226, 299, 385, 430, 280, 137,  0 },

};*/

///    ********************* CoM Experiment *******************************

/*const uint16 RangLookupData[7][7] = {    //  for CoM change this array   // 7 is the number of robots

		{0, 140, 250, 460, 570, 460, 190},
		{140, 0, 130, 390, 510, 440, 200},
		{250, 130, 0, 260, 400, 340, 180},
		{460, 390, 260,0, 150, 150, 290},
		{570, 51, 40, 150, 0, 130, 390},
		{460, 440, 340, 150,130, 0, 280},
		{190, 200, 180, 290,390,280, 0},

};*/

/*Experiment 1: 4 robots*/
const uint16 RangLookupData[4][4] = {
		{0, 298, 340, 170},
		{298, 0, 170, 340},
		{340, 170, 0, 298},
		{170, 340, 298, 0},
};


///    ********************* CoM Experiment *******************************

 const int32 robotidx[4] = {71, 74, 79, 68}; //  for CoM change this array , change based on the order of robot you use


//const int32 robotidx[11] = {80, 60, 65, 67, 69, 68, 71, 79, 81, 82, 83};


int16 getnbrRange(int32 id1, int32 id2)

{
	int j =0;
	int i = 0;
	for (i=0;i<11;i++) // 11 is the number of robots
	{
		for (j = 0 ;j<11; j++)  // 11 is the number of robots
		{

			if( ( id1 == robotidx[i] &&  id2 == robotidx[j] ) || ( id2 == robotidx[i] &&  id1 == robotidx[j] ))
			{
				return RangLookupData[i][j];
			}
		}
		}
	return 0;

}






