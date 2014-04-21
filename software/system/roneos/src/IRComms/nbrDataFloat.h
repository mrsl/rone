/*
 * nbrDataFloat.h
 *
 *  Created on: Sep 18, 2013
 *      Author: Golnaz
 */

#ifndef NBRDATAFLOAT_H_
#define NBRDATAFLOAT_H_

void nbrDataCreateFloat(NbrDataFloat* msgPtr, const char* name);
void nbrDataSetFloat(NbrDataFloat* msgPtr, float val);
float nbrDataGetNbrFloat(NbrDataFloat* msgPtr, Nbr* nbrPtr);
float nbrDataGetFloat(NbrDataFloat* Float);

#endif /* NBRDATAFLOAT_H_ */
