/*
 * centroidLite.h
 *
 *  Created on: Jan 16, 2015
 *      Author: Zak
 */

#ifndef CENTROIDLITE_H_
#define CENTROIDLITE_H_

#include "roneos.h"
#include "ronelib.h"
#include "../util/centroidData.h"

int32 centroidLiteGetDistance();

int32 centroidLiteGetBearing();

centroidValue centroidLiteGetXCoordinate();
centroidValue centroidLiteGetYCoordinate();

void centroidLiteInit();

#endif /* CENTROIDLITE_H_ */
