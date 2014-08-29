/*
 * lookupTable.h
 *
 *  Created on: Aug 28, 2014
 *      Author: Golnaz
 */

#ifndef LOOKUPTABLE_H_
#define LOOKUPTABLE_H_

#include "roneos.h"

struct {
	int16 distance;
	int32 orientation;
	int32 bearing;
} typedef robotLookup;


extern int lookupTableIndex;
extern int lookupTableLookup[10];
extern robotLookup lookupTable[10][10];

void setLookup(uint8 myId, uint8 theirId, int16 distance, int32 orientation, int32 bearing);
uint8 getLookupIndex(uint8 id);
int16 lookupGetDistance(uint8 myId, uint8 theirId);
int32 lookupGetOrientation(uint8 myId, uint8 theirId);
int32 lookupGetBearing(uint8 myId, uint8 theirId);

#endif /* LOOKUPTABLE_H_ */
