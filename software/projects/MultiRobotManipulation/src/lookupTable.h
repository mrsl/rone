/*
 * lookupTable.h
 *
 *  Created on: Aug 28, 2014
 *      Author: Golnaz
 */

#ifndef LOOKUPTABLE_H_
#define LOOKUPTABLE_H_

#include "roneos.h"
#include "ronelib.h"

struct {
	int16 distance;
} typedef robotLookup;


extern int lookupTableIndex;
extern int lookupTableLookup[10];
extern robotLookup lookupTable[10][10];

void setLookup(uint8 myId, uint8 theirId, int16 distance);
void clearLookup();
uint8 getLookupIndex(uint8 id);
int16 lookupGetDistance(uint8 myId, uint8 theirId);

#endif /* LOOKUPTABLE_H_ */
