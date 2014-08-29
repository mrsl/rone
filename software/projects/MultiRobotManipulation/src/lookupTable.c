/*
 * lookupTable.c
 *
 *  Created on: Aug 28, 2014
 *      Author: Golnaz
 */

#include "lookupTable.h"

int lookupTableIndex = 0;
int lookupTableLookup[10];
robotLookup lookupTable[10][10];

void setLookup(uint8 myId, uint8 theirId, int16 distance, int32 bearing, int32 orientation) {
	uint8 myIndex = getLookupIndex(myId);
	uint8 theirIndex = getLookupIndex(theirId);

	lookupTable[myIndex][theirIndex].distance = distance;
	lookupTable[myIndex][theirIndex].orientation = orientation;
	lookupTable[myIndex][theirIndex].bearing = bearing;

	lookupTable[theirIndex][myIndex].distance = distance;
	lookupTable[theirIndex][myIndex].orientation = bearing;
	lookupTable[theirIndex][myIndex].bearing = orientation;
}

void printLookupTable() {
	int i, j;
	for (i = 0; i < lookupTableIndex; i++) {
		for (j = 0; j < lookupTableIndex; j++) {
			cprintf("%5d,%5d,%5d | ", lookupTable[i][j].distance, lookupTable[i][j].bearing, lookupTable[i][j].orientation);
		}
		cprintf("\n");
	}
}

uint8 getLookupIndex(uint8 id) {
	int i;
	boolean inTable;
	uint8 index;

	inTable = FALSE;
	for (i = 0; i < lookupTableIndex; i++) {
		if (lookupTableLookup[i] == id) {
			inTable = TRUE;
			index = i;
			break;
		}
	}
	if (!inTable) {
		index = lookupTableIndex;
		lookupTableLookup[lookupTableIndex++] = id;
	}

	return index;
}

int16 lookupGetDistance(uint8 myId, uint8 theirId) {
	uint8 myIndex = getLookupIndex(myId);
	uint8 theirIndex = getLookupIndex(theirId);

	return lookupTable[myIndex][theirIndex].distance;
}

int32 lookupGetOrientation(uint8 myId, uint8 theirId) {
	uint8 myIndex = getLookupIndex(myId);
	uint8 theirIndex = getLookupIndex(theirId);

	return lookupTable[myIndex][theirIndex].orientation;
}

int32 lookupGetBearing(uint8 myId, uint8 theirId) {
	uint8 myIndex = getLookupIndex(myId);
	uint8 theirIndex = getLookupIndex(theirId);

	return lookupTable[myIndex][theirIndex].bearing;
}
