/*
 * lookupTable.c
 *
 *  Created on: Aug 28, 2014
 *      Author: Golnaz
 */

#include "lookupTable.h"

int lookupTableIndex = 0;
int lookupTableLookup[GLOBAL_ROBOTLIST_MAX_SIZE];
robotLookup lookupTable[GLOBAL_ROBOTLIST_MAX_SIZE][GLOBAL_ROBOTLIST_MAX_SIZE];

void setLookup(uint8 myId, uint8 theirId, int16 distance) {
	uint8 myIndex = getLookupIndex(myId);
	uint8 theirIndex = getLookupIndex(theirId);

	lookupTable[myIndex][theirIndex].distance = distance;

	lookupTable[theirIndex][myIndex].distance = distance;
}

void clearLookup() {
	lookupTableIndex = 0;
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
