/*
 * centroidData.h
 *
 *  Created on: Nov 11, 2014
 *      Author: Zak
 */

#ifndef CENTROIDDATA_H_
#define CENTROIDDATA_H_

/* Typedefs */
typedef float centroidValue;
#define CENTROID_VALUE_SIZE	4

typedef struct centroidNbrValue centroidNbrValue;
typedef union centroidValueConvert centroidValueConvert;
typedef struct centroidNbrData centroidNbrData;
typedef struct centroidData	centroidData;

/* Structs */
struct centroidNbrValue {
	NbrData data[CENTROID_VALUE_SIZE];
};

union centroidValueConvert {
	centroidValue val;
	uint8 bytes[CENTROID_VALUE_SIZE];
};

struct centroidNbrData {
	centroidNbrValue x;
	centroidNbrValue y;
};

struct centroidData {
	centroidValue x;
	centroidValue y;
};

void centroidNbrDataCreate(centroidNbrData *data);
void centroidNbrDataSet(centroidValue x, centroidValue y, centroidNbrData *data);
void centroidNbrDataGet(centroidValue *x, centroidValue *y, centroidNbrData *data);
void centroidNbrDataGetNbr(centroidValue *x, centroidValue *y, centroidNbrData *data, Nbr *nbrPtr);

void centroidNbrDataCopy(centroidData *toCopy, centroidNbrData *toHere);
void centroidNbrDataPaste(centroidNbrData *toPaste, centroidData *toHere);
void centroidNbrDataPasteNbr(centroidNbrData *toPaste, centroidData *toHere, Nbr *nbrPtr);

void centroidDataSet(centroidValue x, centroidValue y, centroidData *data);
void centroidDataGet(centroidValue *x, centroidValue *y, centroidData *data);
void centroidDataCopy(centroidData *toCopy, centroidData *toHere);

void centroidDataIIR(centroidData *new, centroidData *old, centroidValue alpha);

void centroidTransform(centroidValue *x, centroidValue *y, Nbr *nbrPtr);

#endif /* CENTROIDDATA_H_ */
