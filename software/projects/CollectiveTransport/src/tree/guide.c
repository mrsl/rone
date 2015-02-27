/*
 * guide.c
 *
 *  Created on: Jan 17, 2015
 *      Author: Golnaz
 */

#include "guide.h"

NbrData isGuide;
NbrData hops;
NbrData type;

//NbrData transferType;

boolean guideIsGuide() {
	return nbrDataGet(&isGuide);
}

boolean guideNbrIsGuide(Nbr *nbrPtr) {
	return nbrDataGetNbr(&isGuide, nbrPtr);
}

void guideSetGuide(boolean val) {
	nbrDataSet(&isGuide, val);
}

void guideSetHops(uint8 myHops) {
	nbrDataSet(&hops, myHops);
}

uint8 guideGetGuideHops(Nbr *nbrPtr) {
	if (guideNbrIsGuide(nbrPtr)) {
		return nbrDataGetNbr(&hops, nbrPtr);
	} else {
		return MAX_HOPS;
	}
}

uint8 guideGetMyHops() {
	return nbrDataGet(&hops);
}

void guideSetType(uint8 toType) {
	nbrDataSet(&type, toType);
}

uint8 guideGetType() {
	return nbrDataGet(&type);
}

uint8 guideGetNbrType(Nbr *nbrPtr) {
	if (guideNbrIsGuide(nbrPtr)) {
		return nbrDataGetNbr(&type, nbrPtr);
	} else {
		return GUIDE_G;
	}
}

//uint8 guideNbrGetTransferType(Nbr *nbrPtr) {
//	if (guideNbrIsGuide(nbrPtr)) {
//		return nbrDataGetNbr(&transferType, nbrPtr);
//	} else {
//		return GUIDE_STOP;
//	}
//}

Nbr *guideGetNextGuide(NbrList *nbrListPtr) {
	Nbr *guide = NULL;
	uint8 hops = MAX_HOPS;

	uint8 i;

	// Find guide robot
	for (i = 0; i < nbrListPtr->size; i++) {
		Nbr *nbrPtr = nbrListGetNbr(nbrListPtr, i);

		// Found a guide
		if (guideNbrIsGuide(nbrPtr)) {
			uint8 tHops = guideGetGuideHops(nbrPtr);

			if (tHops < hops) {
				guide = nbrPtr;
				hops = tHops;
			}
		}
	}

	return guide;
}

//void guideSetTransferType(uint8 type) {
//	nbrDataSet(&transferType, type);
//}
//
//void guideGetTransferType(uint8 myType, uint8 theirType) {
//	if (myType == GUIDE_G) {
//		guideSetTransferType(GUIDE_STOP);
//	} else if (myType == GUIDE_S) {
//		if (theirType == GUIDE_G) {
//			guideSetTransferType(GUIDE_S_2_S);
//		} else if (theirType == GUIDE_S) {
//			guideSetTransferType(GUIDE_S_2_S);
//		} else if (theirType == GUIDE_SM) {
//			guideSetTransferType(GUIDE_S_2_SM);
//		}
//	} else if (myType == GUIDE_SM) {
//		if (theirType == GUIDE_G) {
//			guideSetTransferType(GUIDE_S_2_S);
//		} else if (theirType == GUIDE_S) {
//			guideSetTransferType(GUIDE_S_2_S);
//		} else if (theirType == GUIDE_SM) {
//			guideSetTransferType(GUIDE_S_2_SM);
//		}
//	}
//}

// Need to search for guide robot
//void guideCallback(NbrDatabase* ndPtr) {
//	NbrList nbrList;
//	nbrListCreate(&nbrList);
//
//	Nbr *nextGuide = guideGetNextGuide(&nbrList);
//
//	uint8 type;
//	uint8 myType = guideGetType();
//
//	if (nextGuide != NULL) {
//		type = guideGetNbrType(nextGuide);
//	} else {
//		type = GUIDE_G;
//	}
//
//	guideSetTransferType(myType, type);
//}

boolean guideEn = FALSE;

boolean guideEnabled() {
	return guideEn;
}

void guideInit() {
	nbrDataCreate(&isGuide, "isGuide", 1, 0);
	nbrDataCreate(&hops, "hops", 4, MAX_HOPS);
	nbrDataCreate(&type, "type", 2, GUIDE_G);

	guideEn = TRUE;
//	nbrDataCreate(&transferType, "type", 2, GUIDE_STOP);

//	neighborsAddReceiveCallback(guideCallback);
}
