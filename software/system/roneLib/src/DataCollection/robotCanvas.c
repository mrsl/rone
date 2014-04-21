/*
 * @file robotCanvas.c
 *
 * @since Apr 23, 2011
 * @author jamesm
 */


#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

void rcNewPage(void) {
	cprintf("rc,newPage\n");
}

//void rcUpdate(void) {
//	cprintf("nd,end\n");
//}

void rcCommand(char* command) {
	cprintf("rc,%s\n", command);
}

void rcUpdatePage(void) {
	cprintf("rc,updatePage\n");
}

void rcSetLayer(uint8 layer) {
	cprintf("rc,setLayer,%d\n",layer);
}

void rcValue(char* name, uint32 val) {
	cprintf("rc,val,%s,%d\n", name, val);
}


void rcNbrPrint(Nbr* nbr, uint8 round) {
	uint8 i;

	if(nbr) {
		cprintf("nd,%d,%d,%d,%d,%d,%d,%d,%d",
			roneID,
			osTaskGetTickCount(),
			round,
			nbr->ID,
			nbr->bearing,
			nbr->orientation,
			nbr->updateTime,
			//nbr->nbrnbrsSize
			nbrDataCount() - 1);
		if (nbrDataCount() > 1) {
			cprintf(",");
			nbrDataPrintNbr(nbr);
		}
		cprintf("\n");
	}
}


void rcNbrPrintAll(void) {
	uint8 i;
	NbrList nbrList;

	nbrListCreate(&nbrList);
	for (i = 0; i < nbrList.size; i++) {
		rcNbrPrint(nbrList.nbrs[i], neighborsGetRound());
	}
}



void rcPosePrint(char* name, uint8 id, Pose* posePtr) {
	if (posePtr) {
		cprintf("r,%s,%d,%d,%d,%d\n",
				name,
				id,
				posePtr->x,
				posePtr->y,
				posePtr->theta);
	}
}




void rcNbrVarsPrint(Nbr* nbr, uint8 round) {
	uint8 i;

	if(nbr) {
		cprintf("nv,%d,%d,", nbr->ID, round);
		nbrDataPrintNbr(nbr);
	}
}


void rcNbrVarsPrintAll(void) {
	uint8 i;
	NbrList nbrList;

	nbrListCreate(&nbrList);
	for (i = 0; i < nbrList.size; i++) {
		rcNbrVarsPrint(nbrList.nbrs[i], neighborsGetRound());
	}
}


void rcNbrVarsPrintHeader(void) {
	cprintf("nvh,");
	nbrDataPrintHeaders();
}



////TODO this is made up after I moved the scale-free code to the scale-free project
//#define SCALE_FREE_COEFF_SCALER 1
//
///* to print to Data Collection system over a wired connection */
//void nbrPrintDataSF(Nbr* nbr, uint32 round) {
//	uint8 i;
//	static boolean printedHeader = FALSE;
//	if (!printedHeader) {
//		cprintf("id,round,nbrID,bearing,coeff,nbrNbrSize, time, nbrNbrID,nbrNbrBearing,...\n");
//		printedHeader = TRUE;
//	}
//	int coeff = (int) (nbr->scaleFreeCoeff * SCALE_FREE_COEFF_SCALER);
//	cprintf("ndSF,%d,%d,%d,%d,%d,",
//			roneID,round,nbr->ID,nbr->bearing,coeff);
//	cprintf("%d,%d",  nbr->nbrnbrsSize,osTaskGetTickCount());
//
//	for (i = 0; i < nbr->nbrnbrsSize; ++i) {
//		cprintf(",%d,%d",nbr->nbrnbrs[i].ID, nbr->nbrnbrs[i].bearing);
//	}
//
//	cprintf("\n");
//}


