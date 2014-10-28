/*
 * consensusPipeline.h
 *
 *  Created on: Oct 28, 2014
 *      Author: zkk
 */

#ifndef SRC_CONSENSUSPIPELINE_H_
#define SRC_CONSENSUSPIPELINE_H_

#include "consensus.h"

void consensusPipelinePrintPipeline(void (*printFunction)(uint8 index));

uint8 consensusPipelineGetOldestIndex(void);

void consensusPipelineInit(uint8 size,
		void (*cellInput)(uint8 index),
		void (*cellStoreTempData)(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex),
		void (*cellOperation)(uint8 index));

#endif /* SRC_CONSENSUSPIPELINE_H_ */
