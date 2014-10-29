/*
 * conensusPipeline.c
 *
 * Generic abstracted pipeline consensus methods.
 *
 *  Created on: Oct 27, 2014
 *      Author: zkk
 */

#include "consensus.h"

NbrData consensusPipelineHead;		// The current head, or most recent
NbrData consensusPipelineCount;		// The current element count in the pipeline
uint8 consensusPipelineSize;		// The maximum size of the pipeline
uint32 consensusPipelineRound;		// Round check to know when to input

/* User provided function to input new pipeline data to the neighbor data at the
 * specified index. */
void (*consensusPipelineCellInput)(uint8 index);

/* User provided function to store data from a neighbor to a temporary buffer.
 * Moves the index locations to correspond to our pipeline. */
void (*consensusPipelineCellStoreTempData)(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex);

/* Does consensus on the specified index of our pipeline to the temporary buffer
 * at the same index. */
void (*consensusPipelineCellOperation)(uint8 index);

/**
 * Outputs the contents of the pipeline using a user defined cell output function.
 * Iterates over the entire pipeline starting at the head and calls the provided
 * function.
 *
 * @param void (*printFunction)(uint8 index)
 * 		Function called on each element of the pipeline, should print out or
 * 		display data about it
 */
void consensusPipelinePrintPipeline(void (*printFunction)(uint8 index)) {
	/* Get values from neighbor data */
	uint8 myHead = nbrDataGet(&consensusPipelineHead);
	uint8 myCount = nbrDataGet(&consensusPipelineCount);

	/* Iterate over pipeline */
	uint8 i;
	for (i = 0; i < myCount; i++) {
	    uint8 index = (myHead + i) % consensusPipelineSize;

	    /* Call the provided print function */
	    (*printFunction)(index);
	}
}

/**
 * Function called at the beginning of each round. Handles input into the
 * pipeline and the rolling aspects and count incrementing in the pipeline.
 */
void consensusPipelineNextRound(void) {
	/* Only input if a new state has occurred */
	if (!consensusNewStateCheck(&consensusPipelineRound)) {
		return;
	}

	/* Decrement the head and set it */
	uint8 newHead = nbrDataGet(&consensusPipelineHead);
	newHead = (newHead + consensusPipelineSize - 1) % consensusPipelineSize;
	nbrDataSet(&consensusPipelineHead, newHead);

	/* Input new value into head */
	consensusPipelineCellInput(newHead);

	/* Increment the count if we are less than full */
	uint8 count = nbrDataGet(&consensusPipelineCount);
	count++;
	if (count <= consensusPipelineSize) {
		nbrDataSet(&consensusPipelineCount, count);
	}
}

/**
 * Consensus operation on successful gossip. Iterates over the pipeline and
 * calls the cell consensus function intialized by the user on each cell.
 * Predicated on the fact that consensusPipelineStoreTempData was called
 * before it, as the cellOperation should use the temporary data storage array.
 */
void consensusPipelineOperation(void) {
	/* Get values from neighbor data */
	uint8 myHead = nbrDataGet(&consensusPipelineHead);
	uint8 myCount = nbrDataGet(&consensusPipelineCount);

	/* Iterate over pipeline */
	uint8 i;
	for (i = 0; i < myCount; i++) {
	    uint8 index = (myHead + i) % consensusPipelineSize;

	    /* Perform consensus with corresponding stored values in the similarly
	     * indexed temporary pipeline */
	    consensusPipelineCellOperation(index);
	}
}

/**
 * Stores data into temporary buffer at corresponding index locations to our
 * pipeline. Calls the user provided cellStoreTempData to store data in
 * locations in the temporary storage array.
 *
 * @param nbrPtr
 * 		The neighbor pointer to look up the data from
 */
void consensusPipelineStoreTempData(Nbr *nbrPtr) {
	/* Get values from neighbor data */
	uint8 myHead = nbrDataGet(&consensusPipelineHead);
	uint8 theirHead = nbrDataGetNbr(&consensusPipelineHead, nbrPtr);
	uint8 myCount = nbrDataGet(&consensusPipelineCount);
	uint8 theirCount = nbrDataGetNbr(&consensusPipelineCount, nbrPtr);

	/* Iterate through your entire pipeline, storing relevant data in relative
	 * locations */
	uint8 i = 0;
	uint8 j = 0;
	for (; i < myCount; i++) {
	    uint8 destIndex = (myHead + i) % consensusPipelineSize;
	    uint8 srcIndex = (theirHead + j) % consensusPipelineSize;

	    /* Store the data from the neighbor into locations in the temporary
	     * storage that correspond to our indices in our pipeline. */
	    consensusPipelineCellStoreTempData(nbrPtr, srcIndex, destIndex);

	    /* Only increment their count if it is less than their size */
	    if (j < theirCount) {
	    	j++;
	    }
	}
}

/**
 * Gets the last index in the pipeline
 *
 * @return The last (oldest) index in the pipeline.
 */
uint8 consensusPipelineGetOldestIndex(void) {
	uint8 head = nbrDataGet(&consensusPipelineHead);
	uint8 count = nbrDataGet(&consensusPipelineCount);

	return (head + count - 1) % consensusPipelineSize;
}

/**
 * Initialize the pipeline. Also initializes the consensus subroutine.
 *
 * Requires the user to instantiate their own neighbor data, temporary data
 * pipeline, and functions that act on the data. This functions initializes
 * the consensus subroutine using functions that iterate over the pipeline
 * received from other robots.
 *
 * @param size
 * 		The maximum size of the pipeline array.
 *
 * @param void (*cellInput)(uint8 index)
 * 		A function that inserts the input value for the pipeline into the cell
 * 		of the pipeline array given by the index parameter to the function.
 *
 * 	@param void (*cellStoreTempData)(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex)
 * 		A function that stores a neighbors data that we are going to gossip
 * 		with into a temporary storage array provided by the user. The function
 * 		provided should accept the pointer to the neighbor, nbrPtr, a source
 * 		index from the neighbor's pipeline to copy data from, and a destination
 * 		index where the data should be stored in the temporary storage array.
 * 		This will look something like this:
 * 		tempValue[destIndex] = nbrDataGetNbr(&value[srcIndex], nbrPtr);
 *
 * 	@param void (*cellOperation)(uint8 index))
 * 		The function called upon a successful gossip. This should update the
 * 		value in the index of your pipeline using data from the temporary
 * 		data array at the same index.
 */
void consensusPipelineInit(uint8 size,
		void (*cellInput)(uint8 index),
		void (*cellStoreTempData)(Nbr *nbrPtr, uint8 srcIndex, uint8 destIndex),
		void (*cellOperation)(uint8 index)) {

	/* Set the size of the pipeline */
	consensusPipelineSize = size;

	/* Create our neighbor data */
	nbrDataCreate(&consensusPipelineHead, "cpHead", 8, 0);
	nbrDataCreate(&consensusPipelineCount, "cpCount", 8, 0);

	/* Initialize all the provided function pointers */
	consensusPipelineCellInput = cellInput;
	consensusPipelineCellStoreTempData = cellStoreTempData;
	consensusPipelineCellOperation = cellOperation;

	/* Set the beginning round operation for consensus subtask */
	consensusSetRoundOperation(consensusPipelineNextRound);

	/* Initalize the consensus subtask */
	consensusInit(consensusPipelineStoreTempData, consensusPipelineOperation);
}
