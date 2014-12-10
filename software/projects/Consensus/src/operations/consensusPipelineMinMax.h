/*
 * consensusPipelineMinMax.h
 *
 *  Created on: Dec 10, 2014
 *      Author: Golnaz
 */

#ifndef CONSENSUSPIPELINEMINMAX_H_
#define CONSENSUSPIPELINEMINMAX_H_


void consensusPipelineMinMaxGetCentroid(float *x, float *y);
void consensusPipelineMinMaxGetPosDiff(float *x);
void consensusPipelineMinMaxGetPosMult(float *x);


void consensusPipelineMinMaxSetPosDiff(float newPosDiff);

void consensusPipelineMinMaxSetPosMult(float newPosMult);


void consensusPipelineMinMaxInit(void);

#endif /* CONSENSUSPIPELINEMINMAX_H_ */
