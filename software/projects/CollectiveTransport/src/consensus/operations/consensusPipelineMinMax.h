/*
 * consensusPipelineMinMax.h
 *
 *  Created on: Dec 10, 2014
 *      Author: Zak
 */

#ifndef CONSENSUSPIPELINEMINMAX_H_
#define CONSENSUSPIPELINEMINMAX_H_


void consensusPipelineMinMaxGetCentroid(float *x, float *y);
void consensusPipelineMinMaxGetPosDiff(float *x);
void consensusPipelineMinMaxGetPosMult(float *x);
void consensusPipelineMinMaxGetWidth(float *x);
void consensusPipelineMinMaxGetDiameter(float *x);


void consensusPipelineMinMaxSetPosDiff(float newPosDiff);

void consensusPipelineMinMaxSetPosMult(float newPosMult);

void consensusPipelineMinMaxSetWidth(float newWidth);

void consensusPipelineMinMaxSetDiameter(float newDiameter);

void consensusPipelineMinMaxInit(void);






#endif /* CONSENSUSPIPELINEMINMAX_H_ */
