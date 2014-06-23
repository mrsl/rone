/*
 * mrm.c
 *
 * Created on: July 31st, 2012
 *     Author: Divya Bhat
 *
 *     TODO: K_PROPORTIONAL has been adjusted. Now K_INTEGRAL has to be adjusted (as you pick a high value and bring it down to stabilize
 *     		the robot, settle on one, and decrease K_PROPORTIONAL slightly if necessary). 0.001 may be the right value (the errors get very large).
 *
 *			Don't bother testing them while stationary-- it doesn't look right even with exact SuperDemo code.
 *     To get a better K_INTEGRAL_MAX, print out the integral values and see what range they tend to fall in. Pick a max based on that.
 *
 *
 */

//Behavior: move forward; if robot bumps into something, enter into flock behavior


#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD		300

#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50

#define K_INTEGRAL 					0
#define K_INTEGRAL_MAX				400 //400 works, 300 doesn't
#define K_TV						110 //110 WORKS IN WEIGHTED TEST
#define K_PROP						1
#define K_RV						1

#define OBJECT_MASS					778

boolean isBumped = FALSE;

void behaviorTask(void *parameters);
void backgroundTask(void *parameters);
int32 behController(Beh* behPtr, int32 theta, int32 integralError, uint32 deltaTicks);
int32 behFrictionFlock(Beh* behPtr, NbrList* nbrListPtr, int32 tv, int32 integralError, uint32 currentTime, uint32 prevTime);

int main(void) {
	volatile uint32 val1, val2;

	systemPreInit();
	systemInit();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	neighborsXmitEnable(TRUE);
	val1 = osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	val2 = osTaskCreate(backgroundTask, "background", 1024, NULL, BACKGROUND_TASK_PRIORITY);

	if ((val1 != pdTRUE)) {
		cprintf("could not create a task");
	}

	/* Start the scheduler. */
	osTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
   	 task. This is a very bad thing, and should not happen.*/
	return 0;
}


// the background task runs all the time
void backgroundTask(void* parameters) {
	for (;;) {
		// delay to let other tasks run at same priority
		osTaskDelay(100);
	}
}


/*
int32* getNetworkArray(NbrList* nbrListPtr){
	int32 i, x = 0;
	int32* nbrArray;
	for (i = 0; i < nbrListPtr->size; i++){
		nbrPtr = nbrListPtr-> nbrs[i];
		if (nbrPtr-> orientationValid) and (memberOf!(nbrPtr, nbrArray)){
			return copy of nbrArray + nbrPointer+ getNetworkArray(nbrPointer, nbrArray);
		}
	}
}

int32* getNetworkSize(NbrList* nbrListPtr, count){
	int32 i, x = 0;
	int32* nbrArray;
	for (i = 0; i < nbrListPtr->size; i++){
		nbrPtr = nbrListPtr-> nbrs[i];
		if (nbrPtr-> orientationValid) and (memberOf!(nbrPtr, nbrArray)){
			return count + getNetworkSize(nbrPointer, nbrArray);
		}
	}
}

int32 memberOf(nbrPtr, int32* networkArray){
	//remove duplicates
	int32 i;
	for (i= 0; i < size of array, i++) {
		if nbrPtr = nbrListPtr->nbrs[i] {
			return True;
		}
	}
	return False;
}
*/


int32 behController(Beh* behPtr, int32 theta, int32 integralError, uint32 deltaTicks, uint32 networkSize) {
	int error, preIntegralGain, tv, rv, integral, proportional;

	integralError = integralError + (theta*deltaTicks);
	integral = integralError / 100000; //rv's get around 40 max with 100,000 WEIGHTED TEST WITH 100,000
	preIntegralGain = integral;
	if (integral > K_INTEGRAL_MAX) {
		integral = K_INTEGRAL_MAX;
	} else if (integral < -K_INTEGRAL_MAX) {
		integral = -K_INTEGRAL_MAX;
	}

	proportional = (theta * 6) / 10; //without integral term, oscillates just a bit with 13/20, WEIGHTED TEST WORKS WITH 6/10

	rv = (integral + proportional) * 2; //WEIGHTED TEST WORKS WITH 2
	//rv = ((100 - (OBJECT_MASS*100)/778) * rv) / 100; //formula works when object mass is 778 and 4 robots
	rv = ((100 - (networkSize*100)/4) * rv) / 100;

	tv = ((95 - (integral*100)/K_INTEGRAL_MAX) * K_TV)/100;
	//tv = ((100 - (OBJECT_MASS*100)/778) * tv) / 100;
	tv = ((100 - (networkSize*100)/4) * tv) / 100; //original code designed for 4 robots

	cprintf("theta: %d, int: %d, prop: %d, rv: %d, tv: %d \n", theta, integral, proportional, rv, tv);

	behPtr->tv = tv;
	behPtr->rv = rv;
	return integralError;
}


int32 behFrictionFlock(Beh* behPtr, NbrList* nbrListPtr, int32 tv, int32 integralError, uint32 currentTime, uint32 prevTime) {
	int32 i, x, y, avgOrientation;
	uint32 networkSize;
	Nbr* nbrPtr;

	x = 0;
	y = 0;
	for (i = 0; i < nbrListPtr->size; ++i) {
		nbrPtr = nbrListPtr->nbrs[i];
		if (nbrPtr->orientationValid) {
			int32 theta = normalizeAngleMilliRad((int32)nbrPtr->bearing + (int32)MILLIRAD_PI - (int32)nbrPtr->orientation);
			x += cosMilliRad(theta);
			y += sinMilliRad(theta);
		}
	}
	if (nbrListPtr->size > 0) {
		int32 theta = atan2MilliRad(y, x);
		//rvBearingController(behPtr, theta, 40);
		theta = -1*smallestAngleDifference(0,theta);
		uint32 deltaTicks = encoder_delta_ticks(currentTime, prevTime);
		//networkSize = getNetworkSize(nbrListPtr);
		integralError = behController(behPtr, theta, integralError, deltaTicks, 4);
	} else {
		behPtr->tv = 0;
		behPtr->rv = 0;
		behPtr->active = TRUE;
	}


	return integralError;
}

void behaviorTask(void* parameters) {
	//initialization
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound;
	uint8 neighbors;
	uint32 currentTime = osTaskGetTickCount();
	uint32 prevTime = 0;

	int32 integralError = 0;

	Beh behMove;
	NbrList *nbrList = malloc(sizeof(NbrList));

	//begin by moving forward
	behMove.tv = K_TV;
	behMove.rv = 0;
	behMove.active = TRUE;
	motorSetBeh(&behMove);


	for (;;) {
		neighborsGetMutex();
		nbrListCreate(nbrList);

		prevTime = currentTime;
		currentTime = osTaskGetTickCount();

		if (isBumped) {
			ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			integralError = behFrictionFlock(&behMove, nbrList, K_TV, integralError, currentTime, prevTime);
		}
		else {
			if (bumpSensorsGetBits() != 0) {
				if (nbrList-> size == 1) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				}
				else if (nbrList-> size == 2){
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				}
				else if (nbrList-> size == 3){
					ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				}

				isBumped = TRUE;
				integralError = behFrictionFlock(&behMove, nbrList, K_TV, integralError, currentTime, prevTime);
			}
			else {
				ledsSetPattern(LED_RED, LED_PATTERN_CLAW, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				behMove.tv = K_TV;
				behMove.rv = K_RV;
				behMove.active = TRUE;}
		}

		//if (buttons_get(BUTTON_RED)) {
//
	//	}

		motorSetBeh(&behMove);
		neighborsPutMutex();
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}



