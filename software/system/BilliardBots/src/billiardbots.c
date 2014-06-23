#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

#define PI 	3141

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

#define WAIT_MOMENTUM					200

// States
#define STATE_IDLE 		0
#define STATE_CUEBALL 	1
#define STATE_POCKET 	2
#define STATE_BALL 		3

#define MOVE_IDLE 		0
#define MOVE_ROTATE 	1
#define MOVE_FORWARD 	2


void behaviorTask(void* parameters) {
	int state = STATE_IDLE;
	int movementState;
	int16 bumpBearing,rotateRV, currTv;
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 rotateTime, momentumTime;
	NbrList nbrList;
	Beh behOutput;
	uint8 buttonRed, buttonGreen, buttonBlue;
	boolean momementumActive, printNow;
	uint32 neighborRound = 0;
	int i;
	uint32 nbrBearing;

	Nbr* nbrPtr;
	NbrData type;
	NbrData velocity;

	// Init nbr system
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);
	radioCommandSetSubnet(1);
    neighborsInit(300);

    // Create nbr data
	nbrDataCreate(&type, "type", 2, 0);
	nbrDataCreate(&velocity, "velocity", 8, 0);

    // Print startup message and thread memory usage
	systemPrintStartup();
	systemPrintMemUsage();


	for (;;) {
		behOutput = behInactive;
		printNow = neighborsNewRoundCheck(&neighborRound);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);
		// Determine state from buttons
		if (state == STATE_IDLE) {
			if (buttonsGet(BUTTON_RED)) {
				state = STATE_CUEBALL;
			} else if (buttonsGet (BUTTON_GREEN)) {
				state = STATE_POCKET;
			} else if (buttonsGet (BUTTON_BLUE)) {
				state = STATE_BALL;
			}
		}

		// Set LEDs for each state
		switch (state) {
		case STATE_CUEBALL: {
			ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			nbrDataSet(&type, STATE_CUEBALL);
			break;
		}
		case STATE_POCKET: {
			ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			nbrDataSet(&type, STATE_POCKET);
			nbrDataSet(&velocity, 0);
			movementState = MOVE_IDLE;
			break;
		}
		case STATE_BALL: {
			nbrDataSet(&type, STATE_CUEBALL);
			if (roneID % 2)
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			else
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			break;
		}
		case STATE_IDLE:
		default: {
			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			movementState = MOVE_IDLE;
			break;
		}
		}

		//Movement State Machine
		switch (movementState) {
			case MOVE_IDLE: {
				momementumActive = 0;
				behSetTvRv(&behOutput, 0, 0);
				nbrDataSet(&velocity, 0);
				if (bumpSensorsGetBearing() != -1) {
					movementState = MOVE_ROTATE;
					rotateTime = osTaskGetTickCount();
					bumpBearing = bumpSensorsGetBearing();

					if (state == STATE_CUEBALL){
						currTv = 150;
					} else if(state == STATE_BALL){
						//currTv = 75;
						for (i = 0; i < nbrList.size; i++){
							nbrPtr = nbrList.nbrs[i];
							nbrBearing = nbrGetBearing(nbrPtr);
							cprintf("NBR ID: %d, BB %d, NB %d\n",nbrGetID(nbrPtr),bumpBearing,nbrBearing);
							if(abs(nbrBearing -bumpBearing) < 500){
								currTv = nbrDataGetNbr(&velocity, nbrPtr);
								cprintf("currTv %d")
							}
						}
					}else{
						currTv = 0;
					}

					if(bumpBearing > 0){
						bumpBearing = PI - bumpBearing;
						rotateRV = -1000;
					} else{
						bumpBearing = PI - abs(bumpBearing);
						rotateRV = 1000;
					}
					//bearing = bearing * 1.25;
					behSetTvRv(&behOutput, 0, rotateRV);
				}
				break;
			}
			case MOVE_ROTATE: {
				momementumActive = 0;
				behSetTvRv(&behOutput, 0, rotateRV);
				nbrDataSet(&velocity, 0);
				if((rotateTime + bumpBearing) < osTaskGetTickCount()){
					movementState = MOVE_FORWARD;
					momentumTime = osTaskGetTickCount();
				}
				break;
			}
			case MOVE_FORWARD: {
				momementumActive = 1;
				behSetTvRv(&behOutput, currTv, 0);
				nbrDataSet(&velocity, abs(behOutput.tv));
				if (bumpSensorsGetBearing() != -1) {
					momementumActive = 0;
					movementState = MOVE_ROTATE;
					rotateTime = osTaskGetTickCount();
					bumpBearing = -bumpSensorsGetBearing();
					if(state == STATE_BALL){
						for (i = 0; i < nbrList.size; i++){
							nbrPtr = nbrList.nbrs[i];
							nbrBearing = nbrGetBearing(nbrPtr);
							if(abs(nbrBearing -bumpBearing) < 500){
								if(nbrDataGetNbr(&type, nbrPtr) == STATE_POCKET){
									state = STATE_IDLE;
								}
							}
					}
					if(bumpBearing > 0){
						rotateRV = 1000;
					} else{
						rotateRV = -1000;
					}
					bumpBearing = bumpBearing * 2;
					behSetTvRv(&behOutput, 0, rotateRV);

				}
				if(currTv == 0){
					movementState = MOVE_IDLE;
				}
				break;
			}
		}

		if(((currTv > 0) && (momentumTime + WAIT_MOMENTUM) < osTaskGetTickCount()) && (momementumActive)){
			currTv -=5;
			if(currTv < 0){
				currTv = 0;
			}
			momentumTime = osTaskGetTickCount();
		}

		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		lastWakeTime = osTaskGetTickCount();
	}
}


int main(void) {
	systemInit();
	systemPrintStartup();

	behaviorSystemInit(behaviorTask, 4096);

	osTaskStartScheduler();
	return 0;
}

