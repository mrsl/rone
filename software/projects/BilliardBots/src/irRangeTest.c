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
#define STATE_BOUNCE 		3

#define MOVE_IDLE 		0
#define MOVE_ROTATE 	1
#define MOVE_FORWARD 	2


void behaviorTask(void* parameters) {
	int state = STATE_IDLE;
	int movementState;
	int16 bumpBearing,rotateRV;
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 rotateTime, momentumTime;
	NbrList nbrList;
	Beh behOutput;
	uint8 buttonRed, buttonGreen, buttonBlue;
	boolean momementumActive, printNow;
	uint32 neighborRound = 0;
	int i;
	int16 cnt = 0;
	uint32 nbrBearing,nbrRange,nbrOrientation;
	Nbr* nbrPtr;
	uint32 tempWakeTime = 0;
	Pose pose;


	// Init nbr system
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);
	radioCommandSetSubnet(1);
    neighborsInit(300);

    // Create nbr data

    // Print startup message and thread memory usage
	systemPrintStartup();
	systemPrintMemUsage();

	// Initalize Encoder
	encoderInit();

	NbrData TV_H;
	NbrData TV_L;
	NbrData RV_H;
	NbrData RV_L;
	nbrDataCreate16(&TV_H,&TV_L,"TV_H","TV_L",0);
	nbrDataCreate16(&RV_H,&RV_L,"RV_H","RV_L",0);

	for (;;) {
		behOutput = behInactive;
		printNow = neighborsNewRoundCheck(&neighborRound);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);
		// Determine state from buttons
		//if (state == STATE_IDLE) {
			if (buttonsGet(BUTTON_RED)) {
			} else if (buttonsGet (BUTTON_GREEN)) {
				state = STATE_IDLE;
			} else if (buttonsGet (BUTTON_BLUE)) {
				state = STATE_BOUNCE;
				movementState = MOVE_FORWARD;
			}
		//}

		// Set LEDs for each state
		switch (state) {
		case STATE_BOUNCE: {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			uint8* bitMatrix = irObstaclesGetBitMatrix();
			/*for(i = 0; i < 8; i++){
				rprintf("%d", bitMatrix[i]);

				if (i != 7)
					rprintf(",");
			}
			rprintf("\n");
			*/
			break;
		}
		case STATE_IDLE:
		default: {
			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			movementState = MOVE_IDLE;
			break;
		}
		}

		//Movement State Machine
		switch (movementState) {
			case MOVE_IDLE: {
				momementumActive = 0;
				behSetTvRv(&behOutput, 0, 0);
				break;
			}
			case MOVE_ROTATE: {
				momementumActive = 0;
				behSetTvRv(&behOutput, 0, rotateRV);
				if((rotateTime + bumpBearing) < osTaskGetTickCount()){
					movementState = MOVE_FORWARD;
					momentumTime = osTaskGetTickCount();
				}
				break;
			}
			case MOVE_FORWARD: {
				momementumActive = 1;
				if(cnt%2==0)
				behSetTvRv(&behOutput, 150, 0);
				if(cnt%4==1)
				behSetTvRv(&behOutput,100,-500);
				if(cnt%4==3)
				behSetTvRv(&behOutput,100,500);
				if (bumpSensorsGetBearing() != -1) {
					cnt++;
					movementState = MOVE_ROTATE;
					rotateTime = osTaskGetTickCount();
					bumpBearing = bumpSensorsGetBearing();

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
		}

		nbrDataSet16(&TV_H,&TV_L,(int16)behOutput.tv);
		nbrDataSet16(&RV_H,&RV_L,(int16)behOutput.rv);
		motorSetBeh(&behOutput);

		encoderPoseUpdate();
		encoderGetPose(&pose);

			if (!printNow)
				rprintf("%d, %d, %d\n",pose.x,pose.y,pose.theta);


			for (i = 0; i < nbrList.size; i++){
				nbrPtr = nbrList.nbrs[i];
				nbrBearing = nbrGetBearing(nbrPtr);
				nbrRange = nbrGetRange(nbrPtr);
				nbrOrientation = nbrGetOrientation(nbrPtr);
					if (printNow){
						rprintf("%d, %d, %d, %d, %d, %d, %d, %d\n", pose.x, pose.y, pose.theta, nbrBearing, nbrOrientation, nbrRange, (int16)nbrDataGetNbr16(&TV_H,&TV_L,nbrPtr),(int16)nbrDataGetNbr16(&RV_H,&RV_L,nbrPtr));
					}
			}

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

