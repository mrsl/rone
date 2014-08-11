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
#define STATE_BOUNCE 	3
#define STATE_REMOTE	4

#define MOVE_IDLE 		0
#define MOVE_ROTATE 	1
#define MOVE_FORWARD 	2
#define FLOCKWAIT		2000
#define FLOCKSPEED 		25
#define PROBABILITYSTOP 1

#define CHECK 0xCAFE // unique code to check for correct message

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 val;
	char pad[25];
} typedef remoteControlMsg;


char buffer[160];
char *bufp;

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
	int i, n;
	uint32 nbrBearing, nbrOrientation, nbrRange;
	Nbr* nbrPtr;
	uint32 tempWakeTime = 0;
	RadioMessage radioMessageRX;
	RadioMessage radioMessageTX;
	RadioCmd radioCmdRemoteControl;
	char* name = "remoteControl";
	int comBlue = 0;
	radioCommandAddQueue(&radioCmdRemoteControl, name, 1);
	uint32 tickSeconds = osTaskGetTickCount();

	rprintfSetSleepTime(10);
	rprintfEnableRobot(ROBOT_ID_ALL, FALSE);
	rprintfEnableRobot(95 , TRUE);

	// Init nbr system
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);
	radioCommandSetSubnet(1);
    neighborsInit(300);

    // Create nbr data

    // Print startup message and thread memory usage
	systemPrintStartup();
	systemPrintMemUsage();

	NbrData TV_H;
	NbrData TV_L;
	NbrData RV_H;
	NbrData RV_L;
	nbrDataCreate16(&TV_H,&TV_L,"TV_H","TV_L",0);
	nbrDataCreate16(&RV_H,&RV_L,"RV_H","RV_L",0);

	for (;;) {
//		if (rprintfIsHost) {
//			osTaskDelay(1000);
//			continue;
//		}
		behOutput = behInactive;
		printNow = neighborsNewRoundCheck(&neighborRound);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);
		radioCommandSetSubnet(1);

		// read buttons
		buttonRed = buttonsGet(BUTTON_RED);
		buttonGreen = buttonsGet(BUTTON_GREEN);
		buttonBlue = buttonsGet(BUTTON_BLUE);

		// set state machine
		if (buttonRed) {
			if (state == STATE_IDLE) {
				state = STATE_REMOTE;
			} else if (state == STATE_REMOTE) {
				remoteControlMsg *rcMsg = (remoteControlMsg *)radioCommandGetDataPtr(&radioMessageTX);
				rcMsg->check = CHECK;
				rcMsg->val = 1;
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
			}
		} else if (buttonGreen) {
			if (state != STATE_IDLE) {
				state = STATE_IDLE;
			}
		} else if (buttonBlue) {
			if (state == STATE_IDLE) {
				state = STATE_BOUNCE;
			} else if (state == STATE_REMOTE) {
				remoteControlMsg *rcMsg = (remoteControlMsg *)radioCommandGetDataPtr(&radioMessageTX);
				rcMsg->check = CHECK;
				rcMsg->val = 0;
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
			}
		}

		// parse remote messages
		if (state != STATE_REMOTE) {
			if (radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX, 0)) {
				remoteControlMsg *rcMsg = (remoteControlMsg *)radioCommandGetDataPtr(&radioMessageRX);
				if (rcMsg->check == CHECK) {
					if (rcMsg->val == 1) {
						state = STATE_BOUNCE;
					} else {
						state = STATE_IDLE;
					}
				}
			}
		}

		// Set LEDs for each state
		switch (state) {
		case STATE_REMOTE: {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			movementState = MOVE_IDLE;
			break;
		}
		case STATE_BOUNCE: {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			uint8* bitMatrix = irObstaclesGetBitMatrix();
			movementState = MOVE_FORWARD;
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
			if (osTaskGetTickCount() - tickSeconds > FLOCKWAIT) {
				movementState = MOVE_FORWARD;
				tickSeconds = osTaskGetTickCount();
			}
			break;
		}
		case MOVE_ROTATE: {
			momementumActive = 0;
			behSetTvRv(&behOutput, 0, rotateRV);
			if ((rotateTime + bumpBearing) < osTaskGetTickCount()) {
				movementState = MOVE_FORWARD;
				momentumTime = osTaskGetTickCount();
			}
			break;
		}
		case MOVE_FORWARD: {
			momementumActive = 1;
			behFlock(&behOutput, &nbrList, FLOCKSPEED);
			if (osTaskGetTickCount() - tickSeconds > 1000) {
				tickSeconds = osTaskGetTickCount();
				if (rand() % 1000 <= PROBABILITYSTOP)
					movementState = MOVE_IDLE;
				else
					behFlock(&behOutput, &nbrList, FLOCKSPEED);
			}
			break;
		}
		}

		// Set nbr data
		nbrDataSet16(&TV_H,&TV_L,(int16)behOutput.tv);
		nbrDataSet16(&RV_H,&RV_L,(int16)behOutput.rv);

		// print neighbor list


		if (printNow) {
			for (i = 0; i < nbrList.size; i++) {
				nbrPtr = nbrList.nbrs[i];
				nbrBearing = nbrGetBearing(nbrPtr);
				nbrOrientation = nbrGetOrientation(nbrPtr);
				nbrRange = nbrGetRange(nbrPtr);

				cprintf("%d,%d,%d,%d,%d,%d,", nbrPtr->ID,
												  (int16) nbrBearing,
												  (int16) nbrOrientation,
												  (int16) nbrRange,
												  (int16) nbrDataGetNbr16(&TV_H, &TV_L, nbrPtr),
												  (int16) nbrDataGetNbr16(&RV_H, &RV_L, nbrPtr));

			}

		}

//		if (printNow) {
//			bufp = buffer;
//			for (i = 0; i < nbrList.size; i++) {
//				nbrPtr = nbrList.nbrs[i];
//				nbrBearing = nbrGetBearing(nbrPtr);
//				nbrOrientation = nbrGetOrientation(nbrPtr);
//				nbrRange = nbrGetRange(nbrPtr);
//
//				n = sprintf(bufp, "%d,%d,%d,%d,%d,%d,", nbrPtr->ID,
//												  (int16) nbrBearing,
//												  (int16) nbrOrientation,
//												  (int16) nbrRange,
//												  (int16) nbrDataGetNbr16(&TV_H, &TV_L, nbrPtr),
//												  (int16) nbrDataGetNbr16(&RV_H, &RV_L, nbrPtr));
//				bufp += n;
//			}
//			n = sprintf(bufp - 1, "\n");
//			fprintf(buffer);
//		}

		// set behavior and delay task
		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		lastWakeTime = osTaskGetTickCount();
	}
}


int main() {
	systemInit();
	systemPrintStartup();

	behaviorSystemInit(behaviorTask, 4096);

	osTaskStartScheduler();
	return 0;
}

