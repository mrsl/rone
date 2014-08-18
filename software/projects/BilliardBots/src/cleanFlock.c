#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

#define PI 	3141

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

// States
#define STATE_IDLE 		0
#define STATE_BOUNCE 	1
#define STATE_REMOTE	2

#define MOVE_IDLE 		0
#define MOVE_FORWARD 	1
#define MOVE_STOP 		2
#define FLOCKWAIT		3000
#define FLOCKSPEED 		25
#define PROBABILITYSTOP 30
#define NEIGHBOR_ROUND_TIME 300

#define PROBSTOP_MAX	10000

#define CHECK 0xCAFE // unique code to check for correct message
#define CHECK2 0xBABE

uint32 probStop = 30;

struct __attribute__((__packed__)) {
	uint32 check;
	uint8 val;
	char pad[25];
} typedef remoteControlMsg;

struct __attribute__((__packed__)) {
	uint32 check;
	uint32 val;
	char pad[22];
} typedef probControlMsg;


SerialCmd serialCmdPS;

RadioCmd radioCmdPS;
RadioMessage radioCmdPSMessage;


void serialCmdPSFunc(char* command) {
	int i;
	probControlMsg *newMessage = (probControlMsg *) radioCommandGetDataPtr(&radioCmdPSMessage);

	command += 2;
	newMessage->check = CHECK2;

	if (sscanf(command, "%u", (unsigned int *)&newMessage->val) != 1) {
		cprintf("Invalid Probability! Range is [0, 10000].");
		return;
	}

	probStop = newMessage->val;
	cprintf("Probability set to %u out of %u!\n", probStop, PROBSTOP_MAX);

	for (i = 0; i < 5; i++) {
		radioCommandXmit(&radioCmdPS, ROBOT_ID_ALL, &radioCmdPSMessage);
	}
}

void behaviorTask(void* parameters) {

	int state = STATE_IDLE;
	int movementState;
	uint32 lastWakeTime = osTaskGetTickCount();
	NbrList nbrList;
	Beh behOutput;
	uint8 buttonRed, buttonGreen, buttonBlue;
	boolean printNow;
	uint32 neighborRound = 0;
	int i;
	uint32 nbrRawRange, nbrRawBearing, nbrRawOrientation;
	Nbr* nbrPtr;
	RadioMessage radioMessageRX;
	RadioMessage radioMessageTX;
	RadioCmd radioCmdRemoteControl;
	char* name = "remoteControl";
	radioCommandAddQueue(&radioCmdRemoteControl, name, 1);
	uint32 waitTime = 0;
	Pose pose;
	uint8 checkstop = 0;

	serialCommandAdd(&serialCmdPS, "ps", serialCmdPSFunc);
	radioCommandAddQueue(&radioCmdPS, "probSetter", 1);

	// Init nbr system
	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);
	radioCommandSetSubnet(1);
    neighborsInit(NEIGHBOR_ROUND_TIME);

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
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			osTaskDelay(1000);
			continue;
		}

		if (radioCommandReceiveNonBlocking(&radioCmdPS, &radioCmdPSMessage)) {
			probControlMsg *newMessage = (probControlMsg *) radioCommandGetDataPtr(&radioCmdPSMessage);

			if (newMessage->check == CHECK2) {
				ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_TURBO);
				osTaskDelay(50);

				probStop = newMessage->val;
				cprintf("Probability set to %u out of %u!\n", probStop, PROBSTOP_MAX);
			}
		}

		behOutput = behInactive;
		printNow = neighborsNewRoundCheck(&neighborRound);
		nbrListCreate(&nbrList);
		broadcastMsgUpdate(&broadcastMessage, &nbrList);

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
				if (movementState == MOVE_STOP){
						movementState = MOVE_STOP;
				}else
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
				break;
			}
			case MOVE_FORWARD: {
				if (rand() % PROBSTOP_MAX <= probStop) {
					movementState = MOVE_STOP;
					checkstop = 0;
				} else {
					behFlock(&behOutput, &nbrList, FLOCKSPEED);
				}
				break;
			}
			case MOVE_STOP: {
				checkstop = 1;
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				behSetTvRv(&behOutput, 0, 0);
				if(printNow)
					waitTime += NEIGHBOR_ROUND_TIME;
				if (waitTime > FLOCKWAIT) {
					movementState = MOVE_FORWARD;
					waitTime = 0;
				}
				break;
			}
			default: {
				break;
			}
		}

		// Set nbr data
		nbrDataSet16(&TV_H,&TV_L,(int16)behOutput.tv);
		nbrDataSet16(&RV_H,&RV_L,(int16)behOutput.rv);

		// print neighbor list

		// encoderPoseUpdate();

		if (printNow) {

			encoderGetPose(&pose);

			rprintf("%d,%d,%d,%d,", checkstop, pose.x, pose.y, pose.theta);

			for (i = 0; i < nbrList.size; i++) {

				nbrPtr = nbrList.nbrs[i];
				nbrRawRange = nbrGetRawRange(nbrPtr);
				nbrRawOrientation = nbrGetRawOrientation(nbrPtr);
				nbrRawBearing = nbrGetRawBearing(nbrPtr);

				rprintf("%d,%d,%d,%d,%d,%d",
												  nbrPtr->ID,
										  (int16) nbrRawBearing,
										  (int16) nbrRawRange,
										  (int16) nbrRawOrientation,
										  (int16) nbrDataGetNbr16(&TV_H, &TV_L, nbrPtr),
										  (int16) nbrDataGetNbr16(&RV_H, &RV_L, nbrPtr));
				if (i < nbrList.size-1)
					rprintf(",");
			}
			rprintf("\n");
			rprintfFlush();
		}

		// set behavior and delay task
		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		lastWakeTime = osTaskGetTickCount();
	}
}


int main() {

	ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
	systemInit();
	systemPrintStartup();

	behaviorSystemInit(behaviorTask, 4096);

	osTaskStartScheduler();
	return 0;
}
