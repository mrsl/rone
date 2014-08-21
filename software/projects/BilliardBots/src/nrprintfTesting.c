#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

#define PI 	3141

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50

// States
#define STATE_IDLE 			0
#define STATE_FLOCKFORWARD 	1
#define STATE_STOPPED		2
#define STATE_REMOTE		3

#define MOVE_IDLE 			0
#define MOVE_FORWARD 		1
#define MOVE_STOP 			2

#define FLOCKWAIT			3000
#define FLOCKSPEED 			25
#define PROBABILITYSTOP 	30
#define NEIGHBOR_ROUND_TIME 300

#define PROBSTOP_MAX		10000

#define CHECK 				0xCAFE

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
	newMessage->check = CHECK;

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
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 recentMoveTime;
	uint8 id;
	NbrList nbrList;
	Beh behOutput;
	uint8 buttonRed,
		  buttonGreen,
		  buttonBlue;
	boolean printNow;
	uint32 neighborRound = 0;
	int i;
	int16 nbrRawRange;
	int32 nbrRawBearing, nbrRawOrientation;
	Nbr *nbrPtr;
	RadioMessage radioMessageRX;
	RadioMessage radioMessageTX;
	RadioCmd radioCmdRemoteControl;
	radioCommandAddQueue(&radioCmdRemoteControl, "remoteControl", 1);
	Pose mostRecentPose;
	uint32 lastMoveTime;

	serialCommandAdd(&serialCmdPS, "ps", serialCmdPSFunc);
	radioCommandAddQueue(&radioCmdPS, "probSetter", 1);

	// Init nbr system
	radioCommandSetSubnet(1);
    neighborsInit(NEIGHBOR_ROUND_TIME);

    // Print startup message and thread memory usage
	systemPrintStartup();
	systemPrintMemUsage();

	// Create neighbor data
	NbrData TV_H;
	NbrData TV_L;
	NbrData RV_H;
	NbrData RV_L;
	nbrDataCreate16(&TV_H, &TV_L, "TV_H", "TV_L", 0);
	nbrDataCreate16(&RV_H, &RV_L, "RV_H", "RV_L", 0);

	for (;;) {
		if (rprintfIsHost()) {
			neighborsDisable();
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			osTaskDelay(1000);
			continue;
		}

		// Check to see if any probability setting messages have arrived
		if (radioCommandReceiveNonBlocking(&radioCmdPS, &radioCmdPSMessage)) {
			probControlMsg *newMessage = (probControlMsg *) radioCommandGetDataPtr(&radioCmdPSMessage);

			if (newMessage->check == CHECK) {
				ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_TURBO);
				osTaskDelay(50);

				probStop = newMessage->val;
				cprintf("Probability set to %u out of %u!\n", probStop, PROBSTOP_MAX);
			}
		}

		behOutput = behInactive;
		printNow = neighborsNewRoundCheck(&neighborRound);
		nbrListCreate(&nbrList);

		// Read buttons
		buttonRed = buttonsGet(BUTTON_RED);
		buttonGreen = buttonsGet(BUTTON_GREEN);
		buttonBlue = buttonsGet(BUTTON_BLUE);

		// Set state with buttons
		if (buttonRed) {
			if (state == STATE_IDLE) {
				state = STATE_REMOTE;
			}
		} else if (buttonGreen) {
			state = STATE_IDLE;
		} else if (buttonBlue) {
			if (state == STATE_IDLE) {
				state = STATE_FLOCKFORWARD;
			}
		}

		// Parse remote messages
		if (state != STATE_REMOTE) {
			if (radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX, 0)) {
				remoteControlMsg *rcMsg = (remoteControlMsg *)radioCommandGetDataPtr(&radioMessageRX);
				if (rcMsg->check == CHECK) {
					if (rcMsg->val == 1) {
						state = STATE_FLOCKFORWARD;
					} else {
						state = STATE_IDLE;
					}
				}
			}
		// Send remote messages
		} else {
			remoteControlMsg *rcMsg = (remoteControlMsg *)radioCommandGetDataPtr(&radioMessageTX);
			rcMsg->check = CHECK;
			if (buttonRed) {
				rcMsg->val = 1;
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
			} else if (buttonBlue) {
				rcMsg->val = 0;
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
			}
		}

		// Stop at random intervals
		if (state == STATE_FLOCKFORWARD) {
			if (rand() % PROBSTOP_MAX <= probStop) {
				state = STATE_STOPPED;
				lastMoveTime = osTaskGetTickCount();
			}
		} else if (state == STATE_STOPPED) {
			if (osTaskGetTickCount() > lastMoveTime + FLOCKWAIT) {
				state = STATE_FLOCKFORWARD;
			}
		}

		// Set LEDs for each state
		switch (state) {
		case STATE_REMOTE: {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			break;
		}
		case STATE_FLOCKFORWARD: {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			break;
		}
		case STATE_STOPPED: {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			break;
		}
		case STATE_IDLE:
		default: {
			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			break;
		}
		}

		// Set movement for each state
		switch (state) {
		case STATE_FLOCKFORWARD: {
			behFlock(&behOutput, &nbrList, FLOCKSPEED);
			break;
		}
		case STATE_STOPPED:
		case STATE_REMOTE:
		case STATE_IDLE:
		default: {
			break;
		}
		}

		if (printNow) {
			int16 tv1, rv1;

			encoderGetPose(&mostRecentPose);
			rprintf("%d,%d,%d,%d", state == STATE_STOPPED,
								   	  mostRecentPose.x,
								   	  mostRecentPose.y,
								   	  mostRecentPose.theta);
			cprintf("%d,%d,%d,%d", state == STATE_STOPPED,
								   	  mostRecentPose.x,
								   	  mostRecentPose.y,
								   	  mostRecentPose.theta);


			for (i = 0; i < nbrList.size; i++) {
				if ((nbrPtr = nbrList.nbrs[i])) {
					id = nbrGetID(nbrPtr);
					if (id != ROBOT_ID_NULL) {
						nbrRawRange = nbrGetRange(nbrPtr);
						nbrRawOrientation = nbrGetOrientation(nbrPtr);
						nbrRawBearing = nbrGetBearing(nbrPtr);
						tv1 = (int16) nbrDataGetNbr16(&TV_H, &TV_L, nbrPtr);
						rv1 = (int16) nbrDataGetNbr16(&RV_H, &RV_L, nbrPtr);

						cprintf(",%d,%d,%d,%d,%d,%d", id,
													  nbrRawBearing,
													  nbrRawRange,
													  nbrRawOrientation,
													  tv1,
													  rv1);
//

		//				nbrRawRange = nbrGetRawRange(nbrPtr);
//		//				nbrRawOrientation = nbrGetRawOrientation(nbrPtr);
//		//				nbrRawBearing = nbrGetRawBearing(nbrPtr);

//

////
						rprintf(",%d,%d,%d,%d,%d,%d", id,
													  nbrRawBearing,
													  nbrRawRange,
													  nbrRawOrientation,
													  tv1,
													  rv1);

//						nbrRawBearing = 1000;
//						nbrRawRange = 1000;
//						nbrRawOrientation = 1000;
//						rprintf(",%d,%d,%d,%d", id,
//												nbrRawBearing,
//												nbrRawRange,
//												nbrRawOrientation);
					}
				}
			}
			rprintf("\n");
			cprintf("\n");
			rprintfFlush();
		}

		// Set nbr data
		nbrDataSet16(&TV_H, &TV_L, (int16) behOutput.tv);
		nbrDataSet16(&RV_H, &RV_L, (int16) behOutput.rv);

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
	return (0);
}
