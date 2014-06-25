/**
 * @file SamProject.c
 * @brief Code for TVRV transmission testing
 * @since June 6, 2014
 * @author ZKK
 */

#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

#define STATE_IDLE		0
#define STATE_RECIEVE 	1
#define STATE_SEND		2

#define PI 				3141
#define PI2 			1571

#define TV_SPEED		70
#define WALL_FOLLOW_SIDE 0

RadioMessage radioMessageTX;
RadioMessage radioMessageRX;
RadioCmd radioCmdRemoteControl;
char* name = "PFILTER";

void
behaviorTask(void* parameters)
{
	int x, y;
	int i, dead;
	int state = STATE_IDLE;
	NbrList nbrList;
	uint32 c = 0;
	uint32 neighborsCheck;
	uint8 buttonRed, buttonGreen, buttonBlue;
	int16 bearing, orientation, newRV;
	int32 tv, rv;
	int32 tvR, rvR = 0;
	boolean foundObj = 0;
	Beh behU = behInactive;
	uint32 lastWakeTime = osTaskGetTickCount();
	char* RXmsg;

	radioCommandSetSubnet(2);

	/* Set LEDs to idle state pattern */
	ledsSetPattern(LED_GREEN,
			LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);

	/* Neighbor data sets for TV and RV */
	NbrData moving;

	/* Initialize nbrs and data */
    neighborsInit(200);
    nbrDataCreate(&moving, "moving", 8, 0);

	for (;;) {
		/* Receive user input and set state */
		buttonRed = buttonsGet(BUTTON_RED);
		buttonGreen = buttonsGet(BUTTON_GREEN);
		buttonBlue = buttonsGet(BUTTON_BLUE);

		if (buttonRed) {
			state = STATE_RECIEVE;
			ledsSetPattern(LED_RED,
					LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		} else if (buttonBlue) {
			dead = 0;
			state = STATE_SEND;
			ledsSetPattern(LED_BLUE,
					LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		} else if (buttonGreen) {
			state = STATE_IDLE;
			ledsSetPattern(LED_GREEN,
					LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		}

		neighborsGetMutex();
		switch (state) {
		/* Receiving data from traveling robot */
		case STATE_RECIEVE: {
		    nbrDataSet(&moving, STATE_RECIEVE);
			/* Only print on a new nbr round */
			if (neighborsNewRoundCheck(&neighborsCheck)) {
				nbrListCreate(&nbrList);
				if(radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX, 0))
					RXmsg = radioCommandGetDataPtr(&radioMessageRX);

				for (i = 0; i < nbrList.size; i++) {
					if (nbrDataGetNbr(&moving, nbrList.nbrs[i]) == STATE_SEND) {
						/* cprintf("%d,%d,%s",
							nbrList.nbrs[i]->orientation,
							nbrList.nbrs[i]->bearing,
							RXmsg); */
						rprintf("%d,%d,%s",
								nbrList.nbrs[i]->orientation,
								nbrList.nbrs[i]->bearing,
								RXmsg);
						c += 1;
					}
				}
			}
			break;
		}
		/* Drive in a U shaped path around neighbor */
		case STATE_SEND: {
		    nbrDataSet(&moving, STATE_SEND);
			nbrListCreate(&nbrList);

			/*if (nbrList.size > 0) {
				bearing = nbrList.nbrs[0]->bearing;

				// If we are still driving towards the nbr
				if (abs(bearing) < PI2) {
					behSetTvRv(&behU, TV_SPEED, 0);
				 //Orbiting the nbr
				} else {
					bearing -= PI2;

					if (bearing < 0) {
						newRV = (bearing - PI2) / 1.5;
						behSetTvRv(&behU, TV_SPEED, newRV);
					} else {
						newRV = (bearing + PI2) / 1.5;
						behSetTvRv(&behU, TV_SPEED, newRV);
					}

					if (abs(newRV) < 200)
						behSetTvRv(&behU, TV_SPEED, 0);
				}

				// If we have completed a half circle, just go forward
				orientation = nbrList.nbrs[0]->orientation;
				if (orientation > 2900)
					behSetTvRv(&behU, TV_SPEED, 0);
			// Go to idle if we lose the nbr
			} else {
				dead++;

				if (dead > 5) {
					state = STATE_IDLE;
					ledsSetPattern(LED_GREEN,
							LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					break;
				}
			}*/
			if(irObstaclesGetBearing() || foundObj){
				foundObj = 1;
				behWallMove(&behU,TV_SPEED, WALL_FOLLOW_SIDE);
			} else{
				behSetTvRv(&behU,TV_SPEED,0);
			}

			/* Transmit tv and rv */
			motorGetTVRV(&tv, &rv);

			if (neighborsNewRoundCheck(&neighborsCheck)) {
				//cprintf("%d,%d\n", tv, rv);
				rprintf("%d,%d\n", x++, y++);
				x = x % 198;
				y = y % 173;
			}

			//sprintf(radioMessageTX.command.data, "%ld,%ld\n", tv, rv);
			//radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);

			break;
		}
		/* Idle until active */
		case STATE_IDLE:
		default: {
			foundObj == 0;
		    nbrDataSet(&moving, STATE_IDLE);
			behU = behInactive;
			break;
		}
		}
		neighborsPutMutex();
		motorSetBeh(&behU);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

int
main(void)
{
	systemInit();
	behaviorSystemInit(behaviorTask, 4096);
	osTaskStartScheduler();
	return 0;
}

