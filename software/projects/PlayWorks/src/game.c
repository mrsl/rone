/*
 * game.c
 *
 *  Created on: Mar 31, 2014
 *      Author: jamesm
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"


#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50

#define WAYPOINTS_MAX				20
#define COMMAND_DONE				0
#define COMMAND_FORWARD				1
#define COMMAND_LEFT				2
#define COMMAND_RIGHT				3
#define COMMAND_EXECUTE_TIME		2000
#define BUTTON_PRESS_TIME			300

#define PRINT_TIME					200
#define PRINT_TIME_MEM				5000

#define GAME_MODE_IDLE				0
#define GAME_MODE_MATH_PROGRAM		1
#define GAME_MODE_MATH_TILE			2
#define GAME_MODE_STORY_PROGRAM		3
#define GAME_MODE_STORY_TILE		4
#define GAME_MODE_NAVIGATION		5

#define MAX_NUM_INPUT				10


/******** user code ********/


#if 1
#define BLUETOOTH_COMMAND_ARG_LEN		20
#define BLUETOOTH_COMMAND_IDX			3
#define BLUETOOTH_COMMAND_MOVE_ARG		8

uint8 gameMode = GAME_MODE_IDLE;

static void serialCmdTPFunc(char* command) {
	uint8 i;

	// move past the command text, skip the 'TP,'
	//command += 3;

	//for now, echo the command back.
	uint8 len = strlen(command);
	for (i = 0; i < len; ++i) {
		if (command[i] == ',') {
			command[i] = 0;
		}
	}
	if (strncmp(&command[BLUETOOTH_COMMAND_IDX], "game", BLUETOOTH_COMMAND_ARG_LEN) == 0) {
		// run a tile track game.  The robot is autonomous
		gameMode = GAME_MODE_MATH_TILE;
	} else if (strncmp(&command[BLUETOOTH_COMMAND_IDX], "move", BLUETOOTH_COMMAND_ARG_LEN) == 0) {
		// move under direct program control from the phone
		if (gameMode != GAME_MODE_MATH_PROGRAM) {
			gameMode = GAME_MODE_MATH_PROGRAM;
			gameFSMInit();
			gameFSM(tileMotionGetTile());
		}
		char* gameCmd = &command[BLUETOOTH_COMMAND_MOVE_ARG];
		cprintf("\ngame cmd %s\n", gameCmd);
		boolean validCmd = FALSE;
		if (strncmp(gameCmd, "forward", BLUETOOTH_COMMAND_ARG_LEN) == 0) {
			tileMotion(TILEMOTION_FORWARD);
			validCmd = TRUE;
		} else if (strncmp(gameCmd, "left", BLUETOOTH_COMMAND_ARG_LEN) == 0) {
			tileMotion(TILEMOTION_ROTATE_LEFT);
			validCmd = TRUE;
		} else if (strncmp(gameCmd, "right", BLUETOOTH_COMMAND_ARG_LEN) == 0) {
			tileMotion(TILEMOTION_ROTATE_RIGHT);
			validCmd = TRUE;
		}
		if(validCmd) {
			tileMotionWaitUntilDone();
			gameFSM(tileMotionGetTile());
			cprintf("\nTP,move,done\n");
		}
	}

}


//static void serialCmdStartFunc(char* command) {
//
//}

#else
/* echo the programmign commands back to the phone */
static void serialCmdTPFunc(char* command) {
	uint8 i;

	// move past the command text, skip the 'TP,'
	//command += 3;

	//for now, echo the command back.
	for (i = 0; i < strlen(command); ++i) {
		if (command[i] == ',') {
			command[i] = '-';
		}
	}
	cprintf("\nTP,debug,%s\n", command);
	osTaskDelay(200);
	cprintf("\nTP,move,done\n");
}
#endif



// behaviors run every 50ms (?).  They should be designed to be short, and terminate quickly.
// they are used for robot control.
void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 gameMode = GAME_MODE_NAVIGATION;
	static uint32 printTime = 0;
	static uint32 printTimeMem = 0;
	uint8 gameStatus = GAME_STATUS_CONTINUE;
	static SerialCmd serialCmdTP, serialCmdStart;

	uint32 i;
	uint8 inputList[MAX_NUM_INPUT]; // Store a list of turn/forward inputs
	uint8 inputListPtr = 0, inputListExecPtr = 0;
	uint8 numInput = 0;

	// Led start screen
	ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);

	// init the neighbor system.  use a 300ms update
	neighborsInit(300);

	// init the magnetometers
	// TODO remove
    magInit();

    // Init the reflective sensors
    //TODO small delay

    // Create tileMotion task
    tileMotionInit();

    // Create sound task
    playworksSoundInit();

    // setup cprintf to send only a newline
    //cprintfCRLFMode(CPRINTF_CRLF_LF);

    // Add a serial command to receive remote control messages from the joysticks
	serialCommandAdd(&serialCmdTP, "TP", serialCmdTPFunc);

	// Start command after the user finishes inputing commands
//	serialCommandAdd(&serialCmdStart, "start", serialCmdStartFunc);


	// Init inputList
	for (i = 0; i < MAX_NUM_INPUT; i++) {
		inputList[i] = TILEMOTION_IDLE;
	}

	while (TRUE) {
		neighborsGetMutex();

		boolean printNow = printTimer(&printTime, PRINT_TIME);
		boolean printNowMem = printTimer(&printTimeMem, PRINT_TIME_MEM);

		if (printNowMem) {
			//systemPrintMemUsage();
		}

		switch (gameMode) {
		case GAME_MODE_IDLE:
			// Select game mode: RED = NAVI.
			if (buttonsGetEdge(BUTTON_RED)) {
				cprintf("Game mode = Navigation\n");
				gameMode = GAME_MODE_NAVIGATION;
			}
			break;
		case GAME_MODE_NAVIGATION:
			// Robot execute commands when the input buffer is full
			// Check if the inputList is full
			if (numInput == MAX_NUM_INPUT) {
				// Send new command to tileMotion task when it is ready
				if (tileMotionDone() && (inputListExecPtr < MAX_NUM_INPUT)) {
					tileMotion(inputList[inputListExecPtr]);
					inputListExecPtr++;
				}
				// Finished executing all inputs. Revert back to normal state
				if (inputListExecPtr == MAX_NUM_INPUT) {
					inputListExecPtr = 0;
					numInput = 0;
				}
			} else {
				// Enter the sequence of motion commands
				if (buttonsGetEdge(BUTTON_RED)) {
					// Left
					cprintf("%d = left\n", numInput);
					inputList[inputListPtr] = TILEMOTION_ROTATE_LEFT;
					inputListPtr = (inputListPtr + 1) % MAX_NUM_INPUT;
					numInput++;
				} else if (buttonsGetEdge(BUTTON_BLUE)) {
					// Right
					cprintf("%d = right\n", numInput);
					inputList[inputListPtr] = TILEMOTION_ROTATE_RIGHT;
					inputListPtr = (inputListPtr + 1) % MAX_NUM_INPUT;
					numInput++;
				} else if (buttonsGetEdge(BUTTON_GREEN)) {
					// Forward
					cprintf("%d = forward\n", numInput);
					inputList[inputListPtr] = TILEMOTION_FORWARD;
					inputListPtr = (inputListPtr + 1) % MAX_NUM_INPUT;
					numInput++;
				}
			}
			break;
		default:
			break;

		}

		neighborsPutMutex();

		osTaskDelayUntil(&lastWakeTime, 20);
	}
}

#if 0
// check the tile game
// make a choice for the next tile, or declare victory
gameStatus = gameFSM(tileCurrentPtr);
if (gameStatus == GAME_STATUS_END_VICTORY) {
	// victory
	cprintf("\nVictory!\n");
	mode = TILEMOTION_MODE_IDLE;
}else if (gameStatus == GAME_STATUS_END_TRY_AGAIN) {
	// epic fail
	cprintf("\nTry again.\n");
	mode = TILEMOTION_MODE_IDLE;
} else {
	// play on
	moveStart(tileCurrentPtr, "rotate");
	mode = TILEMOTION_MODE_MOVE_ROTATE;
}
#endif


int main(void) {
	// init the rone hardware and roneos services
	systemInit();
	systemPrintStartup();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 2048);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}

