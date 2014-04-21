/*
 * tileGame.c
 *
 *  Created on: Mar 30, 2014
 *      Author: jamesm
 */

#include <string.h>
#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"


TileInfo* tileHistory[TILE_HISTORY_MAX];
uint8 tileHistoryCounter = 0;

uint8 gameFSMState = GAME_STATE_READING;

const char gameStringsValid[][TILE_HISTORY_MAX] = {
	"1+2=3",
	"2+1=3",
	"1+3=4",
	"3+1=4",
	"2+2=4",
	"",
};


char* gameGetHistoryString(void) {
	//TODO support different length names
	uint8 i;
	static char gameString[TILE_HISTORY_MAX + 1];
	for (i = 0; i < tileHistoryCounter; i++) {
		gameString[i] = tileGetName(tileHistory[i])[0];
	}
	gameString[i] = 0;
	return gameString;
}


void gameFSMInit(void) {
	gameFSMState = GAME_STATE_READING;
}


uint8 gameFSM(TileInfo* tilePtr) {
	/* add the tile to the history
	 */
	uint8 gameStatus = GAME_STATUS_CONTINUE;

	tileHistory[tileHistoryCounter++] = tilePtr;
	char * gameString = gameGetHistoryString();
	cprintf("\nTP,debug,history=%s\n", gameString);

//	if (gameFSMState == GAME_STATE_CHECK_GAMESTRING_NEXTTIME) {
//		gameFSMState = GAME_STATE_CHECK_GAMESTRING;
//		// check the expression in the history
//		uint8 i = 0;
//		gameStatus = GAME_STATUS_END_TRY_AGAIN;
//		do {
//			if (strncmp(gameString, gameStringsValid[i], TILE_HISTORY_MAX) == 0) {
//				// we have a valid history string.  Hooray!
//				gameStatus = GAME_STATUS_END_VICTORY;
//				break;
//			}
//			i++;
//		} while (gameStringsValid[i] != 0);
//	}
//
//	if (tileGetType(tilePtr) == TILE_EQUAL) {
//		gameFSMState = GAME_STATE_CHECK_GAMESTRING_NEXTTIME;
//	}
	return gameStatus;
}
