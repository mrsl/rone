/*
 * tileGame.h
 *
 *  Created on: Mar 30, 2014
 *      Author: jamesm
 */

#ifndef TILEGAME_H_
#define TILEGAME_H_

#define TILE_HISTORY_MAX						20

#define GAME_STATE_READING						0
#define GAME_STATE_CHECK_GAMESTRING_NEXTTIME	1
#define GAME_STATE_CHECK_GAMESTRING				2

#define GAME_STATUS_CONTINUE					0
#define GAME_STATUS_END_VICTORY					1
#define GAME_STATUS_END_TRY_AGAIN				2


uint8 gameFSM(TileInfo* tilePtr);
void gameFSMInit(void);
char* gameGetHistoryString(void);

#endif /* TILEGAME_H_ */
