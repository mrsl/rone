/*
 * debug_navbeacon.c
 *
 *     Created on: 2012-11-05
 *         Author: jmclurkin
 *        Summary: This code is to test detection, orientation, and navigation to the nav towers
 * Current Status: working {unknown, working, defunct, inProgress}
 */
#include "roneos.h"
#include "ronelib.h"
#include "playworks.h"


static osQueueHandle soundMsgQueue;

#define SOUND_TYPE_TILE		0

typedef struct SoundMessage {
	uint8 soundType;
	uint8 tileID;
} SoundMessage;


#define SOUND_NUMBER_DELAY	100
#define SOUND_NUMBER_NOTE	(middleC + oneOctave)
#define SOUND_NUMBER_PATCH 	MIDI_GLOCKENSPIEL

// Sounds run whever they get a message to play.
void soundTask(void* parameters) {
	portBASE_TYPE val;
	uint8 i;
	SoundMessage soundMsg;

	while(TRUE) {
		val = osQueueReceive(soundMsgQueue, (void*)(&soundMsg), portMAX_DELAY);
		if (val == pdPASS) {
			// play a sound!
			switch (soundMsg.soundType) {
			case SOUND_TYPE_TILE: {
				// play a two-note tile message
				if (soundMsg.tileID != 0) {
					switch (soundMsg.tileID) {
					case TILE_1:
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						break;
					case TILE_2:
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						osTaskDelay(SOUND_NUMBER_DELAY);
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						break;
					case TILE_3:
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						osTaskDelay(SOUND_NUMBER_DELAY);
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						osTaskDelay(SOUND_NUMBER_DELAY);
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						break;
					case TILE_4:
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						osTaskDelay(SOUND_NUMBER_DELAY);
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						osTaskDelay(SOUND_NUMBER_DELAY);
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						osTaskDelay(SOUND_NUMBER_DELAY);
						audioNoteOn(SOUND_NUMBER_PATCH, SOUND_NUMBER_NOTE, 127, 75);
						break;
					case TILE_PLUS:
						audioNoteOn(MIDI_TRUMPET, middleC + ENote, 127, 200);
						osTaskDelay(100);
						audioNoteOn(MIDI_TRUMPET, middleC + GNote, 127, 200);
						break;
					case TILE_EQUAL:
						audioNoteOn(MIDI_TRUMPET, middleC + oneOctave, 127, 200);
						audioNoteOn(MIDI_TRUMPET, middleC + oneOctave + GNote, 127, 200);
						break;
					default:
						break;
					}
				}
				break;
			}
			default:
				// do nothing
				break;
			}
		}
	}
}


void playworksSoundInit(void) {
	// make the sound msg command queue
    soundMsgQueue = osQueueCreate(1, sizeof(SoundMessage));
    audioVolume(AUDIO_VOLUME_MAX);

	// start the sound thread
	osTaskCreate(soundTask, "sound", 1024, NULL, BEHAVIOR_TASK_PRIORITY);
}


void playTileSound(uint8 tileID) {
	SoundMessage soundMsg;
	soundMsg.soundType = SOUND_TYPE_TILE;
	soundMsg.tileID = tileID;
	osQueueSend(soundMsgQueue, (void*)(&soundMsg), 0);
}

