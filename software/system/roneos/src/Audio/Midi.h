/**
 * @file Midi.h
 *
 * @brief Functions for playing MIDI files on the robot.
 * @since Jul 20, 2011
 * @author: Sunny Kim
 */

#ifndef MIDI_H_
#define MIDI_H_

/******** Defines ********/

#define		MIDI_NUM_OF_PATCHES	    		128
#define 	MIDI_TRACKS_MAX	                16

#include	"VS1053_PatchTable.h"


/******** Functions ********/

/**
 * @brief Initialize midi configuration to default.
 *
 * @returns void
 */
void MIDIInit(void);

/**
 * @brief Play midi file.
 *
 * @param MIDIFilePtr Pointer for midi file to play
 * @returns void
 */
void MIDIFilePlay(const char* MIDIFilePtr);

/**
 * @brief Check and see if a MIDI file is playing.
 *
 * @returns TRUE is there is a MIDI file playing
 */
boolean MIDIFileIsPlaying(void);

#endif /* MIDI_H_ */
