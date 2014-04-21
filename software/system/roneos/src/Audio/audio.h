/**
 * @file audio.h
 *
 * @since Jul 6, 2011
 * @author Sunny Kim
 *
 * @brief Files for controlling audio speaker and MIDI files.
 */

#ifndef AUDIO_H_
#define AUDIO_H_

/******** Defines ********/

#define OCTAVE_MAX						  		10
#define NOTES_IN_ONE_OCTAVE						12
#define MajorThird								4
#define MinorThird								3
#define Perfect5th								7
#define oneOctave								NOTES_IN_ONE_OCTAVE
#define twoOctaves								(NOTES_IN_ONE_OCTAVE * 2)
#define threeOctaves 							(NOTES_IN_ONE_OCTAVE * 3)
#define fourOctaves								(NOTES_IN_ONE_OCTAVE * 4)
#define middleC									0x3C

#define CNote									0
#define CsNote									1
#define DNote									2
#define DsNote									3
#define ENote									4
#define FNote									5
#define FsNote									6
#define GNote									7
#define GsNote									8
#define ANote									9
#define AsNote									10
#define BNote									11
#define CNoteUp									12

#include "VS1053_PatchTable.h"

#define AUDIO_VOLUME_MAX   						255
#define AUDIO_VOLUME_MIN     					0

#define AUDIO_VELOCITY_MIN						0
#define AUDIO_VELOCITY_MAX						127

/******** Structs ********/

/******** Functions ********/

/**
 * @brief Initialize audio chip.
 *
 * @returns void
 */
void audioInit(void);


/**
 * @brief Plays given note for given time.  This will schedule a note off message for the sound chip.
 *
 * @param instrument Instrument to play
 * @param key Note to play
 * @param velocity the velocity (volume) to play. range is from 0-127
 * @param duration Duration of the note (ms)
 */
void audioNoteOn(uint8 instrument, uint8 key, uint8 velocity, uint32 duration);


/**
 * @brief Turns off given note.
 *
 * @param instrument Instrument to stop
 * @param key Note to turn off
 * @returns void
 */
void audioNoteOff(uint8 instrument, uint8 key);


/**
 * @brief Turns off all notes.
 * @returns void
 */
void audioNoteOffAll(void);


/**
 * @brief Set the volume level.
 *
 * @param volume   the output level for left and right channels. 0 = total silence, 255 = max volume
 * @returns void
 */
void audioVolume(uint8 volume);

//TODO: what does this actually do?
/**
 * @brief Turns off sound. Sends audioMIDINoteOff data through SSI to soundchip.
 *
 * @param test
 * @returns void
 */
void soundChipTest(uint8 test);


/**
 * @brief Plays a three note major chord
 *
 * @param instrument
 * @param baseNote root of the chord (played at baseNote + middleC)
 * @param volume   the output level for left and right channels. 0 = total silence, 255 = max volume
 * @param duration Duration of the note (ms)
 * @returns void
 */
void playMajorChord(uint8 instrument, uint8 baseNote, uint8 volume, uint32 duration);


/**
 * @brief Plays a three note minor chord
 *
 * @param instrument
 * @param baseNote root of the chord (played at baseNote + middleC)
 * @param volume   the output level for left and right channels. 0 = total silence, 255 = max volume
 * @param duration Duration of the note (ms)
 * @returns void
 */
void playMinorChord(uint8 instrument, uint8 baseNote, uint8 volume, uint32 duration);
#endif /* AUDIO_H_ */
