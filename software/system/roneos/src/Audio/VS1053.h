#ifndef VS1053_H_
#define VS1053_H_

/* MIDI COMMANDS for the VS1053 chip */
#define MIDI_CHANNEL_EVENT_TYPE_MASK	0xF0
#define MIDI_NOTEOFF	0x80	/* Note off */
#define MIDI_NOTEON		0x90	/* Note on */
#define MIDI_PRESSURE	0xa0	/* Polyphonic key pressure */
#define MIDI_CONTROL	0xb0	/* Control change */
#define MIDI_PROGRAM	0xc0	/* Program change */
#define MIDI_CHANPRES	0xd0	/* Channel pressure */
#define MIDI_PITCHB		0xe0	/* Pitch wheel change */
#define MIDI_SYSEX		0xf0	/* System exclusive data */

#define MIDI_CHANNEL_EVENT_CHANNEL_MASK	0x0F
#define MIDI_CHANNEL_MASK	MIDI_CHANNEL_EVENT_CHANNEL_MASK

#endif
