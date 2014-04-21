/*
 * @file audio.c
 *
 * @since Jul 6, 2011
 * @author Sunny Kim
 */

#if (defined(RONE_V9) || defined(RONE_V12))

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"

#include "roneos.h"
#include "Midi.h"
#include "VS1053.h"

#define DREQ_PERIPH 			SYSCTL_PERIPH_GPIOG
#define DREQ_BASE 				GPIO_PORTG_BASE
#define DREQ_PIN 				GPIO_PIN_0

#define VS1053_SCI_MODE				0x00
#define VS1053_SM_DIFF				0x0001
#define VS1053_SM_TESTS				0x0020
#define VS1053_SM_SDINEW			0x0800

#define VS1053_SCI_STATUS			0x01
#define SOUNDCHIP_STATUS_WRITE		0x01

#define VS1053_SCI_WRAMADDR			0x07
#define SOUNDCHIP_WRAMADDR_WRITE	0xC001

#define VS1053_SCI_WRAM				0x06
#define SOUNDCHIP_WRAM_WRITE		0x01

#define VS1053_SCI_AIADDR			0x0A

#define VS1053_SCI_VOL				0x0B
#define SOUNDCHIP_VOL_DEFAULT		0x0000

#define VS1053_SCI_AICTRL0			0x0C
#define VS1053_SCI_AICTRL1			0x0D

#define SOUNDCHIP_WRITE_CMD 0x02
#define SOUNDCHIP_READ_CMD 0x03
#define SOUNDCHIP_BASE SSI0_BASE

#define SOUNDCHIP_CLOCKF_ADDR 0x03
#define SOUNDCHIP_CLOCKF_MULT 0xA000


//#define AUDIO_NUM_OF_ON_NOTES				1
#define AUDIO_NOTE_OFF_THREAD_PERIOD		10
#define AUDIO_MAX_NOTE_PER_CHANNEL			8
#define AUDIO_NUM_OF_CHANNELS				MIDI_TRACKS_MAX

#define NOTE_OFF_VELOCITY					100

#define SOUND_CHIP_WRITE_DELAY				50
//#define AUDIO_QUEUE_SIZE	50

//#define SPI_WORDNUM				16


/******** Structs ********/

/**
 * @brief information about a single MIDI note
 */
typedef struct audioMIDINote {
	uint8	key;
	uint32 	endTick;
	boolean keydown;
} audioMIDINote;


/**
 * @brief information about a single MIDI channel
 */
typedef struct audioMIDIChannel {
	uint8	noteNum;
	uint8	instrument;
	audioMIDINote OnNotes[AUDIO_MAX_NOTE_PER_CHANNEL];
} audioMIDIChannel;



/******** Globals ********/

static audioMIDIChannel MidiChannels[MIDI_TRACKS_MAX];
static audioMIDINote* noteOffNextPtr = NULL;
static uint8 noteOffNextChannel;
static osSemaphoreHandle audioChannelMutex;

/******** Internal Functions ********/

/**
 * @brief Puts data through SSI.
 *
 * @param c Data to put through SSI.
 */
static void soundChipPutchar(char c) {
	MAP_SSIDataPut(SSI0_BASE, c);
}


/**
 * @brief Changes patch of the given channel.
 *
 * @param instrument Patch number to change to
 * @param channel Channel to change patch number
 * @returns void
 */
static void soundChipChangePatch(uint8 channel, uint8 instrument) {
	//if (channel != 0) {
		SPISelectDevice(SPI_AUDIOSDI);
		soundChipPutchar(MIDI_PROGRAM | channel);
		soundChipPutchar(instrument);
		SPIDeselect();
	//}
}


/**
 * @brief Turns on sound. Sends audioNoteOn data through SPI to soundchip.
 *
 * @param channel Sound channel to use, ranges from 0-8 (why not 0-16?)
 * @param noteNum Note from table to play. ranges from 0-255
 * @param volume Volume to play, ranges from 0-15
 * @returns void
 */
static void soundChipNoteOn(uint8 channel, uint16 noteNum, uint8 volume) {
	SPISelectDevice(SPI_AUDIOSDI);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	MAP_SSIDataPut(SSI0_BASE, MIDI_NOTEON | channel);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	MAP_SSIDataPut(SSI0_BASE, noteNum);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	MAP_SSIDataPut(SSI0_BASE, volume);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	SPIDeselect();
}


/**
 * @brief Turns off sound. Sends audioMIDINoteOff data through SSI to soundchip.
 *
 * @param channel Soundchip channel of the note to turn off
 * @param key Note to turn off
 * @returns void
 */
static void soundChipNoteOff(uint8 channel, uint8 key, uint8 volume) {
	SPISelectDevice(SPI_AUDIOSDI);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	MAP_SSIDataPut(SSI0_BASE, MIDI_NOTEOFF | channel);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	MAP_SSIDataPut(SSI0_BASE, key);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	MAP_SSIDataPut(SSI0_BASE, volume);
	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
	SPIDeselect();
}


/**
 * @brief Reset soundchip. Done whenever hardware is reset.
 *
 * @returns void
 */
static void soundChipReset(void) {
//	spi_select_device(SPI_AUDIO, SPI_WORDNUM);
//	audioputchar(0x00);
//	spi_deselect();

//	SPISelectDevice(SPI_AUDIOSDI);
//	soundChipPutchar(0xFF);
//	SPIDeselect();
}


/**
 * @brief Write to soundchip registers.
 *
 * @param addr Address of the register to write
 * @param data Data to write to register. See datasheet for soundchip for detailed commands.
 * @returns void
 */
static void soundChipWriteCommand(uint8 addr,uint16 data) {
	int i;
	uint16 addrIn = 0;
	uint16 dataIn = 0;

	SPISelectDevice(SPI_AUDIOSCI);
	MAP_SSIDataPut(SSI0_BASE, ((uint16)SOUNDCHIP_WRITE_CMD << 8) | addr);
	MAP_SSIDataPut(SSI0_BASE, data);
	SPIDeselect();
}


/**
 * @brief Turns off sound. Sends audioMIDINoteOff data through SSI to soundchip.
 *
 * @param channel Soundchip channel of the note to turn off
 * @param key Note to turn off
 * @returns void
 */
void soundChipTest(uint8 test) {
	// enable SDI tests
//	soundChipWriteCommand((uint8)VS1053_SCI_MODE, VS1053_SM_DIFF | VS1053_SM_SDINEW | VS1053_SM_TESTS);
//
//	// sens sin wave test command
//	SPISelectDevice(SPI_AUDIOSDI);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0x53);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0xEF);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0x6E);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, test);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	MAP_SSIDataPut(SSI0_BASE, 0);
//	SysCtlDelay(SOUND_CHIP_WRITE_DELAY);
//	SPIDeselect();


	soundChipWriteCommand(VS1053_SCI_AIADDR, 0x4020);

	// play 440 hz
	soundChipWriteCommand(VS1053_SCI_AICTRL0, 658);
	soundChipWriteCommand(VS1053_SCI_AICTRL1, 658);

	//soundChipWriteCommand(VS1053_SCI_AIADDR, 0x4022);
}

void playMajorChord(uint8 instrument, uint8 baseNote, uint8 volume, uint32 duration) {
	audioNoteOn(instrument, baseNote + middleC, volume, duration);
	audioNoteOn(instrument, baseNote+MajorThird + middleC, volume, duration);
	audioNoteOn(instrument, baseNote+Perfect5th + middleC, volume, duration);
	audioNoteOn(instrument, baseNote+oneOctave + middleC, volume, duration);
}

/**
 * @brief Plays a three note minor chord
 *
 * @param volume   the output level for left and right channels. 0 = total silence, 255 = max volume
 * @param baseNote root of the chord (played at baseNote + middleC)
 * @param volume   the output level for left and right channels. 0 = total silence, 255 = max volume
 * @param duration Duration of the note (ms)
 * @returns void
 */
void playMinorChord(uint8 instrument, uint8 baseNote, uint8 volume, uint32 duration) {
	audioNoteOn(instrument, baseNote + middleC - oneOctave, volume, duration);
	audioNoteOn(instrument, baseNote+MinorThird + middleC, volume, duration);
	audioNoteOn(instrument, baseNote+Perfect5th + middleC, volume, duration);
	audioNoteOn(instrument, baseNote+oneOctave + middleC, volume, duration);
}


/**
 * @brief Normalize tick.
 *
 * @param tick Tick to normalize
 * @returns normalized tick count.
 */
////TODO: find out appropriate tick normalize number
//static uint32 tickNormalize(uint32 tick) {
//	if (tick >= 0xFFFFFFFF) {
//		tick -= 0xFFFFFFFF;
//	}
//	return (tick);
//}
static uint32 tickNormalize(uint32 tick) {
	return tick;
}


/**
 * @brief Inits the data structure for keeping track of notes that are on and off.
 *
 * @returns void
 */
static void noteChannelInit(void) {
	uint8 i;
	uint8 note;

	osSemaphoreTake(audioChannelMutex, portMAX_DELAY);
	for (i=0; i < AUDIO_NUM_OF_CHANNELS; i++) {
		for (note=0; note < AUDIO_MAX_NOTE_PER_CHANNEL; note++) {
			MidiChannels[i].OnNotes[note].key = 0;
			MidiChannels[i].OnNotes[note].keydown = FALSE;
			MidiChannels[i].OnNotes[note].endTick = 0;
		}
		MidiChannels[i].noteNum = 0;
		MidiChannels[i].instrument = 0;
	}
	osSemaphoreGive(audioChannelMutex);
}


/**
 * @brief Thread that turns off notes. Not used by midi files.
 *
 * @returns void
 */
void audioThread(void* parameters) {
	uint8 channel;
	uint8 note;
	uint32 currentTick;
	uint32 updateTick = osTaskGetTickCount();
	audioMIDINote* notePtr;


	for (;;) {
		osSemaphoreTake(audioChannelMutex, portMAX_DELAY);
		currentTick = osTaskGetTickCount();
//		if (currentTick > noteOffNextPtr->endTick) {
//			// stop the next note to turn off
//			if (noteOffNextPtr->keydown) {
//				soundChipNoteOff(noteOffNextChannel, noteOffNextPtr->key, NOTE_OFF_VELOCITY);
//				noteOffNextPtr->keydown = false;
//				MidiChannels[noteOffNextChannel].noteNum--;
//			}
//		}
		//for all channels, for all notes, see if any note expired and if yes, turn off.
		for (channel = 0; channel < AUDIO_NUM_OF_CHANNELS; channel++) {
			for (note = 0; note < AUDIO_MAX_NOTE_PER_CHANNEL; note++) {
				notePtr = &MidiChannels[channel].OnNotes[note];
				if ((notePtr->endTick != 0) && (currentTick >= notePtr->endTick) && notePtr->keydown) {
					soundChipNoteOff(channel, notePtr->key, NOTE_OFF_VELOCITY);
					notePtr->keydown = FALSE;
					MidiChannels[channel].noteNum--;
				}
			}
		}
		osSemaphoreGive(audioChannelMutex);
		// make time for other tasks to run.
		osTaskDelayUntil(&updateTick, AUDIO_NOTE_OFF_THREAD_PERIOD);
	}
}


// ************************ functions in audio.h (public) ***************************************
void audioNoteOn(uint8 instrument, uint8 key, uint8 velocity, uint32 duration) {
	int32 channel = -1;
	int32 noteIdx = -1;
	int32 i;
	uint32 currentTick;
	uint32 endTick;
	audioMIDINote* notePtr;

	// get the mutex
	osSemaphoreTake(audioChannelMutex, portMAX_DELAY);

	// look for a channel with the same patch (instrument)
	for (i = 0; i < AUDIO_NUM_OF_CHANNELS; i++){
		if ((instrument == MidiChannels[i].instrument) && (MidiChannels[i].noteNum < (AUDIO_MAX_NOTE_PER_CHANNEL - 1))) {
			channel = i;
			break;
		}
	}

	// Not playing that instrument? look for an empty channel
	if (channel == -1) {
		for (i = 0; i < AUDIO_NUM_OF_CHANNELS; i++){
			if (MidiChannels[i].noteNum == 0) {
				MidiChannels[i].instrument = instrument;
				channel = i;
				soundChipChangePatch(channel, instrument);
				break;
			}
		}
	}

	if (channel >= 0) {
		currentTick = osTaskGetTickCount();
		endTick = tickNormalize(currentTick + duration);

		for (i = 0; i < AUDIO_MAX_NOTE_PER_CHANNEL; i++){
			if (MidiChannels[channel].OnNotes[i].keydown == FALSE) {
				noteIdx = i;
				break;
			}
		}

		if (noteIdx >= 0) {
			notePtr = &(MidiChannels[channel].OnNotes[noteIdx]);
			notePtr->key = key;
			notePtr->endTick = endTick;
			notePtr->keydown = TRUE;
			MidiChannels[channel].noteNum++;

			if ((noteOffNextPtr == NULL) || (endTick < noteOffNextPtr->endTick)) {
				noteOffNextPtr = notePtr;
				noteOffNextChannel = channel;
			}
			velocity = bound(velocity, AUDIO_VELOCITY_MIN, AUDIO_VELOCITY_MAX);
			soundChipNoteOn(channel, key, velocity);
		}
	}
	osSemaphoreGive(audioChannelMutex);
}


void audioNoteOff(uint8 instrument, uint8 key) {
	uint8 note, channel;
	boolean foundNote = FALSE;
	audioMIDINote* notePtr;

	if(channel >= 0) {
		osSemaphoreTake(audioChannelMutex, portMAX_DELAY);
		for (channel = 0; channel < AUDIO_NUM_OF_CHANNELS; channel++) {
			if (MidiChannels[channel].instrument == instrument) {
				for (note = 0; note < AUDIO_MAX_NOTE_PER_CHANNEL; note++) {
					notePtr = &MidiChannels[channel].OnNotes[note];
					if ((notePtr->key == key) && notePtr->keydown) {
						soundChipNoteOff(channel, key, NOTE_OFF_VELOCITY);
						notePtr->keydown = FALSE;
						MidiChannels[channel].noteNum--;
						foundNote = TRUE;
						break;
					}
				}
			}
			if(foundNote) {
				break;
			}
		}
		osSemaphoreGive(audioChannelMutex);
	}
}


void audioNoteOffAll(void) {
	uint8 channel;
	uint8 note;
	audioMIDINote* notePtr;

	osSemaphoreTake(audioChannelMutex, portMAX_DELAY);
	for (channel = 0; channel < AUDIO_NUM_OF_CHANNELS; channel++) {
		for (note = 0; note < AUDIO_MAX_NOTE_PER_CHANNEL; note++) {
			notePtr = &MidiChannels[channel].OnNotes[note];
			if (notePtr->keydown) {
				soundChipNoteOff(channel, notePtr->key, NOTE_OFF_VELOCITY);
				notePtr->keydown = FALSE;
				MidiChannels[channel].noteNum--;
			}
		}
	}
	osSemaphoreGive(audioChannelMutex);
}



void audioVolume(uint8 volume){
	uint16 data;

	// compute the attenuation level
	uint8 level = 0xFF - volume;

	// limit max attenuation to 0xFE to avoid auto power shut off
	if(level == 0xFF) {
		level = 0xFE;
	}

	// build the left and right volume data word
	data = (((uint16)level) << 8) | ((uint16)level);

	// write the sound chip
	soundChipWriteCommand((uint8)VS1053_SCI_VOL, data);
}

void audioMIDIChangePatch(uint8 channel, uint8 patch) {
	soundChipChangePatch(channel, patch);
}

void audioMIDINoteOn(uint8 channel, uint8 key, uint8 velocity) {
	soundChipNoteOn(channel, key, velocity);
}

void audioMIDINoteOff(uint8 channel, uint8 key, uint8 velocity) {
	soundChipNoteOff(channel, key, velocity);
}


// ************************ end functions in audio.h ***************************************


void audioInit(void){
	uint32 val;
	int i;

	// Initialize the midi parser
	audioChannelMutex = osSemaphoreCreateMutex();

	noteChannelInit();

	// init audio thread
	osTaskCreate(audioThread, "audio", 1024, NULL, BACKGROUND_TASK_PRIORITY );

	// select differential output
	soundChipWriteCommand((uint8)VS1053_SCI_MODE, VS1053_SM_DIFF | VS1053_SM_SDINEW);

	audioVolume(AUDIO_VOLUME_MAX);

	// I have no clue what this does
	soundChipWriteCommand((uint8)VS1053_SCI_STATUS, (uint16)SOUNDCHIP_STATUS_WRITE);
//	soundchip_write_command(VS1053_SCI_WRAMADDR, SOUNDCHIP_WRAMADDR_WRITE);
//	soundchip_write_command(VS1053_SCI_WRAM, SOUNDCHIP_WRAM_WRITE);

	// reset the sound chip
	soundChipReset();
	osTaskDelay(10);

	// First change clock freq
	// Second do a software reset
	// Third enable MIDI mode

	//TODO: figure out if we want to use SCI or SDI to set CLOCKF and MODE registers
//	MAP_SysCtlPeripheralEnable(AUDIOSCI_SELECT_SYSCTL);
//	MAP_GPIOPinTypeGPIOOutput(AUDIOSCI_SELECT_PORT, AUDIOSCI_SELECT_PIN);

	//TODO: decide whether we need dreq or not
	//	dreq_init();

	//CS (chip select) to 0V, not really necessary but I've done it anyway.
	//RST (reset) to 3.3V, this is a must, otherwise IC goes into shutdown mode.
//	MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
	//GPIO0 unconnected and GPIO1 to 3.3V, sets IC to Real Time MIDI mode.
	//GPIO2 and GPIO3 to 3.3V Sets the output volume.

//	//TODO: check if we have to create thread here
//	val = osTaskCreate( /* function */ RTmiditest,
//					  /* name */ "RTmiditest",
//				/* stack size */ 1024,
//		   /* *char parameter */ NULL,
//				  /* priority */ BACKGROUND_TASK_PRIORITY);

}



//#define VOLUME					200
//#define INSTRUMENT					MIDI_Shakuhachi
//
///**
// * @brief Play Harry Potter theme song.
// *
// * @returns void
// */
//void playHarryPotter(void) {
//	//Harry Potter
//	noteOnWithTaskDelay(middleC-oneOctave+BNote, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 750);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 250);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 1000);
//
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+ANote, VOLUME, INSTRUMENT, 1500);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 1500);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 750);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 250);
//
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+DNote+1, VOLUME, INSTRUMENT, 1000);
//	noteOnWithTaskDelay(middleC+FNote, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC-oneOctave+BNote, VOLUME, INSTRUMENT, 2500);
//	noteOnWithTaskDelay(middleC-oneOctave+BNote, VOLUME, INSTRUMENT, 500);
//
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 750);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 250);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 1000);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 500);
//
//	noteOnWithTaskDelay(middleC+oneOctave+DNote, VOLUME, INSTRUMENT, 1000);
//	noteOnWithTaskDelay(middleC+oneOctave+DNote-1, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+oneOctave+CNote, VOLUME, INSTRUMENT, 1000);
//	noteOnWithTaskDelay(middleC+ANote-1, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+oneOctave+CNote, VOLUME, INSTRUMENT, 750);
//
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 250);
//	noteOnWithTaskDelay(middleC+BNote-1, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC-oneOctave+BNote-1, VOLUME, INSTRUMENT, 1000);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 500);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 2500);
//}
//
//
///**
// * @brief Play Angry Bird theme song.
// *
// * @returns void
// */
//void playAngryBird(void) {
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 300);
//
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 600);
//
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ANote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 150);
//
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 150);
//
//	noteOnWithTaskDelay(middleC+ANote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 150);
//
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ENote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 300);
//
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ENote+oneOctave, VOLUME, INSTRUMENT, 150);
//
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+BNote, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+DNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+oneOctave+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ANote, VOLUME, INSTRUMENT, 150);
//
//
//
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 600);
//	osTaskDelay(600);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 450);
//	noteOnWithTaskDelay(middleC+DNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 300);
//
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 450);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+ANote, VOLUME, INSTRUMENT, 300);
//
//	audioNoteOn(middleC+ANote, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote-1, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+ANote, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote-1, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+ANote, VOLUME, INSTRUMENT, 600);
//	audioNoteOn(middleC+BNote-1, VOLUME, INSTRUMENT, 600);
//	osTaskDelay(600);
//
//	audioNoteOn(middleC+ANote+oneOctave, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote+oneOctave-1, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+ANote+oneOctave, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote+oneOctave-1, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+ANote+oneOctave, VOLUME, INSTRUMENT, 600);
//	audioNoteOn(middleC+BNote+oneOctave-1, VOLUME, INSTRUMENT, 600);
//	osTaskDelay(600);
//
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 450);
//	noteOnWithTaskDelay(middleC+DNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+ENote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 450);
//	noteOnWithTaskDelay(middleC+FNote+1, VOLUME, INSTRUMENT, 150);
//	noteOnWithTaskDelay(middleC+GNote, VOLUME, INSTRUMENT, 300);
//	noteOnWithTaskDelay(middleC+ANote, VOLUME, INSTRUMENT, 300);
//
//	audioNoteOn(middleC+oneOctave+1, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+oneOctave+1, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote+oneOctave, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+oneOctave+1, VOLUME, INSTRUMENT, 600);
//	audioNoteOn(middleC+BNote+oneOctave, VOLUME, INSTRUMENT, 600);
//	osTaskDelay(600);
//
//	audioNoteOn(middleC+twoOctaves+1, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote+twoOctaves, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+twoOctaves+1, VOLUME, INSTRUMENT, 300);
//	audioNoteOn(middleC+BNote+twoOctaves, VOLUME, INSTRUMENT, 300);
//	osTaskDelay(300);
//	audioNoteOn(middleC+twoOctaves+1, VOLUME, INSTRUMENT, 600);
//	audioNoteOn(middleC+BNote+twoOctaves, VOLUME, INSTRUMENT, 600);
//	osTaskDelay(600);
//}
//
//
///**
// * @brief Play C major cord.
// *
// * @returns void
// */
//void playMajorC(volume, duration) {
//	audioNoteOn(middleC, volume, MIDI_piano, duration);
//	audioNoteOn(middleC+ENote, volume, MIDI_piano, duration);
//	audioNoteOn(middleC+GNote, volume, MIDI_piano, duration);
//	audioNoteOn(middleC+oneOctave, volume, MIDI_piano, duration);
//}


//boolean audio_get_note(audioMIDINote* NotePtr) {
//
//	portBASE_TYPE val;
//
//	val = osQueueReceive(audioQueue, (void*)(NotePtr), 0);
//    if (val == pdPASS) {
//        return TRUE;
//    } else {
//    	NotePtr->key = 0;
//    	NotePtr->velocity = 0;
//    	NotePtr->audioNoteOn = FALSE;
//        return FALSE;
//    }
//}
//
//void dreq_init(void) {
//	MAP_SysCtlPeripheralEnable(DREQ_PERIPH);
//	MAP_GPIOPinTypeGPIOInput(DREQ_BASE, DREQ_PIN);
//}
//
//uint32 dreq_check(void) {
//	uint8 val = FALSE;
//
//	// For SDI input DREQ is needed but not for UART Real time MIDI.
//	if (MAP_GPIOPinRead(DREQ_BASE, DREQ_PIN)) {
//		val = TRUE;
//	} else {
//		val = FALSE;
//	}
//
//	return val;
//}
#endif
