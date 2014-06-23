///*
// * flock.c
// *
// *  Created on: Sept 19, 2011
// *      Author: lyncas
// */
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//
//#include "roneos.h"
//#include "ronelib.h"
//
//
//void playStartSoundFlock(void)
//{
//	//Star Wars main theme first 5 measures
//	uint8 baseDuration = 200;
//	uint8 baseNote = middleC;
//	noteOnWithDelay(baseNote-10, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-10, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-10, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-5, 200, trumpet, baseDuration*5);
//	noteOnWithDelay(baseNote+2, 200, trumpet, baseDuration*5);
//
//	noteOnWithDelay(baseNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-1, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-3, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote+7, 200, trumpet, baseDuration*5);
//	noteOnWithDelay(baseNote+2, 200, trumpet, baseDuration*2.5);
//
//	noteOnWithDelay(baseNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-1, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-3, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote+7, 200, trumpet, baseDuration*5);
//	noteOnWithDelay(baseNote+2, 200, trumpet, baseDuration*2.5);
//
//	noteOnWithDelay(baseNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-1, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote, 200, trumpet, baseDuration);
//	noteOnWithDelay(baseNote-3, 200, trumpet, baseDuration*5);
//
//	audioNoteOffAll();
//}
//
//
//void bumpSound(void)
//{
//	//Sound to be played once the bump sensor is triggered
//	uint8 baseDuration = 200;
//	uint8 baseNote = middleC;
//	noteOnWithDelay(baseNote, 200, acousticBass, baseDuration);
//	noteOnWithDelay(baseNote-7, 200, acousticBass, baseDuration);
//	audioNoteOffAll();
//}
//
