/** demo_MassiveManipulate3Controllers.c
 *
 *     @since: 2/28/2013
 *     @author: ab55, smd2, gh4 (Aaron Becker and Sarah)
 *     @brief: One robot is the remote control, mapping x gravity to fwd/backwards and y gravity to left/right
 *     All other robots use this command as their input using WiFi, in 3 different ways:
 *
 *     All robots start as followers.  To make one the leader, hold 'R'&'G' button for 2 seconds.  To change to follower, 'G'&'B' button for 2 seconds
 *
 *		@todo: use flocking or a global AprilTag signal to keep robots aligned.
 *		@todo: use a complimentary filter with gyro and accelerometer to improve RC control.
 *		@todo: tune the deadzone and speed scaling.  (what is the maximum turn and speed values allowed on the robot?)
 *		@todo: flush the queue when they get a message
 *		@todo: bump behavior when they hit a target (either turn left or right and move 1/2 body length)
 *
 *  the follower is not updating it's state based on the commands from the controller....
 * Current Status: inProgress {unknown, working, defunct, inProgress}
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "roneos.h"
#include "ronelib.h"

//stuff for flocking
#define DEMO_TV_FLOCK					0
#define TV_MIN							15
#define RADIO_MESSAGE_PERSISTANCE		200
#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50


/******** user code ********/
void RCusingAccelerometer(void);

RadioMessage radioMessageTX;
RadioMessage radioMessageRX;
RadioCmd radioCmdRemoteControl;
char* name = "RCwifi";

enum ENS_CONTROL         // Defines an enumeration type
{
	C_SERIAL = 0, //Red
	C_LOCAL = 1,  //Green
	C_GLOBAL = 2, //Blue
};


void playMajorC(uint8 volume, uint32 duration) {
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC+MajorThird, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC+Perfect5th, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC+oneOctave, volume, duration);
}

void playMinorC(uint8 volume, uint32 duration) {
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC+MinorThird, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC+Perfect5th, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, middleC+oneOctave, volume, duration);
}


void playMajorChord(uint8 baseNote, uint8 volume, uint32 duration) {
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote + middleC - oneOctave, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote+MajorThird + middleC - oneOctave, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote+Perfect5th + middleC - oneOctave, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote+oneOctave + middleC - oneOctave, volume, duration);
}

void playMinorChord(uint8 baseNote, uint8 volume, uint32 duration) {
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote + middleC - oneOctave, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote+MinorThird + middleC - oneOctave, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote+Perfect5th + middleC - oneOctave, volume, duration);
	audioNoteOn(MIDI_ACOUSTIC_GRAND_PIANO, baseNote+oneOctave + middleC - oneOctave, volume, duration);
}



void behaviorTask(void* parameters) {

	int numRobots = 3; // num follower robots (don't count the leader/RC robot)
	int robotIDs[3] = {61,65,66}; // follower robots


	uint32 lastWakeTime = osTaskGetTickCount();
	uint8 orientationBits = 0;
	uint8 bumpSensorBits, irRangeBits;

	enum ENS_CONTROL ensControl = C_SERIAL;

	//static int32 motor_vel = 0;
	int redPressCount = 0;
	int greenPressCount = 0;
	int bluePressCount = 0;
	int redgreenPressCount = 0;
	int greenbluePressCount = 0;
	int SERIAL_count = 0;



	int isLeader = 0;
	boolean buttonPress;
	char TXmsg[RADIO_COMMAND_MESSAGE_DATA_LENGTH];
	uint32 TimePrev = 0;
	uint32 Time  = osTaskGetTickCount();
	int32 accX = 0;
	int32 accY = 0;
	int32 gyroX, gyroY;
	uint32 radioMessageTimePrev = 0;

	// init the radio behavior to inactive only once
	Beh behRadio = behInactive;
	Beh behOutput, behMove, behIRObstacle, behBump;
	int32 TVcmd, RVcmd;
	NbrList nbrList;
	int wifiControlState;
	int wifiControlStateOld = 0;
	int wifiControlCount;
	int wifiControlCountOld = 0;

	systemPrintStartup();
	neighborsInit(300);

	// make a receive radio queue that is one message deep
	radioCommandAddQueue(&radioCmdRemoteControl,name, 1);
	while (TRUE) {
		// init the output behavior to inactive
		behOutput = behInactive;

		//just use the accelerometer data and ignore gyro for now...
		accX = (((int32)accelerometerGetValue(ACCELEROMETER_X) * ACCEL_IIR_GAIN) + (accX * (100 - ACCEL_IIR_GAIN))) / 100;
		accY = (((int32)accelerometerGetValue(ACCELEROMETER_Y) * ACCEL_IIR_GAIN) + (accY * (100 - ACCEL_IIR_GAIN))) / 100;
		accX = deadzone(accX, ACCEL_DEAD_ZONE);
		accY = deadzone(accY, ACCEL_DEAD_ZONE);
		gyroX = gyroGetValue(GYRO_X_AXIS);
		gyroY = gyroGetValue(GYRO_Y_AXIS);



		// toggle if this robot is the leader, or if it is a follower
		if (buttons_get(BUTTON_RED) && buttons_get(BUTTON_GREEN) ) {
			redgreenPressCount++;
			if(redgreenPressCount > 20)
			{ isLeader = 1;
			playMajorC(AUDIO_VELOCITY_MAX*3/4, 400);
			redgreenPressCount = 0;
			}
		}else if (buttons_get(BUTTON_GREEN) && buttons_get(BUTTON_BLUE) ) {
			greenbluePressCount++;
			if(greenbluePressCount > 20)
			{ isLeader = 0;
			playMinorC(AUDIO_VELOCITY_MAX*3/4, 400);
			greenbluePressCount = 0;
			}
		}else if (buttons_get(BUTTON_RED)) {
			redPressCount++;
			if(redPressCount > 10)
			{ ensControl = C_SERIAL;
			SERIAL_count++;
			playMajorChord(DNote, AUDIO_VELOCITY_MAX*3/4, 200);
			redPressCount = 0; //reset counter
			}
		}else if (buttons_get(BUTTON_GREEN)) {
			bluePressCount++;
			if(bluePressCount > 10)
			{ ensControl = C_LOCAL;
			playMajorChord(FNote, AUDIO_VELOCITY_MAX*3/4, 200);
			bluePressCount = 0;
			}
		}else if (buttons_get(BUTTON_BLUE)) {
			greenPressCount++;
			if(greenPressCount > 10)
			{ ensControl = C_GLOBAL;
			playMajorChord(GNote, AUDIO_VELOCITY_MAX*3/4, 200);
			greenPressCount = 0;
			}
		}else{  //zero all counters
			redgreenPressCount = 0;
			greenbluePressCount = 0;
			redPressCount   = 0;
			greenPressCount = 0;
			bluePressCount  = 0;
		}

		nbrListCreate(&nbrList);

		//if the leader, send command
		if( isLeader ) {
			switch(ensControl){
			case C_SERIAL:
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				break;
			case C_LOCAL:
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				break;
			case C_GLOBAL:
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				break;
			}

			///////////////////// Leaders send WiFi ///////////////////////////
			int sc = 5;
			TVcmd = -accX/sc;
			RVcmd = -accY * 2;
			if (abs(TVcmd) < TV_MIN) {
				if(TVcmd > 0) {
					TVcmd = TV_MIN;
				} else {
					TVcmd = -TV_MIN;
				}
			}

			// start with 2 characters that are discarded.  We'll use 'RC'
			sprintf(radioCommandGetDataPtr(&radioMessageTX),"RC%d,%d,%d,%d",TVcmd, RVcmd,ensControl,SERIAL_count%(numRobots+1));
			cprintf(radioCommandGetDataPtr(&radioMessageTX));
			radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
		}else{
			///////////////////// Followers  get WiFi ///////////////////////////
			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			if( radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX,0) ) {
				char* RXmsg = radioCommandGetDataPtr(&radioMessageRX);
				radioMessageTimePrev = osTaskGetTickCount();
				if( RXmsg[0] == 'R' && RXmsg[1] == 'C'){
					sscanf(&RXmsg[2],"%d,%d,%d,%d", &TVcmd, &RVcmd,&wifiControlState,&wifiControlCount); //parse the speed and turning rate
					cprintf("F:% 6d,% 6d,% 6d,% 6d",TVcmd,RVcmd,wifiControlState,wifiControlCount);
					switch( wifiControlState)
					{
					case C_SERIAL:
						if( ensControl != wifiControlState)
						{  //sing!
							playMinorChord(DNote, AUDIO_VELOCITY_MAX*3/4, 200);
						}
						if(wifiControlCount != wifiControlCountOld )
						{  //sing!
							playMinorChord(CNote, AUDIO_VELOCITY_MAX*3/4, 200);
						}
						ensControl =C_SERIAL;
						break;
					case C_LOCAL:
						if( ensControl != wifiControlState)
						{  //sing!
							playMajorChord(FNote, AUDIO_VELOCITY_MAX*3/4, 200);
						}
						ensControl =C_LOCAL;
						break;
					case C_GLOBAL:
						if( ensControl != wifiControlState)
						{  //sing!
							playMinorChord(DNote, AUDIO_VELOCITY_MAX*3/4, 200);
						}
						ensControl =C_GLOBAL;
						break;
					}
					wifiControlCountOld = wifiControlCount;

					behSetTvRv(&behRadio, TVcmd, RVcmd); //command the motors
					behSetActive(&behRadio);
				}
			} else {
				// no message this time.  see if you can just run the last command, or if it has timed out
				if (osTaskGetTickCount() > (radioMessageTimePrev + RADIO_MESSAGE_PERSISTANCE)) {
					// the previous message is too old.  clear the behRadio
					behRadio = behInactive;
				} else {
					// use the old behRadio
					// nothing to do, the behRadio is static and will hold it's previous value
				}
			}



			switch(ensControl){
			case C_SERIAL:
				ledsSetPattern(LED_ALL, LED_PATTERN_CLAW, LED_BRIGHTNESS_MED, LED_RATE_FAST);

				if(wifiControlCount >= 0 && wifiControlCount < numRobots && roneID != robotIDs[wifiControlCount] )
				{  //stop the robot if only one robot is being controlled and it isn't this robot
					behSetTvRv(&behRadio,0, 0);

					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				}
				behSetActive(&behOutput);
				behOutput.rv += behRadio.rv;
				behOutput.tv += behRadio.tv;
				//otherwise, obey the command
				//doesn't use flocking
				break;
			case C_LOCAL:
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				behSetActive(&behOutput);
				behOutput.rv += behRadio.rv;
				behOutput.tv += behRadio.tv;
				// doesn't use flocking
				break;
			case C_GLOBAL:

				//stuff for flocking, only used for global control
				behFlock(&behOutput, &nbrList, DEMO_TV_FLOCK);
				behSetActive(&behOutput);
				if (behRadio.active) {
					//ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
					behOutput.rv += behRadio.rv;
					behOutput.tv += behRadio.tv;
				} else {
					//ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);
				}

				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				break;
			}

		}

		cprintf("\n");
		TimePrev = Time;
		motorSetBeh(&behOutput);

		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}



/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}

