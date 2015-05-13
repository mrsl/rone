/*
 * remoteControl.c
 *
 *  Created on: Mar 25, 2013
 *      Author: Sky
 */

#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include "roneos.h"
#include "ronelib.h"

#define ACCEL_IIR_GAIN					65
#define TV_MIN							15
#define REMOTE_CONTROL_MESSAGE_TIMEOUT	200

#define REMOTE_CONTROL_TV_GAIN			40
#define REMOTE_CONTROL_RV_GAIN			200
#define REMOTE_CONTROL_TV_MAX			100
#define REMOTE_CONTROL_RV_MAX			1800

#define RADIO_CONTROL_TEAM_MAX			3
#define RADIO_CONTROL_COMPASS_TV_MIN	20


#define ACCEL_DEAD_ZONE					20
//#define ACCEL_INPUT_SCALE				25
//#define ACCEL_RV_RECV_GAIN				160
//#define ACCEL_TV_RECV_GAIN				150
//#define TV_SPEED						100
#define COMPASS_RV_GAIN					60

//serial host defines
#define REMOTE_CONTROL_SERIAL_HOST_TIMEOUT		500

// what runmode is determines what demomode will be - there will be a switch statement
// that takes in runmode as input and set datamode accordingly
#define RUNMODE_IDLE					0
#define RUNMODE_ACTIVE					1
#define RUNMODE_WAIT					2

#define WAIT_TIME						50000
#define LOOP_TIME						1000000

#define TEAM_RED						0
#define TEAM_GREEN						1
#define TEAM_BLUE						2

#define DEMOMODE_IDLE					50 // this is a default large value for DEMOMODE_IDLE to avoid unnecessary mod

#define DEMOMODE_COUNT					4 // expect Demomode to be consecutive. (Example: use it for mod)

#define DEMOMODE_FOLLOW					0
#define DEMOMODE_FLOCK					1
#define DEMOMODE_CLUSTER				2
#define DEMOMODE_DISPERSE				3

#define JOYSTICK_VAL_UP					127
#define JOYSTICK_VAL_DOWN				-127
#define JOYSTICK_VAL_RIGHT				127
#define JOYSTICK_VAL_LEFT				-127

#define JOYSTICK_MASK_LEFT				0x2
#define JOYSTICK_MASK_RIGHT				0x1
#define JOYSTICK_MASK_UP				0x8
#define JOYSTICK_MASK_DOWN				0x4

#define JOYSTICK_ACTIVE_TIMEOUT			2000
#define JOYSTICK_ACTIVE_DIRECTION_MAG	30

static Joystick joysticks[REMOTE_CONTROL_JOYSTICK_NUM];

static RadioCmd radioCmdRemoteControl;
static SerialCmd serialCmdUI;

static const Joystick joystickClear = {0, 0, 0};

int32 deadzone(int32 val, int32 deadzone) {
	if (val > deadzone) {
		val -= deadzone;
	} else if (val < -deadzone) {
		val += deadzone;
	} else {
		val = 0;
	}
	return val;
}


void remoteControlSendMsg(void) {
	RadioMessage radioMessage;
	uint8 i, j;

//	TVcmd = boundAbs(TVcmd, REMOTE_CONTROL_TV_MAX);
//	RVcmd = boundAbs(RVcmd, REMOTE_CONTROL_RV_MAX);
//	team = bound(team, 0, RADIO_CONTROL_TEAM_MAX);
//	//TODO send int16s instead.  Use pack16() or it's relatives
//	sprintf(radioMsgTX.command.data,"%d,%d,%d",TVcmd, RVcmd, team);
//	//sprintf(radioMsgTX.command.data,"test message%d",count++);
//	radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMsgTX);
//	cprintf("%s\n",radioMsgTX.command.data);
//
//	/*
//	 * Message sending out should look like an array of int8
//	 * |X|Y|B||X|Y|B||X|Y|B|
//	 * \  R  /\  G  /\  B  /
//	 */

	j = 0;
	for (i = 0; i < REMOTE_CONTROL_JOYSTICK_NUM; i++) {
		radioMessage.command.data[j++] = joysticks[i].x;
		radioMessage.command.data[j++] = joysticks[i].y;
		radioMessage.command.data[j++] = joysticks[i].buttons;
	}
	radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessage);
}


void remoteControlSendMsgAccel(uint8 team) {
	static int32 accX = 0;
	static int32 accY = 0;
	int32 accXNew, accYNew, Xcmd, Ycmd;
	Joystick joysticks[REMOTE_CONTROL_JOYSTICK_NUM];

	/* This is math for the complimentary filter. It ignores the gyroscopes for now.
	 * (So not really a complimentary filter, but still filters the accelerometer)
	 */
	//TOOD bound to -127 to 128
	accXNew = deadzone((int32)accelerometerGetValue(ACCELEROMETER_X), ACCEL_DEAD_ZONE);
	accYNew = deadzone((int32)accelerometerGetValue(ACCELEROMETER_Y), ACCEL_DEAD_ZONE);
	//TODO scale these down to int8
	joysticks[0].x = (int8)((accXNew * ACCEL_IIR_GAIN) + (accX * (100 - ACCEL_IIR_GAIN))) / 100;
	joysticks[0].y = (int8)((accYNew * ACCEL_IIR_GAIN) + (accY * (100 - ACCEL_IIR_GAIN))) / 100;

	joysticks[1] = joystickClear;
	joysticks[2] = joystickClear;

	remoteControlSendMsg();
}


void remoteControlUpdateJoysticks(void) {
	RadioMessage radioMessage;
	uint8 joyNum, j;

	if (radioCommandReceive(&radioCmdRemoteControl, &radioMessage, 0) ) {

		// unpack all the joystick data from the radio message
		//TODO Fix, add bounds checking to make sure we aren't corrupting things
		j = 0;
		for (joyNum = 0; joyNum < REMOTE_CONTROL_JOYSTICK_NUM; joyNum++) {
			joysticks[joyNum].x = (int8)radioMessage.command.data[j++];
			joysticks[joyNum].y = (int8)radioMessage.command.data[j++];
			joysticks[joyNum].buttons = radioMessage.command.data[j++];

			// see if this joystick is active
			if((vectorMag(joysticks[joyNum].x, joysticks[joyNum].y) > JOYSTICK_ACTIVE_DIRECTION_MAG) || joysticks[joyNum].buttons) {
				joysticks[joyNum].activeTime = osTaskGetTickCount();
			}
			cprintf("joy%d,%4d,%4d,%4d ",joyNum,joysticks[joyNum].x,joysticks[joyNum].y,joysticks[joyNum].buttons);
		}
		cprintf("\n");
	}
}


Joystick* remoteControlGetJoystick(uint8 joystickNum) {
	if (joystickNum < REMOTE_CONTROL_JOYSTICK_NUM) {
		return &joysticks[joystickNum];
	}
	return NULL;
}


boolean remoteControlJoystickIsActive(uint8 joystickNum, uint32 timeOut) {
	boolean val = FALSE;
	if (joystickNum < REMOTE_CONTROL_JOYSTICK_NUM) {
		//if (((osTaskGetTickCount() - joystickActiveTimes[joystickNum]) < timeOut) && (osTaskGetTickCount() > timeOut)) {
		if ((osTaskGetTickCount() - joysticks[joystickNum].activeTime) < timeOut) {
			val = TRUE;
		}
	}
	return val;
}


Beh* behRemoteControl(Beh* behPtr, uint8 joystickNum) {
	int32 tv, rv;
	static Beh radioBeh;
	static uint32 radioMessageTimePrev;

	/* All Non-leaders get radio signal*/
	if (remoteControlJoystickIsActive(joystickNum, JOYSTICK_ACTIVE_TIMEOUT)) {
		Joystick* joystickPtr = remoteControlGetJoystick(joystickNum);
		radioMessageTimePrev = osTaskGetTickCount();

		tv = -boundAbs((int32)joystickPtr->x * REMOTE_CONTROL_TV_GAIN / 100, REMOTE_CONTROL_TV_MAX);
		rv = -boundAbs((int32)joystickPtr->y * REMOTE_CONTROL_RV_GAIN / 100, REMOTE_CONTROL_RV_MAX);

		behSetTvRv(&radioBeh, tv, rv);
		//cprintf("joy % 6d,% 6d\n", joystickPtr->x, joystickPtr->y); //debugging for the radio on the Secure CRT
	}

	// no message this time.  see if you can just run the last command, or if it has timed out
	if (osTaskGetTickCount() > (radioMessageTimePrev + REMOTE_CONTROL_MESSAGE_TIMEOUT)) {
		// the previous message is too old.  clear the behPtr
		radioBeh = behInactive;
	}
	*behPtr = radioBeh;
	return behPtr;
}


#define NAV_TOWER_LOW_RONE_ID			124
#define NAV_TOWER_HIGH_RONE_ID			125
#define NAV_TOWER_FILTER_GAIN			50

uint32 navTowerTime = 0;

void navTowerUpdateHeading(boolean printNbrs) {
	NbrList nbrListAll;
	Nbr* nbrNavTowerHighPtr;
	Nbr* nbrNavTowerLowPtr;
	int32 heading = 0;

	// look for the nav tower
	nbrListCreate(&nbrListAll);
	nbrNavTowerLowPtr = nbrListGetNbrWithID(&nbrListAll, NAV_TOWER_LOW_RONE_ID);
	nbrNavTowerHighPtr = nbrListGetNbrWithID(&nbrListAll, NAV_TOWER_HIGH_RONE_ID);
	if(nbrNavTowerLowPtr || nbrNavTowerHighPtr) {
		navTowerTime = osTaskGetTickCount();
	}

	if (printNbrs){
		if (nbrNavTowerHighPtr){
			cprintf("(NavTower High) ID: %d, bearing:%d, orientation:%d \n", nbrNavTowerHighPtr->ID, nbrNavTowerHighPtr->bearing, nbrNavTowerHighPtr->orientation);
		} else if (nbrNavTowerLowPtr){
			cprintf("(NavTower Low) ID: %d, bearing:%d, orientation:%d \n", nbrNavTowerLowPtr->ID, nbrNavTowerLowPtr->bearing, nbrNavTowerLowPtr->orientation);
		}
	}

	Pose pose;
	encoderGetPose(&pose);
	if(nbrNavTowerLowPtr) {
		// we can see the low power nav tower virtual robot.  update the robot's heading
		heading = normalizeAngleMilliRad2(nbrGetOrientation(nbrNavTowerLowPtr) + (int32)MILLIRAD_PI - nbrGetBearing(nbrNavTowerLowPtr));
		//pose.theta = heading;
		pose.theta = filterIIRAngle(pose.theta, heading, NAV_TOWER_FILTER_GAIN);
	} else if(nbrNavTowerHighPtr) {
		// we can see the high power nav tower virtual robot.  update the robot's heading
		heading = normalizeAngleMilliRad2(nbrGetOrientation(nbrNavTowerHighPtr) + (int32)MILLIRAD_PI - nbrGetBearing(nbrNavTowerHighPtr));
		//pose.theta = heading;
		pose.theta = filterIIRAngle(pose.theta, heading, NAV_TOWER_FILTER_GAIN);
	} else {
		// no nav towers. Leave the pose alone
	}
	encoderSetPose(&pose);
}


#define REMOTE_CONTROL_COMPASS_TV_GAIN 		1
#define REMOTE_CONTROL_COMPASS_RV_GAIN 		50

Beh* behRemoteControlCompass(Beh* behPtr, Joystick* joystickPtr, uint16 tvMax) {
	int32 Xcmd, Ycmd, tv;
	int32 heading, goalVectorAngle, goalVectorMag, alpha;
	static Beh radioBeh;
	static uint32 radioMessageTimePrev;

	// process the remote control message
	radioMessageTimePrev = osTaskGetTickCount();

	heading = encoderGetHeading();

	 /* Set Xcmd and Ycmd */
	Xcmd = joystickPtr->x;
	Ycmd = joystickPtr->y;

	// compute mag and dir of rc message
	goalVectorAngle = atan2MilliRad(Ycmd, Xcmd);
	goalVectorMag = sqrtInt(Ycmd * Ycmd + Xcmd * Xcmd);

	tv = boundAbs(goalVectorMag * REMOTE_CONTROL_COMPASS_TV_GAIN, tvMax);

	if (tv > RADIO_CONTROL_COMPASS_TV_MIN) {
		alpha = smallestAngleDifference(goalVectorAngle, heading);
		behBearingControllerGain(&radioBeh, alpha, REMOTE_CONTROL_COMPASS_RV_GAIN);
		behSetTv(&radioBeh, tv);
	} else {
		// not enough tv to move the robot.  sit still, but keep the behavior active
		behSetTvRv(&radioBeh, 0, 0);
		//cprintf("hdg=%5d, alpha=%5d, tv=%4d, rv=%4d\n", heading, alpha, radioBeh.tv, radioBeh.rv);
		//cprintf("% 6d,% 6d, %1d",TVcmd,RVcmd,teamPtr); //debugging for the radio on the Secure CRT
	}

	// no message this time.  see if you can just run the last command, or if it has timed out
	if (osTaskGetTickCount() > (radioMessageTimePrev + REMOTE_CONTROL_MESSAGE_TIMEOUT)) {
		// the previous message is too old.  clear the behPtr
		radioBeh = behInactive;
	}
	*behPtr = radioBeh;
	return behPtr;
}


uint8 demoMode[3] = {DEMOMODE_IDLE};
uint8 runMode[3] = {RUNMODE_IDLE};
uint8 buttonVal = 0;


boolean remoteControlIsSerialHost(void) {
	if (((osTaskGetTickCount() - serialCommandGetTimestamp(&serialCmdUI)) < REMOTE_CONTROL_SERIAL_HOST_TIMEOUT) && (osTaskGetTickCount() > REMOTE_CONTROL_SERIAL_HOST_TIMEOUT)) {
		return TRUE;
	} else {
		return FALSE;
	}
}


#define JOYSTICK_UI_POSITION_CENTER		128

int8 joystickAxisConvert(int32 val) {
	int32 valOut = (int32)val - JOYSTICK_UI_POSITION_CENTER;
	valOut = bound(valOut, -JOYSTICK_POSITION_MAX, JOYSTICK_POSITION_MAX);
	return (int8)valOut;
}


void joystickParseSingle(int32 joystickNum, char* joystickCommand) {
	joysticks[joystickNum].x = joystickAxisConvert((int32)atoi_hex8(&joystickCommand[0]));
	joysticks[joystickNum].y = joystickAxisConvert((int32)atoi_hex8(&joystickCommand[2]));
	joysticks[joystickNum].buttons = atoi_hex8(&joystickCommand[4]);
}


#define JOYSTICK_MESSAGE_SIZE	6

/* Joystick values:
 * XXYYBB...
 * XX = 8-bit unsigned value for X.  128 = centered
 * YY = 8-bit unsigned value for Y.  128 = centered
 * BB = 8-bit bit-packed buttons.  bit 0 = r, 1 = g, 2 = b
 */
static void serialCmdUIFunc(char* command) {
	uint8 joyNum;

	// move past the command prefix, skip the 'UI'
	command = serialCommandRemovePrefix(command);

	for (joyNum = 0; joyNum < REMOTE_CONTROL_JOYSTICK_NUM; joyNum++) {
		// parse each message for each joystick
		char* joystickCommandPtr = &command[joyNum * JOYSTICK_MESSAGE_SIZE];

		joystickParseSingle(joyNum, joystickCommandPtr);

		// see if this joystick is active
		if((vectorMag(joysticks[joyNum].x, joysticks[joyNum].y) > JOYSTICK_ACTIVE_DIRECTION_MAG) || joysticks[joyNum].buttons) {
			joysticks[joyNum].activeTime = osTaskGetTickCount();
		}
	}
	remoteControlSendMsg();
}



#define REMOTE_CONTROL_LED_COMMAND_LEN		12
char LEDCommandString[REMOTE_CONTROL_LED_COMMAND_LEN + 2];

void remoteControlLEDSendMsg(void){
	//cprintf("%s",LEDCommandString);
}

const char* joystickLedCmdDefault = "RM00GM00BM00\n";

void remoteControlLEDClear() {
	strcpy(LEDCommandString, joystickLedCmdDefault);
}

void remoteControlLEDSetSingle(uint8 color, uint8 rate, uint8 pattern){
	uint8 colorIdx = 0;

	uint8 c;
	uint8 colorStart = LED_RED;
	uint8 colorEnd = LED_BLUE;

	if (color != LED_ALL) {
		// only update one group
		colorStart = color;
		colorEnd = color;
	}

	for (c = colorStart; c <= colorEnd; c++) {
		colorIdx = c * 4;
		// Set other stuff
		if (pattern == LED_PATTERN_MANUAL) {
			LEDCommandString[colorIdx + 1] = 'M';
		} else if (pattern == LED_PATTERN_ON) {
			LEDCommandString[colorIdx + 1] = 'O';
		} else if (pattern == LED_PATTERN_OFF) {
			LEDCommandString[colorIdx + 1] = 'F';
		} else if (pattern == LED_PATTERN_PULSE) {
			LEDCommandString[colorIdx + 1] = 'P';
		}
		char buf[2];
		sprintf(buf, "%.2x", rate);
		LEDCommandString[colorIdx + 2] = buf[0];
		LEDCommandString[colorIdx + 3] = buf[1];
	}
}


/*
 * @brief Sets the properties of the LED animation if remote control mode is off.
 *
 * Sets the color, patter, brightness, and rate of the LED animation.
 * Should be called to set all LEDs
 * @param color which color group to set
 * @param pattern lights can run in patterns such as circle/all/etc.
 * @param brightness the brightness to set the pattern
 * @param rate the speed at which the pattern cycles
 * @returns void
 */
void remoteControlLedsSetPattern(uint8 color, uint8 pattern, uint8 brightness, uint8 rate) {
	remoteControlLEDClear();
	remoteControlLEDSetSingle(color, rate , pattern);
	remoteControlLEDSendMsg();
}

void remoteControlInit() {
	uint8 joyNum;

	// make a new radio command for remote control message
	radioCommandAddQueue(&radioCmdRemoteControl, "remoteControl", 1);

	// add a serial command to receive remote control messages from the joysticks
	serialCommandAdd(&serialCmdUI, "UI", serialCmdUIFunc);

	// clear all the joysticks
	for (joyNum = 0; joyNum < REMOTE_CONTROL_JOYSTICK_NUM; joyNum++) {
		joysticks[joyNum].x = 0;
		joysticks[joyNum].y = 0;
		joysticks[joyNum].buttons = 0;
	}
}

// This code fragment is for setting the demo mode without driving a robot around

//void remoteControlSendDemoMode(RadioCmd* radioCmdPtr, uint8 demoModeXmit) {
//	RadioMessage message;
//	char* msg = radioCommandGetDataPtr(&message);
//
//	msg[0] = RADIO_MESSAGE_DEMO_MODE;
//	msg[1] = demoModeXmit;
//	radioCommandXmit(radioCmdPtr, ROBOT_ID_ALL, &message);
//}
//


//RadioWrap* parseJoystick(char* msg, uint32 index, Beh* Beh){
//	RadioWrap* rw;
////	char color = msg[index];
//	char joystick = msg[index];
//	char button = msg[++index];
//
//	if(joystick == 0x00 && button == 0x00){
//		rw->BehPtr->active = TRUE; //Check if behInactive then outside and then run demoMode???
//		rw->DemoMode = 0; //IDLE Mode
//	}
//
//	if(button == 0x00){ //DO JOYSTICK MOVEMENTS ON FTL
//		rw->DemoMode = 1;
//		rw->BehPtr->active = TRUE;
//		switch (joystick){
//
//		case 0x01: { //f
//			rw->BehPtr->rv = 50;
//		}
//		case 0x02: { //r
//			rw->BehPtr->rv = 50;
//		}
//		case 0x03: { //fr
//			rw->BehPtr->rv = 25;
//		}
//		case 0x04: { //b
//			rw->BehPtr->rv = 50;
//		}
//		case 0x06: { ////br
//			rw->BehPtr->rv = 25;
//		}
//		case 0x08: { //l
//			rw->BehPtr->rv = 50;
//		}
//		case 0x0C: { //bl
//			rw->BehPtr->rv = 25;
//		}
//		case 0x09: { //fl
//			rw->BehPtr->rv = 25;
//		}
//		}
//	}
//	else if(button == 0x01){
//		//DO JOYSTICK MOVEMENTS ON CLUSTER
//	}
//	else if(button == 0x02 || button == 0x03){
//		//DO JOYSTICK MOVEMENTS ON FLOCKING
//	}
//	else if(button == 0x04 || button == 0x05 || button == 0x06 || button == 0x07){
//		//DO JOYSTICK MOVEMENTS ON DISPERSE
//	}
//
//}
//
//RadioWrap* parseRadio(char* msg, Beh* behPtr, int groupID){
//	uint32 curIndex = 0;
//	RadioWrap* rw;
//
//	if (msg[curIndex] == 0x6A){ //check that its a joystick message
//		curIndex++;
//		if(groupID == 0){ //r
//			curIndex = 1;
//			rw = parseJoystick(msg, curIndex, behPtr);
//		}
//		if(groupID == 1){ //g
//			curIndex = 4;
//			rw = parseJoystick(msg, curIndex, behPtr);
//		}
//		if(groupID == 2) {//b
//			curIndex = 7;
//			rw = parseJoystick(msg, curIndex, behPtr);
//		}
//	}
//	return rw;
//}
//
//
//
//RadioWrap* behRadioControlNew(Beh* behPtr, uint8* demoModePtr, int groupID) {
//	char message[RADIO_MESSAGE_LENGTH_RAW] = {0};
//	uint32 message_size;
//	uint32 message_link_quality;
//	RadioWrap* rw;
//
//	*behPtr = behInactive;
//	if (radioGetMessageBlocking(message, &message_size, &message_link_quality)) {
//		rw = parseRadio(message, behPtr, groupID);
//	}
//	return rw;
//}
