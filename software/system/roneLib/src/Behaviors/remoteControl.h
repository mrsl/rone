/**
 * @file remoteControl.h
 * @brief Functions for controlling a robot with a joystick.
 * @details
 * \internal
 * Functions for MSI demo
 * \endinternal
 *
 * @since Mar 29, 2013
 * @author MRSL
 *
 *\internal
 * TODO what is Sky's lastname?
 *\endinternal
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#define JOYSTICK_BUTTON_RED				1
#define JOYSTICK_BUTTON_GREEN			2
#define JOYSTICK_BUTTON_BLUE			4

#define JOYSTICK_POSITION_MAX			127
#define REMOTE_CONTROL_JOYSTICK_NUM		3

/**
 * @brief Remote Controller Struct
 */
typedef struct Joystick{
	int8 x;				/**<x-axis control*/
	int8 y;				/**<y-axis control*/
	uint8 buttons;		/**<button bit vector*/
	uint32 activeTime;	/**<time joystick was last active*/
} Joystick;

/**
 * @brief Initiates robot controller
 * @returns void
 */
void remoteControlInit();

//TODO: delete from h file?
//void remoteControlSendMsg(int32 TVcmd, int32 RVcmd, uint8 team);

/**
 * @brief Gets acceleramtor value across x and y axis
 * @param team not used
 * @returns void
 */
void remoteControlSendMsgAccel(uint8 team);


/**
 * @brief True if this was called less then 500 ticks ago and ticks is greater then 500
 * @returns void
*/
boolean remoteControlIsSerialHost(void);

Beh* behRemoteControl(Beh* behPtr, uint8 joystickNum);

Beh* behRemoteControlCompass(Beh* behPtr, Joystick* joystickPtr, uint16 Tv, Nbr* nbrNavTowerLowPtr, Nbr* nbrNavTowerHighPtr);

//TODO move this to a thread or callback
void remoteControlUpdateJoysticks(void);

Joystick* remoteControlGetJoystick(uint8 joystickNum);

boolean remoteControlJoystickIsActive(uint8 joystickNum, uint32 timeOut);

int32 deadzone(int32 val, int32 deadzone);
void remoteControlAccelRemote(RadioCmd* radioCmdPtr);
//void remoteControlSendMessage(RadioCmd* radioCmdPtr, int32 tv, int32 rv);
void remoteControlSendDemoMode(RadioCmd* radioCmdPtr, uint8 demoModeXmit);
Beh* behRadioControl(RadioCmd* radioCmdPtr, Beh* behPtr, uint8* demoModePtr);

/**
 * @ send led command to joystick
 */
void remoteControlLEDSendMsg();

void remoteControlSendMsg(void);

void remoteControlLEDClear();

void remoteControlLEDSetSingle(uint8 LEDcolor, uint8 LEDFreq, uint8 LEDmode);

void remoteControlLedsSetPattern(uint8 color, uint8 pattern, uint8 brightness, uint8 rate);


/**
 * @ Set led mode color freq for the serial command
 */
//void serialCmdLEDSet(uint8 LEDcolor, uint8 LEDFreq, uint8 LEDmode, char* cmdLine);
//
//void remoteControlLEDSendMsg(char ledCmd []);

void remoteControlUpdateLEDmsg(char* LEDmsg);

#endif /* REMOTECONTROL_H_ */
