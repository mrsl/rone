/*
/*
 * test_bumpSkirtWallAngle.c
 *
 *     Created on: 07/30/2012
 *         Author: Mathew Jellins, Zack
 *        Summary: This code calculates a tree based on the weight.
 *        Weight is determined by how close walls are, and a node can be excluded if a wall is to close.
 *        A path can be calculated to the source from any node.
 */
#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD			300
#define RADIO_MESSAGE_PERSISTANCE		200
#define BEHAVIOR_TASK_PERIOD			50
#define FLOCK_RV_GAIN_MOVEOBJ			150
#define FLOCK_RV_GAIN_GRIPPED			50
#define REALESE_WAIT_TIME				2000
#define LOST_OBJECT_WAIT				5000
#define CONV_ANGL						900

#define ACCEL_DEAD_ZONE					5
#define ACCEL_IIR_GAIN					50
#define TV_MIN							15

#define MODE_IDLE		 0
#define SEEKER_SEL		 1
#define SEEKER			 2
#define GRIPPED			 3
#define CONV_SEL		 4
#define CONVERGE		 5
#define MOVE_OBJ		 6
#define REMOTE			 8
#define RELEASE			 9

#define SEEKING			1
#define CONVERGING		2
#define GRIPPING		3
#define LEAVING			4


#define CNTCLK			1
#define CLKWISE			2
#define REST			0
#define ATTEMPTING		3

RadioMessage radioMessageTX;
RadioMessage radioMessageRX;
RadioCmd radioCmdRemoteControl;
char* name = "RCwifi";

void behaviorTask(void* parameters) {
	//rprintfSetSleepTime(500);

	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 releaseTime;
	uint8 navigationMode = MODE_IDLE;
	Beh behOutput, behRadio, behConv, behGrip, behLeave;
	uint8 i;
	boolean printNow;
	uint32 neighborRound = 0;
	NbrList nbrList;
	NbrList flockList;
	Nbr* nbrPtr;
	Nbr* bestGripped;
	Nbr* lowestPtr;
	uint8 lowestID = 0;
	boolean lostObject = 0;
	boolean releaseDone = 0;
	boolean remoteSkip = 0;
	uint8 orbitedArray[20];
	uint8 orbitSize=0;


	BroadcastMessage broadcastMessage;
	broadcastMsgCreate(&broadcastMessage, 20);

	systemPrintStartup();
	systemPrintMemUsage();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	radioCommandSetSubnet(1);

	NbrData msgNbrType;

	nbrDataCreate(&msgNbrType, "type", 3, 0);

	//uint16 IRXmitPower = IR_COMMS_POWER_MAX/4;

	///Radio Init
	uint32 radioMessageTimePrev = 0;
	int TVcmd, RVcmd = 0;
	int comRel = 0;
	int comGrip = 0;
	int comRot = 0;
	int32 accX = 0;
	int32 accY = 0;
	radioCommandAddQueue(&radioCmdRemoteControl,name, 1);

	//GRIPPER INIT
	gripperBoardInit();
	uint8 gripPos = -1;
	gripperCalibratServo();
	for (;;) {
		if (rprintfIsHost()) {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
			continue;
		} else if(!gripperServoCalibratFinish() && !remoteSkip){
			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
			lastWakeTime = osTaskGetTickCount();
			if (buttonsGet(BUTTON_GREEN)) {
				navigationMode = REMOTE;			//For single remote bot
				remoteSkip = 1;
			}
		}
		else{
			/*** INIT STUFF ***/
			behOutput = behInactive;
			behRadio = behInactive;
			behConv = behInactive;
			behGrip = behInactive;
			behLeave = behInactive;

			neighborsGetMutex();
			printNow = neighborsNewRoundCheck(&neighborRound);

			//irCommsSetXmitPower(IRXmitPower);

			nbrListCreate(&nbrList);

			broadcastMsgUpdate(&broadcastMessage, &nbrList);

			/*** READ BUTTONS ***/
			if(navigationMode != REMOTE){
				if (buttonsGet(BUTTON_RED)) {
					navigationMode = SEEKER_SEL;		//Most Robots
				}else if (buttonsGet(BUTTON_GREEN)) {
					navigationMode = REMOTE;			//For single remote bot
				} else if (buttonsGet(BUTTON_BLUE)) {
					gripPos = REST;						//If in mode idle and want to restart gripping
				}
			}

			/** STATES MACHINE **/
			switch (navigationMode) {
			case MODE_IDLE: {
				//Start Menu
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				nbrDataSet(&msgNbrType, 0);
				lostObject = 0;
				releaseDone = 0;
				break;
			}
			case SEEKER_SEL: {
				//Choose lowest robot amongst themselves to be controlled by remote
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, SEEKING);
				break;
			}
			case SEEKER: {
				//Robot to be controled by remote
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, SEEKING);
				break;
			}
			case GRIPPED: {
				//Robot to be controled by remote
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, GRIPPING);
				break;
			}
			case CONV_SEL: {
				//After robot is found, robots converge one by one
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, CONVERGING);
				break;
			}
			case CONVERGE: {
				//After robot is found, robots converge one by one
				ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, CONVERGING);
				break;
			}
			case REMOTE: {
				//Start Menu
				ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
				nbrDataSet(&msgNbrType, 0);
				break;
			}
			case MOVE_OBJ: {
				//Robot to be controled by remote
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				nbrDataSet(&msgNbrType, GRIPPING);
				lostObject = 1;
				break;
			}
			case RELEASE: {
				//Robot release object and leave
				ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
				nbrDataSet(&msgNbrType, LEAVING);
				break;
			}
			}

			//While in mode idle attempt to grip, blue button restarts it
			if(navigationMode == MODE_IDLE){
				behOutput = behInactive;
				if(!gripperBoardGetGripped()){
					if(gripPos != ATTEMPTING){
						gripperGripUntilGripped();
						gripPos = ATTEMPTING;
					}
				}else{
					gripPos = CLKWISE;
					if(gripperBoardGetServo() > 100){
						gripPos = CNTCLK;
					}
				}
			}

			/**Remote Control **/
			if(navigationMode == REMOTE){
				behOutput = behInactive;

					//convrols tv and rv using accelarmotor sensors
					accX = (((int32)accelerometerGetValue(ACCELEROMETER_X) * ACCEL_IIR_GAIN) + (accX * (100 - ACCEL_IIR_GAIN))) / 100;
					accY = (((int32)accelerometerGetValue(ACCELEROMETER_Y) * ACCEL_IIR_GAIN) + (accY * (100 - ACCEL_IIR_GAIN))) / 100;
					accX = deadzone(accX, ACCEL_DEAD_ZONE);
					accY = deadzone(accY, ACCEL_DEAD_ZONE);

					int sc = 5;			//How fast we want the TV to be
					TVcmd = -accX/sc;
					RVcmd = -accY * 2;
					if (abs(TVcmd) < TV_MIN) {			//Minimum TV
						if(TVcmd > 0) {
							TVcmd = TV_MIN;
						} else {
							TVcmd = -TV_MIN;
						}
					}


				comGrip = 0;			//Tells robot to grip, moves SEEKER bot to GRIPPING
				comRel = 0;				//Tells robot to release object, moves all robots from MOVE_OBJ to RELEASE mode
				comRot = 0;				//Sends comand to rotate based on TVcmd as rotational speed/direction
				if (buttonsGet(BUTTON_BLUE)) {
					comGrip = 1;
				}
				if (buttonsGet(BUTTON_RED)) {
					comRel = 1;
				}
				if (buttonsGet(BUTTON_GREEN)) {
					comRot = 1;
				}

				//Sends commands over radio
				sprintf(radioMessageTX.command.data,"%d,%d,%d,%d,%d",TVcmd, RVcmd, comGrip, comRel, comRot);
				radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessageTX);
				//cprintf("%d,%d,%d,%d\n",TVcmd, RVcmd, gripIdx, gripDir);
			}


			//Decides whether it is a seeker robot, if it is follow controls of Remote until you told to grip
			if(navigationMode == SEEKER_SEL || navigationMode == SEEKER){
				/** Seeker Election **/
				uint8 currID = 0;
				uint8 nbrType;
				//For all parents in list
				for (i = 0; i < nbrList.size; i++){
					nbrPtr = nbrList.nbrs[i];
					currID = nbrGetID(nbrPtr);
					nbrType = nbrDataGetNbr(&msgNbrType, nbrPtr);
					//Look at Seeker Robots only
					if(nbrType == SEEKING){
						//For Lowest ID
						if(lowestID == 0){
							lowestID = currID;
							lowestPtr = nbrPtr;
						} else if(currID < lowestID){
							lowestID = currID;
							lowestPtr = nbrPtr;
						}
					}
					//If you see a converging or gripping robot enter Conv Sel mode
					if(nbrType == CONVERGING || nbrType == GRIPPING){
						navigationMode = CONV_SEL;
					}
				}
				//If you are the lowest robot in seeker mode, become the seeker
				if(navigationMode != CONV_SEL){
					if(roneID < lowestID || lowestID == 0){
						navigationMode = SEEKER;
					} else{
						navigationMode = SEEKER_SEL;
					}
				}

				//Determing commands from the radio
				if( radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX,0) ) {
					//Grabing stream and paresing
					char* RXmsg = radioCommandGetDataPtr(&radioMessageRX);
					radioMessageTimePrev = osTaskGetTickCount();
					sscanf(RXmsg,"%d,%d,%d,%d,%d", &TVcmd, &RVcmd, &comGrip, &comRel,&comRot); //parse the speed and turning rate

					//If command to grip, go into either GRIPPED mode or Convergence Select mode
					if(comGrip){
						if(navigationMode == SEEKER){
							navigationMode = GRIPPED;

						}
						if(navigationMode == SEEKER_SEL){
							navigationMode = CONV_SEL;
						}
					}
					//Seeker taking commands from the remote
					if(navigationMode == SEEKER){
						behSetTvRv(&behRadio, TVcmd, RVcmd); //command the motors
						behSetActive(&behRadio);
					}
				} else {
					// no message this time.  see if you can just run the last command, or if it has timed out
					if (osTaskGetTickCount() > (radioMessageTimePrev + RADIO_MESSAGE_PERSISTANCE)) {
						// the previous message is too old.  clear the behRadio
						behRadio = behInactive;
					}
				}
				//Set Behavior radio as behavior if seeker
				if(navigationMode == SEEKER){
					behOutput = behRadio;
				}
				//If grip is lost reattempt
				//gripperCheckGripped();
				if(!gripperBoardGetGripped()){
					if(gripPos != ATTEMPTING){
						gripperGripUntilGripped();
						gripPos = ATTEMPTING;
					}
				}else{
					navigationMode = GRIPPED;
					gripPos = CLKWISE;
					if(gripperBoardGetServo() > 100){
						gripPos = CNTCLK;
					}
				}
			}

			//If object has been discovered, converge on object, one at a time
			if(navigationMode == CONV_SEL || navigationMode == CONVERGE){
				uint8 currID = 0;
				uint8 lowestconID = 0;
				uint8 nbrType;
				int32 desGripp;
				int32 newRv;
				bestGripped = NULL;
				//For all parents in list
				for (i = 0; i < nbrList.size; i++){
					nbrPtr = nbrList.nbrs[i];
					currID = nbrGetID(nbrPtr);
					nbrType = nbrDataGetNbr(&msgNbrType, nbrPtr);
					//Look at only other converging robots
					if(nbrType == CONVERGING){
						//determine lowest ID
						if(lowestconID == 0){
							lowestconID = currID;
						} else if(currID < lowestconID){
							lowestconID = currID;
							if(lowestconID < lowestID){
								lowestID  = lowestconID;
								lowestPtr = nbrPtr;
							}
						}
						//If you see robots leaving, and you are still converging, assume that object is full and leave
						if(nbrType == LEAVING){
							navigationMode = RELEASE;
							if(gripperBoardGetServo()>100){
								 gripperGripClockwise();
							 }else{
								 gripperGripCounterClockwise();
							 }
							releaseTime = osTaskGetTickCount();
						}
					}
					//Determine gripped NBR with highest ID
					if(nbrType == GRIPPING ){
						if(bestGripped == NULL){
							bestGripped = nbrPtr;
						} else if(currID > nbrGetID(bestGripped)){
							bestGripped = nbrPtr;
						}
					}
				}
				//If lowest ID seen, go into convergence mode and begin attempting to grip
				if(roneID < lowestconID || lowestconID == 0){
					navigationMode = CONVERGE;
					if(gripPos != ATTEMPTING){
						gripperGripUntilGripped();
						gripPos = ATTEMPTING;
					}
				} else{
					navigationMode = CONV_SEL;
				}

				/**Convergence Motion **/
				//Orbit around robot with highest ID
				if(navigationMode == CONVERGE){
					//If I have a Gripped robot to orbit around
					if(bestGripped != NULL){
						//Look if this robot has been orbited before
						boolean insOrbitList = 1;
						for(i=0; i<orbitSize; i++){
							if(nbrGetID(bestGripped) == orbitedArray[i]){
								insOrbitList = 0;
							}
						}
						//List of robots orbited
						if(insOrbitList){
							orbitedArray[orbitSize] =nbrGetID(bestGripped);
							orbitSize++;
						}
						//If I have orbited several robots and the lowest robot i have seen is to my front right, there is no more space, Enter NO_ROOM
						//TODO: Put in for better object
						/*if((orbitSize > 2) && (nbrGetBearing(lowestPtr) < 700) && (nbrGetBearing(lowestPtr) < 0)){
							navigationMode = RELEASE;
							if(gripperBoardGetServo()>100){
								 gripperGripClockwise();
							 }else{
								 gripperGripCounterClockwise();
							 }
							releaseTime = osTaskGetTickCount();
						}*/

						//Head .8 rads to the right of the chosen robot
						desGripp = nbrGetBearing(bestGripped) - CONV_ANGL;
						if(abs(desGripp) < 0){
							behSetTvRv(&behConv, 150, 0);
						} else{
							if(desGripp < 0){
								newRv = desGripp/ 1.5;
								behSetTvRv(&behConv, 150, newRv);
							} else{
								newRv = desGripp/ 1.5;
								behSetTvRv(&behConv, 150, newRv);
							}
						}
					} else{
						//If no gripp robot found, just head straight
						behSetTvRv(&behConv, 125, 0);
					}

					//If you have gripped on something, enter gripped mood
					behOutput = behConv;

					if(printNow){
						//cprintf("BUMP %d A X %d A Y %d A Z %d G X %d G Y %d G Z %d\n", bumpSensorsGetBearing(),
						//		accelerometerGetValue(ACCELEROMETER_X), accelerometerGetValue(ACCELEROMETER_Y), accelerometerGetValue(ACCELEROMETER_Z),
						//		gyroGetValue(GYRO_X_AXIS), gyroGetValue(GYRO_Y_AXIS), gyroGetValue(GYRO_Z_AXIS));
					}
				}
				//If grip is lost reattempt
					//gripperCheckGripped();
					if(!gripperBoardGetGripped()){
						if(gripPos != ATTEMPTING){
							gripperGripUntilGripped();
							gripPos = ATTEMPTING;
						}
					}else{
						navigationMode = GRIPPED;
						gripPos = CLKWISE;
						if(gripperBoardGetServo() > 100){
							gripPos = CNTCLK;
						}
					}

			}

			/**Gripped Flocking & Object move**/
			//Continous Flocking and movement control when it enters object move
			if(navigationMode == GRIPPED || navigationMode == MOVE_OBJ){
				uint8 i;
				flockList.size = 0;
				uint8 objTV = 0;
				Nbr* lowestNbr = NULL;
				//If lost object and refound enter Move_Obj
				if(lostObject && navigationMode == GRIPPED){
					navigationMode = MOVE_OBJ;
				}

				//For all robots
				for (i = 0; i < nbrList.size; i++) {
					nbrPtr = nbrList.nbrs[i];
					//Look at gripping bots
					if (nbrDataGetNbr(&msgNbrType,nbrPtr) == GRIPPING) {
						//create a list of robots to flock to
						flockList.nbrs[flockList.size] = nbrPtr;
						flockList.size = flockList.size + 1;
						cprintf("Flock List %d ID %D \n",flockList.size, nbrGetID(nbrPtr));
						//Find robot with next highest ID
						//Find robot with lowest ID
						if(nbrGetID(nbrPtr) < roneID && lowestNbr == NULL){
							lowestNbr = nbrPtr;
						}else if(nbrGetID(nbrPtr) < nbrGetID(lowestNbr) && nbrGetID(lowestNbr) < roneID){
							lowestNbr = nbrPtr;
						}
					}
					//If you see robots leaving, enter move object
					if(nbrDataGetNbr(&msgNbrType,nbrPtr) == LEAVING){
						navigationMode = MOVE_OBJ;
					}
				}

				//Check radio Commands
				if( radioCommandReceive(&radioCmdRemoteControl, &radioMessageRX,0) ) {
					char* RXmsg = radioCommandGetDataPtr(&radioMessageRX);
					// motor command
					radioMessageTimePrev = osTaskGetTickCount();
					sscanf(RXmsg,"%d,%d,%d,%d,%d", &TVcmd, &RVcmd, &comGrip, &comRel,&comRot); //parse the speed and turning rate

					//move gripped robots to MOVE_OBJ
					if(comRot && navigationMode == GRIPPED){
						navigationMode = MOVE_OBJ;
					}
					//Leave object
					if(comRel && navigationMode == MOVE_OBJ){
						if(gripperBoardGetServo()>100){
							 gripperGripClockwise();
						 }else{
							 gripperGripCounterClockwise();
						 }
						navigationMode = RELEASE;
						gripPos = -1;
						releaseTime = osTaskGetTickCount();
					}

				}
				//Use list to set flock motion
				if(navigationMode == GRIPPED){
					//behFlock_gain(&behGrip, &flockList, objTV, FLOCK_RV_GAIN_GRIPPED);
					behSetTvRv(&behGrip, 0, 0);
				}
				//Adjust flocking motion with commands from radio
				if(navigationMode == MOVE_OBJ){
					behFlock_gain(&behGrip, &flockList, objTV, FLOCK_RV_GAIN_MOVEOBJ);
					behGrip.rv  = behGrip.rv + RVcmd;
					behGrip.tv = TVcmd;
				}

				//Enter rotate mode - simply pivots around lowest ID
				/*
                if(comRot){
                	int32 newRv;
					int32 desGripp = 0;
					if(lowestNbr != NULL){
						desGripp = nbrGetBearing(lowestNbr) - 1571;
					}
					if(abs(desGripp) < 200){
						behSetTvRv(&behGrip, TVcmd, 0);
					} else{
						if(desGripp < 0){
							newRv = desGripp/ 1.5;
							behSetTvRv(&behGrip, TVcmd, newRv);
						} else{
							newRv = desGripp/ 1.5;
							behSetTvRv(&behGrip, TVcmd, newRv);
						}
					}
                }*/

				//If you lose grip begin to grip again
				//gripperCheckGripped();
				if(!gripperBoardGetGripped()){
					if(gripPos != ATTEMPTING){
						gripperGripUntilGripped();
						gripPos = ATTEMPTING;
					}
					//If You have lost your grip for X milliseconds begin orbiting lowestNBR
					if((releaseTime + LOST_OBJECT_WAIT) < osTaskGetTickCount() && navigationMode == MOVE_OBJ){
						navigationMode = CONVERGE;
					}

				}else{
					releaseTime = osTaskGetTickCount();
					gripPos = CLKWISE;
					if(gripperBoardGetServo() > 100){
						gripPos = CNTCLK;
					}
				}
				behOutput = behGrip;
			}


			//Told to ungrip and backup for several seconds before entering MODE_IDLE
			if(navigationMode == RELEASE){
				if((releaseTime + REALESE_WAIT_TIME)>osTaskGetTickCount()){
					behSetTvRv(&behLeave, 0, 0);
					if(releaseDone){
						behSetTvRv(&behLeave, -100, 0);
					}
				}else{
					releaseTime = osTaskGetTickCount();
					if(releaseDone){
						navigationMode = MODE_IDLE;
					}
					releaseDone = 1;
				}
				behOutput = behLeave;
			}

			/*** FINAL STUFF ***/
			motorSetBeh(&behOutput);
			neighborsPutMutex();

			// delay until the next behavior period
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
			lastWakeTime = osTaskGetTickCount();
		}//end not host
	} // end for loop
} //end behavior function


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


