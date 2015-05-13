/**
 * @file robotgame.c
 * @for giving the robots their health
 * @author Sky Davies
 *
 * Current Status: throw up, and mostly the stuff from demo_RCusingAccelerometer.c
 * Working on: 2 teams for radio.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "roneos.h"
#include "ronelib.h"

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50
#define NEIGHBOR_PERIOD					250

// flocking
#define DEMO_TV_FLOCK					0
#define BUTTON_COUNT_CHANGE				20
#define MSI_DEMO_HOPS_MAX				12


#define SNAKE_NAV_TOWER_TIMEOUT			1000
#define MOTION_TV						120
#define MOTION_TV_FLOCK					(MOTION_TV*2/3)
#define MOTION_TV_ORBIT_CENTER			55
#define CAPTAIN_LED_COUNTER_TIME		12

#define FTL_RANGE						300
#define FOLLOW_LEADER_AVOID_RANGE		180

#define BEH_CLUSTER_RANGE				300



#define MODE_IDLE						0
#define MODE_FOLLOW						1
#define MODE_FLOCK						2
#define MODE_CLUSTER					3

#define FLOCK_CLUSTER_THRESHOLD			1

#define JOYSTICK_NUM_MSI				0

#define MUSEUM_RADIO_COMMAND_BEH_IDX	0
#define TEAM_LEADER_ON_TIME				4
#define TEAM_LEADER_TOTAL_TIME			25

#define MS_SECOND						1000
#define MS_MINUTE						(60 * MS_SECOND)

// time for a behavior fo "settle".  After this time, the user can select another behavior
#define BEHAVIOR_READY_TIME 			(10 * MS_SECOND)

// time for a behavior to time out.  After this time, the robots become idle
#define BEHAVIOR_IDLE_TIME				(5 * MS_MINUTE)

#define JOYSTICK_TIMEOUT_TEAM			(5 * MS_MINUTE)
//#define JOYSTICK_TIMEOUT_BEH			20000


/****** Team Broadcast messages *******/
BroadcastMessage broadcastMsg;
NbrData nbrDataMode;

uint32 behaviorChangeTime = 0;
uint8 currentLED = LED_RED;


/******** user code ********/

#define TEAM_LEADER_ON_TIME			4
#define TEAM_LEADER_TOTAL_TIME		25


Beh* demoFlock(Beh* behOutputPtr, Beh* behRadioPtr, NbrList* nbrListPtr) {
	if ((nbrListGetSize(nbrListPtr) > FLOCK_CLUSTER_THRESHOLD) ||
			(broadcastMsgIsSource(&broadcastMsg))){
		// you are the source, or a minion surrounded by member of your team.  flock.
		behFlock(behOutputPtr, nbrListPtr, MOTION_TV_FLOCK);
		behOutputPtr->rv += behRadioPtr->rv;
		behOutputPtr->tv = MOTION_TV_FLOCK;
		behSetActive(behOutputPtr);
	} else {
		behClusterBroadcast(behOutputPtr, nbrListPtr, MOTION_TV, &broadcastMsg);
	}
	return behOutputPtr;
}


Beh* orbitCentroid(Beh* behOutputPtr, Joystick* joystickPtr, NbrList* nbrListPtr) {
	// compute the centroid


	// mix in the remote control
	// compute the dot product between the joystick direction and the current heading
	int32 heading = encoderGetHeading();
	int32 headingJoystick = atan2MilliRad(joystickPtr->y, joystickPtr->x);
	int32 headingDiff = smallestAngleDifference(headingJoystick, heading);
	int32 tvGain = cosMilliRad(headingDiff) * 2; //TODO use some kind of joystick gain instead of 2
	//TODO: remember to divide by MILLIRAD_TRIG_SCALER when you use this

}


void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRound;
	boolean printNbrs;

	Beh behOutput, behIRObstacle, behBump, behCharge, behRadio;
	NbrList nbrList;
	NbrList nbrListAll;
	Nbr* nbrPtr;
	uint32 printMem = 0;

	uint32 captainLEDCounter = 0;

	neighborsInit(NEIGHBOR_PERIOD);
	broadcastMsgCreate(&broadcastMsg, MSI_DEMO_HOPS_MAX);
	nbrDataCreate(&nbrDataMode, "mode", 4, MODE_FOLLOW);

	irCommsSetXmitPower(IR_COMMS_POWER_MAX * 65 /100);

	remoteControlInit();
	radioCommandSetSubnet(2);

	ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
	remoteControlLedsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);

	while (TRUE) {
		/* Initialize the output behavior to inactive */
		behOutput = behInactive;
		behRadio = behInactive;

		neighborsGetMutex();
		printNbrs = neighborsNewRoundCheck(&neighborRound);
		remoteControlUpdateJoysticks();
		navTowerUpdateHeading(printNbrs);

		// only populate nbr list with robots
		nbrListCreate(&nbrListAll);
		nbrListGetRobots(&nbrList, &nbrListAll);



//		if (printNbrs){
//			// print the battery voltages
//			char num1[10],num2[10];
//			sprintf(num1,"%1.2f", systemBatteryVoltageGet());
//			sprintf(num2,"%1.2f", systemUSBVoltageGet());
//			cprintf("vbat=%s vusb=%s charge=%d fast=%d\n", num1, num2, systemBatteryChargingGet(), systemBatteryFastChargingGet());
//		}

		// print the stack usage for debugging
//		uint32 timeTemp = osTaskGetTickCount() / 2000;
//		if(printMem != timeTemp) {
//			printMem = timeTemp;
//			systemPrintMemUsage();
//		}


		if (!remoteControlJoystickIsActive(JOYSTICK_NUM_MSI, BEHAVIOR_IDLE_TIME)) {
			// no joystick activity for a while.  go idle and flash the lights.
			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			remoteControlLedsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
		} else if (osTaskGetTickCount() - behaviorChangeTime > BEHAVIOR_READY_TIME) {
			// behavior is settled.  show a steady light
			ledsSetPattern(currentLED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			remoteControlLedsSetPattern(currentLED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);

			//  look for a new behavior button press
			Joystick* joystickPtr = remoteControlGetJoystick(JOYSTICK_NUM_MSI);
			if(joystickPtr->buttons & JOYSTICK_BUTTON_RED) {
				if (currentLED != LED_RED) {
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					remoteControlLedsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					behaviorChangeTime = osTaskGetTickCount();
				}
				currentLED = LED_RED;
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_GREEN) {
				if (currentLED != LED_GREEN) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					remoteControlLedsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					behaviorChangeTime = osTaskGetTickCount();
				}
				currentLED = LED_GREEN;
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BLUE) {
				if (currentLED != LED_BLUE) {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					remoteControlLedsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					behaviorChangeTime = osTaskGetTickCount();
				}
				currentLED = LED_BLUE;
			}
		}

		if(remoteControlIsSerialHost()) {
			neighborsXmitEnable(FALSE);
			//radioMessage.command.data[MUSEUM_RADIO_COMMAND_BEH_IDX] = currentLED;
			//radioCommandXmit(&radioCmdBehavior, ROBOT_ID_ALL, &radioMessage);
			behSetTvRv(&behOutput, 0, 0);
		} else {
			neighborsXmitEnable(TRUE);
			if(printNbrs) {
				nbrListPrint(&nbrListAll, "nbrs");
				nbrListPrint(&nbrList, "robots");
			}

			// update the broadcast messages to look for the team leaders
			if (printNbrs) {
				broadcastMsgUpdateLeaderElection(&broadcastMsg, &nbrList);
				broadcastMsgUpdateNbrData(&broadcastMsg, &nbrDataMode);
			}

			// see if you are your team leader
			boolean teamLeader = FALSE;
			Joystick* joystickPtr = remoteControlGetJoystick(JOYSTICK_NUM_MSI);
			if (broadcastMsgIsSource(&broadcastMsg)) {
				// you are the team leader
				teamLeader = TRUE;
				if(printNbrs) cprintf("leader of swarm\n");
			}

			// read the joystick for your team.  We read into the behRadio so we can
			// use the joystick for flocking
			if (printNbrs) cprintf("joy %d,%d\n", joystickPtr->x, joystickPtr->y);

			behRemoteControlCompass(&behRadio, joystickPtr, MOTION_TV);
			if (printNbrs) cprintf("beh %d,%d\n", behOutput.tv, behOutput.rv);

			// Code for the minion rbots
			uint8 mode;
			if(teamLeader) {
				// team leader
				if (remoteControlJoystickIsActive(JOYSTICK_NUM_MSI, JOYSTICK_TIMEOUT_TEAM) && joystickPtr) {
					if(joystickPtr->buttons & JOYSTICK_BUTTON_RED) {
						nbrDataSet(&nbrDataMode, MODE_FOLLOW);
					} else if(joystickPtr->buttons & JOYSTICK_BUTTON_GREEN) {
						nbrDataSet(&nbrDataMode, MODE_FLOCK);
					} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BLUE) {
						nbrDataSet(&nbrDataMode, MODE_CLUSTER);
					}
					behOutput = behRadio;
				}

//				if (remoteControlJoystickIsActive(team, JOYSTICK_TIMEOUT_TEAM)) {
//					if (radioCommandReceive(&radioCmdBehavior, &radioMessage, 0) ) {
//						nbrDataSet(&nbrDataMode[team], radioMessage.command.data[MUSEUM_RADIO_COMMAND_BEH_IDX]);
//					}
//					behOutput = behRadio;
//				}

				mode = nbrDataGet(&nbrDataMode);
				switch (mode) {
				case MODE_FOLLOW: {
					// avoid neighbors who are in front of you
					nbrPtr = nbrListGetClosestNbrToBearing(&nbrList, 0);
					if (nbrPtr &&
						(abs(nbrGetBearing(nbrPtr)) < MILLIRAD_DEG_45) &&
						(nbrGetRange(nbrPtr) < FOLLOW_LEADER_AVOID_RANGE)) {
						// move away from other robots - aka don't drive into your line
						behMoveFromNbr(&behOutput, nbrPtr, MOTION_TV);
					} else if ((behGetTv(&behRadio) != 0) || (behGetRv(&behRadio) != 0)) {
						// we have active user input.  follow the joystick
						behOutput = behRadio;
					} else {
						// no nearby robots, no joystick input.  Just move forward
						behMoveForward(&behOutput, MOTION_TV);
					}
					break;
				}
				case MODE_FLOCK: {
					if(printNbrs) {
						cprintf("Team Size: %d\n", nbrList.size);
					}
					demoFlock(&behOutput, &behRadio, &nbrList);
					break;
				}
				case MODE_CLUSTER: {
					if(behIsActive(&behRadio)) {
						behRemoteControlCompass(&behOutput, joystickPtr, MOTION_TV_ORBIT_CENTER);
					}
					break;
				}
				case MODE_IDLE:{
					break;
				}
				default:
					break;
				}
				if(printNbrs) {
					cprintf("leader. mode = %d\n", mode);
				}


				// Show team captain colors (alternate between team colors and all the LEDs)
				if (captainLEDCounter == 0) {
					captainLEDCounter = TEAM_LEADER_TOTAL_TIME;
				}
				if (captainLEDCounter > 0) {
					captainLEDCounter--;
				}

//				if ((captainLEDCounter < TEAM_LEADER_ON_TIME) && (mode != MODE_FLOCK)) {
//					if (hostFlag == FALSE)	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//				} else {
//					if (hostFlag == FALSE)  ledsSetPattern(mode-1, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
//				}
				if (mode == MODE_FLOCK) {
					ledsSetPattern(mode-1, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				} else {
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				}

			} else {
				// find the behavior mode
				mode = nbrDataGet(&nbrDataMode);
				if(printNbrs) cprintf("notleader.  mode = %d\n", mode);

				// Show behavior colors
				ledsSetPattern(mode-1, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
				switch (mode) {
				case MODE_FOLLOW: {
					Beh behFollow, behCluster;
					behMoveForward(&behOutput, MOTION_TV);
					behFollowPredesessor(&behFollow, &nbrList, MOTION_TV, FTL_RANGE);
					behClusterBroadcast(&behCluster, &nbrList, MOTION_TV, &broadcastMsg);
					behSubsume(&behOutput, &behCluster, &behFollow);
					break;
				}
				case MODE_FLOCK: {
					if(printNbrs) {
						cprintf("Team Size: %d\n", nbrList.size);
					}
					demoFlock(&behOutput, &behRadio, &nbrList);
					break;
				}
				case MODE_CLUSTER: {
					//behClusterBroadcast(&behOutput, &nbrListTeam, MOTION_TV, &broadcastTeamMsg[team]);
					nbrPtr = nbrListFindSource(&nbrList, &broadcastMsg);
					if (nbrPtr) {
						int32 orbitVelocity = MOTION_TV;
						if (nbrGetRange(nbrPtr) < BEH_CLUSTER_RANGE) {
							orbitVelocity = MOTION_TV/2;
						}
						behOrbit(&behOutput, nbrPtr, orbitVelocity);
					} else {
						behClusterBroadcast(&behOutput, &nbrList, MOTION_TV, &broadcastMsg);
					}
					break;
				}
				case MODE_IDLE:{
					break;
				}
				default:
					break;
				}
			}
			behChargeStop(&behCharge);
			behBumpAvoid(&behBump, behOutput.tv, 5);
			behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, behOutput.tv, &nbrList, behIsActive(&behBump));
			behSubsume2(&behOutput, &behIRObstacle, &behBump, &behCharge);
			behChargeStopLights(&behCharge);
		}

		if (printNbrs) {
			cprintf("\n\n");
		}

		neighborsPutMutex();
		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}



int main(void) {
	systemInit();
	systemPrintStartup();

	osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	// Start the scheduler

	osTaskStartScheduler();
	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}

