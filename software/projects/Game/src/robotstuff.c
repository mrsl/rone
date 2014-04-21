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
//in milliseconds
#define BEHAVIOR_TASK_PERIOD			50

//stuff for flocking
#define DEMO_TV_FLOCK					0
#define BUTTON_COUNT_CHANGE				20
#define MSI_DEMO_HOPS_MAX				12

#define TEAM_RED						0
#define TEAM_GREEN						1
#define TEAM_BLUE						2
#define TEAM_NONE						3
#define TEAM_COUNT						3

#define SNAKE_NAV_TOWER_TIMEOUT			1000
#define MOTION_TV						85
#define MOTION_TV_FLOCK					(MOTION_TV*2/3)
#define CAPTAIN_LED_COUNTER_TIME		12


#define NAV_TOWER_LOW_RONE_ID				124
#define NAV_TOWER_HIGH_RONE_ID				125

#define MODE_IDLE						0
#define MODE_FOLLOW						1
#define MODE_CLUSTER					2
#define MODE_FLOCK						3

#define FLOCK_CLUSTER_THRESHOLD			1

#define JOYSTICK_TIMEOUT_TEAM			200000
#define JOYSTICK_TIMEOUT_BEH			20000

//void serialCmdUIFunc(char* arguments);

/******Broadcast*******/
NbrData nbrDataTeam;
BroadcastMessage broadcastTeamMsg[TEAM_COUNT];
NbrData nbrDataMode[TEAM_COUNT];
const char const* teamNames[TEAM_COUNT] = {"red","green","blue"};

/******** user code ********/

#define TEAM_LEADER_ON_TIME			4
#define TEAM_LEADER_TOTAL_TIME		25


Beh* demoFlock(Beh* behOutputPtr, Beh* behRadioPtr, NbrList* nbrListTeamPtr, uint8 team) {
	if ((nbrListGetSize(nbrListTeamPtr) > FLOCK_CLUSTER_THRESHOLD) ||
			(broadcastMsgIsSource(&broadcastTeamMsg[team]))){
		// you are the source, or a minion surrounded by member of your team.  flock.
		behFlock(behOutputPtr, nbrListTeamPtr, MOTION_TV_FLOCK);
		behOutputPtr->rv += behRadioPtr->rv;
		behOutputPtr->tv = MOTION_TV_FLOCK;
		behSetActive(behOutputPtr);
	}
	else {
		behClusterBroadcast(behOutputPtr, nbrListTeamPtr, MOTION_TV, &broadcastTeamMsg[team]);
	}

	return behOutputPtr;
}

void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRound;
	boolean printNbrs;

	int32 i,j;
	Beh behOutput, behMove, behIRObstacle, behBump, behRadio;
	NbrList nbrList, nbrListRobots;
	Nbr* nbrPtr;
	Nbr* nbrNavTowerHighPtr;
	Nbr* nbrNavTowerLowPtr;
	uint32 navTowerTime = 0;

	uint32 captainLEDCounter = 0;

	neighborsInit(300);
	nbrDataCreate(&nbrDataTeam, "team", 4, TEAM_NONE);
	for (i = 0; i < TEAM_COUNT; i++) {
		broadcastMsgCreate(&broadcastTeamMsg[i], MSI_DEMO_HOPS_MAX);
		nbrDataCreate(&nbrDataMode[i], teamNames[i], 4, MODE_FOLLOW);
	}
	remoteControlInit();
	radioCommandSetSubnet(2);


	while (TRUE) {
		/* Initialize the output behavior to inactive */
		behOutput = behInactive;

		neighborsGetMutex();
		printNbrs = neighborsNewRoundCheck(&neighborRound);
		remoteControlUpdateJoysticks();

		// look for the nav tower
		nbrListCreate(&nbrList);
		nbrListGetRobots(&nbrListRobots, &nbrList);

		behRadio = behInactive;

		nbrNavTowerLowPtr = nbrListGetNbrWithID(&nbrList, NAV_TOWER_LOW_RONE_ID);
		nbrNavTowerHighPtr = nbrListGetNbrWithID(&nbrList, NAV_TOWER_HIGH_RONE_ID);
		if(nbrNavTowerLowPtr || nbrNavTowerHighPtr) {
			navTowerTime = osTaskGetTickCount();
		}
		//TODO make the robot run always - even with no nav tower
		navTowerTime = osTaskGetTickCount();

		if(remoteControlIsSerialHost()) {
			//### host robot color for joystick buttons
			//!!! bug in LED: faster and faster
			Joystick* joystickPtr = remoteControlGetJoystick(1);
			if(joystickPtr->buttons & JOYSTICK_BUTTON_TOP) {
				ledsSetPattern(LED_RED, LED_PATTERN_CLAW, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_MIDDLE) {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CLAW, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BOTTOM) {
				ledsSetPattern(LED_BLUE, LED_PATTERN_CLAW, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			} else {
				//ledsSetPattern(LED_ALL, LED_PATTERN_CLAW, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			}
			// you are the joystick host.  flash vertical and sit still
			//ledsSetPattern(LED_BLUE, LED_PATTERN_CLAW, LED_BRIGHTNESS_MED, LED_RATE_FAST);
//			//hack for accel remote
//			if(!remoteControlIsSerialHost()) {
//				remoteControlSendMsgAccel(0);
//			}


		} else if ((osTaskGetTickCount() - navTowerTime) > SNAKE_NAV_TOWER_TIMEOUT) {
			// flash idle light
			ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
		} else {
//			// sequential leader election - removed for simplicity of mod team selection
//			// update the broadcast messages and look for the team leaders
//			broadcastMsgUpdateLeaderElection(&broadcastRedMsg);
//			if(broadcastMsgGetSourceID(&broadcastRedMsg) == roneID) {
//				// you are the red leader.  Don't participate in other leader elections
//				broadcastMsgUpdate(&broadcastGreenMsg);
//				broadcastMsgUpdate(&broadcastBlueMsg);
//				nbrDataSet(&nbrDataTeam, TEAM_RED);
//			} else {
//				broadcastMsgUpdateLeaderElection(&broadcastGreenMsg);
//				if(broadcastMsgGetSourceID(&broadcastGreenMsg) == roneID) {
//					// you are the green leader.  Don't participate in other leader elections
//					broadcastMsgUpdate(&broadcastBlueMsg);
//					nbrDataSet(&nbrDataTeam, TEAM_GREEN);
//				} else {
//					broadcastMsgUpdateLeaderElection(&broadcastBlueMsg);
//					if(broadcastMsgGetSourceID(&broadcastBlueMsg) == roneID) {
//						nbrDataSet(&nbrDataTeam, TEAM_BLUE);
//					}
//				}
//			}

			if(printNbrs) {
				nbrListPrint(&nbrList, "nbrs");
				nbrListPrint(&nbrListRobots, "robots");
//				nbrPtr = nbrListGetFirst(&nbrList);
//				if(nbrPtr) {
//					irCommsOrientationBitMatrixPrint(nbrPtr->orientationsBitsMatrix);
//				}
			}

			// see how many joysticks are active
			// count the number of teams and make a team map
			uint8 teamCount = 0;
			uint8 teams[REMOTE_CONTROL_JOYSTICK_NUM];

			for (i = TEAM_RED; i < TEAM_COUNT; i++) {
				if(remoteControlJoystickIsActive(i, JOYSTICK_TIMEOUT_TEAM)) {
					//TODO hack to use only green
					if (i == TEAM_GREEN) {
						teams[teamCount++] = i;
						if (printNbrs) cprintf("team %d active\n", i);
					}
				}
			}

			// select your team.  Use the team map.  If not, then select TEAM_NONE
			if(teamCount > 0) {
				uint8 teamIdx = roneID % teamCount;
				nbrDataSet(&nbrDataTeam, teams[teamIdx]);
			} else {
				nbrDataSet(&nbrDataTeam, TEAM_NONE);
			}
			uint8 team = nbrDataGet(&nbrDataTeam);
			if (printNbrs) cprintf("myteam %d\n", team);


			// update the broadcast messages to look for the team leaders
			if (printNbrs) {
				for (i = 0; i < TEAM_COUNT; i++) {
					if (i == team) {
						broadcastMsgUpdateLeaderElection(&broadcastTeamMsg[i], &nbrListRobots);
					} else {
						broadcastMsgUpdate(&broadcastTeamMsg[i], &nbrListRobots);
					}
					broadcastMsgUpdateNbrData(&broadcastTeamMsg[i], &nbrDataMode[i]);
				}
			}

			// find your teammates
			NbrList nbrListTeam;
			nbrListClear(&nbrListTeam);
			nbrListFindNbrsWithDataEqual(&nbrListTeam, &nbrListRobots, &nbrDataTeam, team);

			if (printNbrs) nbrListPrint(&nbrListTeam, "Team Members");

			// see if you are your team leader
			boolean teamLeader = FALSE;
			Joystick* joystickPtr = remoteControlGetJoystick(team);
			if (team < TEAM_NONE) {
				if (broadcastMsgIsSource(&broadcastTeamMsg[team])) {
					// you are the team leader
					teamLeader = TRUE;
					if(printNbrs) cprintf("leader of team %d\n", team);
				}

				// read the joystick for your team.  We read into the behRadio so we can
				// use the joystick for flocking
				joystickPtr = remoteControlGetJoystick(team);
				if (printNbrs) cprintf("joy%d %d,%d\n", team, joystickPtr->x, joystickPtr->y);
				behRemoteControlCompass(&behRadio, joystickPtr, nbrNavTowerLowPtr, nbrNavTowerHighPtr);
				if (printNbrs) cprintf("beh %d,%d\n", behOutput.tv, behOutput.rv);

			}

			/* Code for the regular robots that are not the head of the snake */
			uint8 mode;	//###
			if(teamLeader) {
				// team leader
				if (remoteControlJoystickIsActive(team, JOYSTICK_TIMEOUT_TEAM)) {
					if(joystickPtr->buttons & JOYSTICK_BUTTON_TOP) {
						nbrDataSet(&nbrDataMode[team], MODE_FOLLOW);
					} else if(joystickPtr->buttons & JOYSTICK_BUTTON_MIDDLE) {
						nbrDataSet(&nbrDataMode[team], MODE_CLUSTER);
					} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BOTTOM) {
						nbrDataSet(&nbrDataMode[team], MODE_FLOCK);
					}
					behOutput = behRadio;
				}
				mode = nbrDataGet(&nbrDataMode[team]);
				switch (mode) {
				case MODE_FOLLOW: {
					behOutput.tv = MOTION_TV;
					break;
				}
				case MODE_CLUSTER: {
					break;
				}
				case MODE_FLOCK: {
					if(printNbrs) cprintf("Team Size: %d\n", nbrListTeam.size);

					demoFlock(&behOutput, &behRadio, &nbrListTeam, team);
					//behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, MOTION_TV, &nbrListRobots);
					//behSubsume(&behOutput, &behOutput, &behIRObstacle);
					break;
				}
				default:
					break;
				}
				if(printNbrs) cprintf("leader. mode = %d\n", mode);


				// Show team captain colors (alternate between team colors and all the LEDs)
				if (captainLEDCounter == 0) {
					captainLEDCounter = TEAM_LEADER_TOTAL_TIME;
				}
				if (captainLEDCounter > 0) {
					captainLEDCounter--;
				}

				if ((captainLEDCounter < TEAM_LEADER_ON_TIME)) {
					ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
					//ledsSetPattern(team, LED_PATTERN_ON, LED_BRIGHTNESS_HIGHER, LED_RATE_FAST);
				} else {
					//ledsSetPattern(team, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					ledsSetPattern(mode-1, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);	//###
				}

//				if ((captainLEDCounter & 0x3F) == 0) {
//					//ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGHER, LED_RATE_FAST);
//					ledsSetPattern(team, LED_PATTERN_ON, LED_BRIGHTNESS_HIGHER, LED_RATE_FAST);
//				} else {
//					ledsSetPattern(team, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
//				}
//				captainLEDCounter++;

			} else {
				// If the regular robot has a team
				if (team < TEAM_NONE) {
					// find the behaviormode
					mode = nbrDataGet(&nbrDataMode[team]);
					if(printNbrs) cprintf("notleader.  mode = %d\n", mode);

					// Show team colors
					//ledsSetPattern(team, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					ledsSetPattern(mode-1, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);	//###
					//TODO follow the leader + bubble sort
					switch (mode) {
					case MODE_FOLLOW: {
						//if(remoteControlJoystickIsActive(team, JOYSTICK_TIMEOUT_BEH)) {
							uint8 parentID;
							Beh behFollow, behCluster;
							behMoveForward(&behOutput, MOTION_TV);
							behFollowPredesessor(&behFollow, &nbrListTeam, MOTION_TV);
							behClusterBroadcast(&behCluster, &nbrListTeam, MOTION_TV, &broadcastTeamMsg[team]);
							behSubsume(&behOutput, &behCluster, &behFollow);
						//}
						break;
					}
					case MODE_CLUSTER: {
						behClusterBroadcast(&behOutput, &nbrListTeam, MOTION_TV, &broadcastTeamMsg[team]);
						break;
					}
					case MODE_FLOCK: {
						if(printNbrs) cprintf("Team Size: %d\n", nbrListTeam.size);

						demoFlock(&behOutput, &behRadio, &nbrListTeam, team);
						//behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, MOTION_TV, &nbrListRobots);
						//behSubsume(&behOutput, &behOutput, &behIRObstacle);

						break;
					}
					default:
						break;
					}
				} else {
					/* If the regular robot is teamless */
					/* Show that you are not on a team */
					ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
					if(printNbrs) cprintf("teamless. ");
				}
			}
			//nbrListFindNbrsWithDataEqual(nbrListFrontierPtr, &nbrList, &nbrDataTeam, TEAM_RED);

//
//				//TODO bump by team captain = change to that captain's team
//				} else {
//					/* Code for Team Captains*/
//					//captains are the only 3 controlled by the radio signal
//
//					/* Show captain colors */
//					if (team == TEAM_RED){
//						ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//					}
//					else if (team == TEAM_GREEN){
//						ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//					}
//					else if (team == TEAM_BLUE){
//						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
//					}
//				}
//
//				if (roneID != NAV_TOWER_RONE_ID ) { //if you are not the beacon, you may move
//					/* Flocking Codes */
//					behFlock(&behOutput, &nbrList, DEMO_TV_FLOCK);
//					behSetActive(&behOutput);
//					if (behRadio.active) {
//						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_TURBO);
//						behOutput.rv += behRadio.rv;
//						behOutput.tv += behRadio.tv;
//					} else {
//						ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);
//					}
//				}
//
//			}
			/*	// en
			cprintf(" vUSB%03d, vBat%03d, bump%02X",
					systemUSBVoltageGet(),
					systemBatteryVoltageGet(),
					bumpSensorsGetBits());
			 */
			behBumpAvoid(&behBump, behOutput.tv, 5);
			behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, MOTION_TV, &nbrListTeam, behIsActive(&behBump));
			behSubsume(&behOutput, &behIRObstacle, &behBump);
		}

		//if (printNbrs) cprintf("\n\n");
		neighborsPutMutex();
		motorSetBeh(&behOutput);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}

//void serialCmdUIFunc(char* arguments) {
//	char parseStr[2];
//	uint8 i, j, bitNum, joystickVal;
//	uint8 gotCorrectTeamString = 0;
//	uint8 prevDemoMode[3];
//	char* msg = radioCommandGetDataPtr(&radioMessage);
//
//	prevDemoMode[0] = msg[2];
//	prevDemoMode[1] = msg[5];
//	prevDemoMode[2] = msg[8];
//
//	//gotCorrectString = 0;
//
//	msg[0] = 'g';
//	msg[1] = 'r';
//	msg[4] = 'g';
//	msg[7] = 'b';
//
//	for (i=0; i<3; i++) // parse message for each team
//	{
//		gotCorrectTeamString = 0;
//		bitNum = i * 4; // the bit number that the message starts for each team
//		for (j=bitNum; j<bitNum+2; j++) {
//			if (arguments[j] == 0) {
//				return;
//			}
//			parseStr[j-bitNum] = arguments[j];
//		}
//
//		joystickVal = atoi_hex8(parseStr);
//		msg[3 + i * 3] = joystickVal;
//
//		for (j=bitNum+2; j<bitNum+4; j++) {
//			if (arguments[j] == 0) {
//				return;
//			}
//			parseStr[j-bitNum-2] = arguments[j];
//		}
//
//		buttonVal = atoi_hex8(parseStr);
//		//radiomsg[4 + i * 3] = buttonVal;
//
//		if (joystickVal != 0 || buttonVal != 0)
//		{
//			gotCorrectTeamString = 1;
//			runMode[i] = RUNMODE_ACTIVE;
//		} else if (runMode[i] != RUNMODE_IDLE) {
//			//runMode[i] = RUNMODE_WAIT;
//			runMode[i] = RUNMODE_IDLE;		// hack to get rid of screensaver
//		}
//
//		switch(runMode[i]) {
//		case RUNMODE_ACTIVE:
//			demoMode[i] = DEMOMODE_IDLE;
//
//			if (gotCorrectTeamString) {
//				switch (buttonVal) {
//				case BUTTON_NONE:
//					demoMode[i] = DEMOMODE_FOLLOW;
//					break;
//				case BUTTON_ONE:
//					demoMode[i] = DEMOMODE_FLOCK;
//					break;
//				case BUTTON_TWO:
//					demoMode[i] = DEMOMODE_CLUSTER;
//					break;
//				case BUTTON_THREE:
//					demoMode[i] = DEMOMODE_DISPERSE;
//					break;
//	//			case BUTTON_ONE + BUTTON_TWO: // example for more button combinations in the future
//	//				demoMode = DEMOMODE_IDLE;
//	//				break;
//				default:
//					demoMode[i] = prevDemoMode[i];
//					break;
//				}
//			}
//			break;
//		case RUNMODE_IDLE:
//			demoMode[i] = DEMOMODE_IDLE;
//			break;
//		default:
//			break;
//		}
//
//		msg[2 + i * 3] = demoMode[i];
//	}
//
//	radioCommandXmit(&radioCmdRemoteControl, ROBOT_ID_ALL, &radioMessage);
//	ledsSetPattern(LED_ALL, LED_PATTERN_MANUAL, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	ledObstacleBits(joystickVal, LED_RED, 25);
//}

int main(void) {
	systemInit();
	systemPrintStartup();

	osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	// Start the scheduler

	osTaskStartScheduler();
	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}
