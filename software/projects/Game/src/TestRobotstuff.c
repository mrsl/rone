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
#define MOTION_TV_ORBIT_CENTER			35
#define CAPTAIN_LED_COUNTER_TIME		12


#define NAV_TOWER_LOW_RONE_ID			124
#define NAV_TOWER_HIGH_RONE_ID			125

#define MODE_IDLE						0
#define MODE_FOLLOW						1
#define MODE_CLUSTER					2
#define MODE_FLOCK						3

#define FLOCK_CLUSTER_THRESHOLD			1

#define JOYSTICK_TIMEOUT_TEAM			200000
#define JOYSTICK_TIMEOUT_BEH			20000

#define JOYSTICK_NUM					1

#define MUSEUM_RADIO_COMMAND_BEH_IDX	0
#define TEAM_LEADER_ON_TIME				4
#define TEAM_LEADER_TOTAL_TIME			25

#define BEHAVIOR_READY_TIME 			10000
#define BEHAVIOR_IDLE_TIME				60000


/****** Team Broadcast messages *******/
NbrData nbrDataTeam;
BroadcastMessage broadcastTeamMsg[TEAM_COUNT];
NbrData nbrDataMode[TEAM_COUNT];
const char const* teamNames[TEAM_COUNT] = {"red","green","blue"};

uint32 behaviorChangeTime = 0;
uint8 currentLED = LED_RED;
boolean hostFlag = FALSE;
static RadioCmd radioCmdBehavior;



/******** user code ********/

#define TEAM_LEADER_ON_TIME			4
#define TEAM_LEADER_TOTAL_TIME		25

#define BEHAVIOR_READY_TIME 		10000
#define BEHAVIOR_IDLE_TIME			60000


Beh* demoFlock(Beh* behOutputPtr, Beh* behRadioPtr, NbrList* nbrListTeamPtr, uint8 team) {
	if ((nbrListGetSize(nbrListTeamPtr) > FLOCK_CLUSTER_THRESHOLD) ||
			(broadcastMsgIsSource(&broadcastTeamMsg[team]))){
		// you are the source, or a minion surrounded by member of your team.  flock.
		behFlock(behOutputPtr, nbrListTeamPtr, MOTION_TV_FLOCK);
		behOutputPtr->rv += behRadioPtr->rv;
		behOutputPtr->tv = MOTION_TV_FLOCK;
		behSetActive(behOutputPtr);
	} else {
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
	RadioMessage radioMessage;
	uint8 irBeaconNumIDs = 4;
	uint8 irBeaconIDs[] = {124, 125, 126, 127};

	uint32 captainLEDCounter = 0;

	neighborsInit(300);
	nbrDataCreate(&nbrDataTeam, "team", 4, TEAM_NONE);
	for (i = 0; i < TEAM_COUNT; i++) {
		broadcastMsgCreate(&broadcastTeamMsg[i], MSI_DEMO_HOPS_MAX);
		nbrDataCreate(&nbrDataMode[i], teamNames[i], 4, MODE_FOLLOW);
	}

	irCommsSetXmitPower(IR_COMMS_POWER_MAX * 75 /100);

	remoteControlInit();
	radioCommandSetSubnet(2);

	ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
	remoteControlLedsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);

	while (TRUE) {
		/* Initialize the output behavior to inactive */
		behOutput = behInactive;

		neighborsGetMutex();
		printNbrs = neighborsNewRoundCheck(&neighborRound);
		remoteControlUpdateJoysticks();

		// look for the nav tower
		nbrListCreate(&nbrList);
		nbrListGetRobots(&nbrListRobots, &nbrList, irBeaconIDs, irBeaconNumIDs);

		behRadio = behInactive;

		nbrNavTowerLowPtr = nbrListGetNbrWithID(&nbrList, NAV_TOWER_LOW_RONE_ID);
		nbrNavTowerHighPtr = nbrListGetNbrWithID(&nbrList, NAV_TOWER_HIGH_RONE_ID);
		if(nbrNavTowerLowPtr || nbrNavTowerHighPtr) {
			navTowerTime = osTaskGetTickCount();
		}

		if (printNbrs){
			if (nbrNavTowerLowPtr){
				cprintf("(NavTower High) ID: %d, bearing:%d, orientation:%d \n", nbrNavTowerHighPtr->ID, nbrNavTowerHighPtr->bearing, nbrNavTowerHighPtr->orientation);
			} else if (nbrNavTowerLowPtr){
				cprintf("(NavTower Low) ID: %d, bearing:%d, orientation:%d \n", nbrNavTowerLowPtr->ID, nbrNavTowerLowPtr->bearing, nbrNavTowerLowPtr->orientation);
			}
		}


		if (!remoteControlJoystickIsActive(JOYSTICK_NUM, BEHAVIOR_IDLE_TIME)) {
			// no joystick activity for a while.  go idle and flash the lights.
			ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			remoteControlLedsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
		} else if (osTaskGetTickCount() - behaviorChangeTime > BEHAVIOR_READY_TIME) {
			// behavior is settled.  show a steady light
			ledsSetPattern(currentLED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			remoteControlLedsSetPattern(currentLED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			// TODO flash the other lights occasionally

			//  look for a new behavior button press
			Joystick* joystickPtr = remoteControlGetJoystick(JOYSTICK_NUM);
			if(joystickPtr->buttons & JOYSTICK_BUTTON_TOP) {
				if (currentLED != LED_RED) {
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					remoteControlLedsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					behaviorChangeTime = osTaskGetTickCount();
				}
				currentLED = LED_RED;
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_MIDDLE) {
				if (currentLED != LED_GREEN) {
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					remoteControlLedsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					behaviorChangeTime = osTaskGetTickCount();
				}
				currentLED = LED_GREEN;
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BOTTOM) {
				if (currentLED != LED_BLUE) {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					remoteControlLedsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
					behaviorChangeTime = osTaskGetTickCount();
				}
				currentLED = LED_BLUE;
			}
		}


		if(remoteControlIsSerialHost()) {
			hostFlag = TRUE;
			//radioMessage.command.data[MUSEUM_RADIO_COMMAND_BEH_IDX] = currentLED;
			//radioCommandXmit(&radioCmdBehavior, ROBOT_ID_ALL, &radioMessage);
		} else {
			if(printNbrs) {
				nbrListPrint(&nbrList, "nbrs");
				nbrListPrint(&nbrListRobots, "robots");
			}

			/* see how many joysticks are active
			 * count the number of teams and make a team map
			 */
			uint8 teamCount = 0;
			uint8 teams[REMOTE_CONTROL_JOYSTICK_NUM];
			for (i = TEAM_RED; i < TEAM_COUNT; i++) {
				if (remoteControlJoystickIsActive(i, JOYSTICK_TIMEOUT_TEAM)) {
					//TODO hack to use only green
					if (i == TEAM_GREEN) {
						teams[teamCount++] = i;
						if (printNbrs)	cprintf("team %d active\n", i);
					}
				}
			}

			int8 a;

			// select your team.  Use the team map.  If not, then select TEAM_NONE
			if(teamCount > 0) {
				uint8 teamIdx = roneID % teamCount;
				nbrDataSet(&nbrDataTeam, teams[teamIdx]);
			} else {
				nbrDataSet(&nbrDataTeam, TEAM_NONE);
			}
			uint8 team = nbrDataGet(&nbrDataTeam);
			if (printNbrs) {
				cprintf("myteam %d\n", team);
			}

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

			if (printNbrs) {
				nbrListPrint(&nbrListTeam, "Team Members");
			}

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
				if (printNbrs) {
					cprintf("joy%d %d,%d\n", team, joystickPtr->x, joystickPtr->y);
				}
				behRemoteControlCompass(&behRadio, joystickPtr, MOTION_TV, nbrNavTowerHighPtr, nbrNavTowerLowPtr);
				if (printNbrs) {
					cprintf("beh %d,%d\n", behOutput.tv, behOutput.rv);
				}
			}

			/* Code for the regular robots that are not the head of the snake */
			uint8 mode;
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

//				if (remoteControlJoystickIsActive(team, JOYSTICK_TIMEOUT_TEAM)) {
//					if (radioCommandReceive(&radioCmdBehavior, &radioMessage, 0) ) {
//						nbrDataSet(&nbrDataMode[team], radioMessage.command.data[MUSEUM_RADIO_COMMAND_BEH_IDX]);
//					}
//					behOutput = behRadio;
//				}

				mode = nbrDataGet(&nbrDataMode[team]);
				switch (mode) {
				case MODE_FOLLOW: {
					// avoid neighbors who are in front of you
					nbrPtr = nbrListGetClosestNbrToBearing(&nbrListTeam, 0);
					if (abs(nbrGetBearing(nbrPtr)) < MILLIRAD_DEG_45) {
						behMoveFromNbr(&behOutput, nbrPtr, MOTION_TV);
					} else {
						behOutput.tv = MOTION_TV;
					}
					break;
				}
				case MODE_CLUSTER: {
					behRemoteControlCompass(&behOutput, joystickPtr, MOTION_TV_ORBIT_CENTER, nbrNavTowerHighPtr, nbrNavTowerLowPtr);
					break;
				}
				case MODE_FLOCK: {
					if(printNbrs) {
						cprintf("Team Size: %d\n", nbrListTeam.size);
					}
					demoFlock(&behOutput, &behRadio, &nbrListTeam, team);
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

				if ((captainLEDCounter < TEAM_LEADER_ON_TIME)) {
					if (hostFlag == FALSE)	ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				} else {
					if (hostFlag == FALSE)  ledsSetPattern(mode-1, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);	//###
				}

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
						//behClusterBroadcast(&behOutput, &nbrListTeam, MOTION_TV, &broadcastTeamMsg[team]);
						nbrPtr = nbrListFindSource(&nbrListTeam, &broadcastTeamMsg[team]);
						if (nbrPtr) {
							behOrbit(&behOutput, nbrPtr, MOTION_TV);
						} else {
							behClusterBroadcast(&behOutput, &nbrListTeam, MOTION_TV, &broadcastTeamMsg[team]);
						}
						break;
					}
					case MODE_FLOCK: {
						if(printNbrs) {
							cprintf("Team Size: %d\n", nbrListTeam.size);
						}
						demoFlock(&behOutput, &behRadio, &nbrListTeam, team);
						break;
					}
					case MODE_IDLE:{
						break;
					}
					default:
						break;
					}
				} else {
					/* If the regular robot is teamless */
					/* Show that you are not on a team */
					if (hostFlag == FALSE) ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
					if(printNbrs) {
						cprintf("teamless.\n");
					}
				}
			}
			behBumpAvoid(&behBump, behOutput.tv, 5);
			behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, behOutput.tv, &nbrListTeam, behIsActive(&behBump));
			behSubsume(&behOutput, &behIRObstacle, &behBump);
		}

		if (printNbrs) {
			cprintf("\n\n");
		}
		neighborsPutMutex();
		if (hostFlag == TRUE ){
			behOutput.tv = 0;
			behOutput.rv = 0;
//			if (mode == MODE_IDLE){
//				ledsSetPattern(LED_ALL, LED_PATTERN_OFF, LED_BRIGHTNESS_LOWEST, LED_RATE_SNAIL);
//			}
		}
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
