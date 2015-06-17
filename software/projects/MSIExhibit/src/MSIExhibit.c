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

//#define MSI_FAKE_HOST_ENABLE

#define BEHAVIOR_TASK_PRIORITY			(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD			50
#define NEIGHBOR_PERIOD					250

// flocking
//#define DEMO_TV_FLOCK					0
//#define BUTTON_COUNT_CHANGE				20
#define MSI_DEMO_HOPS_MAX				12


//#define SNAKE_NAV_TOWER_TIMEOUT			1000
#define MOTION_TV						110
#define MOTION_TV_FLOCK					(MOTION_TV*2/3)
#define MOTION_TV_ORBIT_CENTER			50
//#define CAPTAIN_LED_COUNTER_TIME		12

#define FTL_RANGE						300
#define FOLLOW_LEADER_AVOID_RANGE		300

#define BEH_CLUSTER_RANGE				300



#define MODE_IDLE						0
#define MODE_FOLLOW						1
#define MODE_FLOCK						2
#define MODE_CLUSTER					3

#define JOYSTICK_NUM_MSI				0

#define MUSEUM_RADIO_COMMAND_BEH_IDX	0
#define TEAM_LEADER_ON_TIME				4
#define TEAM_LEADER_TOTAL_TIME			25

#define MS_SECOND						1000
#define MS_MINUTE						(60 * MS_SECOND)

// time for a behavior to "settle".  After this time, the user can select another behavior
#define BEHAVIOR_READY_TIME 			(10 * MS_SECOND)

// time for a behavior to time out.  After this time, the robots become idle
#define BEHAVIOR_SLOW_TIME				(5 * MS_MINUTE)
#define BEHAVIOR_IDLE_TIME				(10 * MS_MINUTE)

#define EXHIBIT_MESSAGE_MODE_IDX		0
#define EXHIBIT_MESSAGE_BUILDING_IDX	1


/****** Team Broadcast messages *******/
BroadcastMessage broadcastMsg;
NbrData nbrDataMode;

uint32 behaviorChangeTime = 0;
static RadioCmd radioCmdExhibit;
static boolean fakeHostEnable = FALSE;

/******** user code ********/
#define FLOCK_CLUSTER_RANGE				300
#define FLOCK_CLUSTER_THRESHOLD			1

#define FLOCK_CLUSTER_COUNTER_MAX			120
#define FLOCK_CLUSTER_COUNTER_CLUSTER 		90
#define FLOCK_CLUSTER_COUNTER_FLOCK 		50


Beh* demoFlock(Beh* behOutputPtr, Beh* behRadioPtr, NbrList* nbrListPtr) {
	static boolean clusterFlock = FALSE;
	static uint32 clusterCounter = 0;

	Nbr* nbrPtr = nbrListGetClosestNbr(nbrListPtr);
	//if ((nbrListGetSize(nbrListPtr) <= FLOCK_CLUSTER_THRESHOLD) || (nbrPtr && (nbrGetRange(nbrPtr) > FLOCK_CLUSTER_RANGE))) {
	if (nbrPtr && (nbrGetRange(nbrPtr) > FLOCK_CLUSTER_RANGE)) {
		clusterCounter++;
	} else {
		clusterCounter-=3;
	}
	clusterCounter = bound(clusterCounter, 0, FLOCK_CLUSTER_COUNTER_MAX);

	if (clusterCounter > FLOCK_CLUSTER_COUNTER_CLUSTER) {
		clusterFlock = TRUE;
	}
	if (clusterCounter < FLOCK_CLUSTER_COUNTER_FLOCK) {
		clusterFlock = FALSE;
	}

	// If we are under the threshold for number of robots to flock or we
	// are too far away, cluster.
	if (clusterFlock) {
		//behClusterBroadcast(behOutputPtr, nbrListPtr, MOTION_TV, &broadcastMsg);
		behCluster(behOutputPtr, nbrListPtr, MOTION_TV);
		ledsSetPattern(LED_GREEN, LED_PATTERN_BLINK, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
	} else {
		// otherwise flock
		behFlock(behOutputPtr, nbrListPtr, MOTION_TV_FLOCK);
		behOutputPtr->rv += behRadioPtr->rv;
		behOutputPtr->tv = MOTION_TV_FLOCK;
		behSetActive(behOutputPtr);
		ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
	}

	return behOutputPtr;
}


#define ORBIT_RV	900

Beh* orbitCentroid(Beh* behOutputPtr, Joystick* joystickPtr, NbrList* nbrListPtr) {
	Beh behFollow = behInactive;
	Beh behCluster = behInactive;
	behSetTvRv(behOutputPtr, MOTION_TV, ORBIT_RV);
	behFollowPredesessor(&behFollow, nbrListPtr, MOTION_TV, FTL_RANGE);
	//behClusterBroadcast(&behCluster, nbrListPtr, MOTION_TV, &broadcastMsg);
	behSubsume(behOutputPtr, &behCluster, &behFollow);
	ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
	return behOutputPtr;
}


int32 rampVelocity(int32 vel, int32 velGoal, int32 step) {
	if (vel < velGoal) {
		vel += step;
		if (vel > velGoal) {
			vel = velGoal;
		}
	}
	if (vel > velGoal) {
		vel -= step;
		if (vel < velGoal) {
			vel = velGoal;
		}
	}
	return vel;
}


void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRound;
	boolean printNbrs;

	Beh behOutput, behIRObstacle, behBump, behCharge, behRadio;
	NbrList nbrList;
	NbrList nbrListAll;
	Nbr* nbrPtr;
//	uint32 printMem = 0;
	boolean behaviorBuilding = FALSE;
	RadioMessage exhibitMessage;

	neighborsInit(NEIGHBOR_PERIOD);

	centroidLiteInit();
	broadcastMsgCreate(&broadcastMsg, MSI_DEMO_HOPS_MAX);
	nbrDataCreate(&nbrDataMode, "mode", 4, MODE_IDLE);

	//irCommsSetXmitPower(IR_COMMS_POWER_MAX * 85 /100);

	remoteControlInit();
	radioCommandSetSubnet(2);
	radioCommandAddQueue(&radioCmdExhibit, "exhibit", 1);

	ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
	remoteControlLedsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);

	while (TRUE) {
		/* Initialize the output behavior to inactive */
		behOutput = behInactive;
		behRadio = behInactive;

		neighborsGetMutex();
		printNbrs = neighborsNewRoundCheck(&neighborRound);
		//printNbrs = FALSE;

		radioWatchdog();

		// update the joystick data from the RaspPi Host
		remoteControlUpdateJoysticks();

		// update the pose heading from the nav tower
		navTowerUpdateHeading(printNbrs);

		// make neighbor lists, all neighbors and robots only
		nbrListCreate(&nbrListAll);
		nbrListGetRobots(&nbrList, &nbrListAll);


		if (printNbrs) cprintf("Robot %d:\n", roneID);
		if (printNbrs){
			// print the battery voltages
			char num1[10],num2[10];
			sprintf(num1,"%1.2f", systemBatteryVoltageGet());
			sprintf(num2,"%1.2f", systemUSBVoltageGet());
			cprintf("vbat=%s vusb=%s charge=%d fast=%d\n", num1, num2, systemBatteryChargingGet(), systemBatteryFastChargingGet());
		}

		// print the stack usage for debugging
		//	uint32 timeTemp = osTaskGetTickCount() / 2000;
		//		if(printMem != timeTemp) {
		//			printMem = timeTemp;
		//			systemPrintMemUsage();
		//		}

		if (printNbrs) radioPrintCounters();

		Joystick* joystickPtr = remoteControlGetJoystick(JOYSTICK_NUM_MSI);
		if (printNbrs) cprintf("joy %d,%d,%d\n", joystickPtr->x, joystickPtr->y, joystickPtr->buttons);

#ifdef MSI_FAKE_HOST_ENABLE
		if (buttonsGet(BUTTON_RED) || buttonsGet(BUTTON_GREEN) || buttonsGet(BUTTON_BLUE)) {
			fakeHostEnable = TRUE;
		}
#endif

		if(remoteControlIsSerialHost() || fakeHostEnable) {
			neighborsXmitEnable(FALSE);
			behSetTvRv(&behOutput, 0, 0);

			boolean printExhibitStatus = FALSE;
			// read the joystick buttons for behavior changes
			if ((osTaskGetTickCount() - behaviorChangeTime) > BEHAVIOR_READY_TIME) {
				if(behaviorBuilding) {
					behaviorBuilding = FALSE;
					printExhibitStatus |= TRUE;
				}
			}

			// if we're not building a behavior, check the buttons
			uint8 modeOld = nbrDataGet(&nbrDataMode);
			if (!behaviorBuilding) {
				if (joystickPtr->buttons & JOYSTICK_BUTTON_RED) {
					nbrDataSet(&nbrDataMode, MODE_FOLLOW);
				} else if(joystickPtr->buttons & JOYSTICK_BUTTON_GREEN) {
					nbrDataSet(&nbrDataMode, MODE_FLOCK);
				} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BLUE) {
					nbrDataSet(&nbrDataMode, MODE_CLUSTER);
				}
			}

#ifdef MSI_FAKE_HOST_ENABLE
			if (buttonsGet(BUTTON_RED)) {
				nbrDataSet(&nbrDataMode, MODE_FOLLOW);
			} else if(buttonsGet(BUTTON_GREEN)) {
				nbrDataSet(&nbrDataMode, MODE_FLOCK);
			} else if(buttonsGet(BUTTON_BLUE)) {
				nbrDataSet(&nbrDataMode, MODE_CLUSTER);
			}
#endif

			// check for a change in mode
			if (modeOld != nbrDataGet(&nbrDataMode)) {
				// behavior change
				behaviorChangeTime = osTaskGetTickCount();
				behaviorBuilding = TRUE;
				printExhibitStatus |= TRUE;
			}

			if (!remoteControlJoystickIsActive(JOYSTICK_NUM_MSI, BEHAVIOR_IDLE_TIME)) {
				// no joystick activity for a long time.  go idle and flash the lights.
				nbrDataSet(&nbrDataMode, MODE_IDLE);
				behaviorBuilding = FALSE;
				printExhibitStatus |= TRUE;
			}

			if (printNbrs || printExhibitStatus) {
				cprintf("MSIExhibit mode=%d building=%d\n", nbrDataGet(&nbrDataMode), behaviorBuilding);
				exhibitMessage.command.data[EXHIBIT_MESSAGE_MODE_IDX] = nbrDataGet(&nbrDataMode);
				exhibitMessage.command.data[EXHIBIT_MESSAGE_BUILDING_IDX] = behaviorBuilding;
				radioCommandXmit(&radioCmdExhibit, ROBOT_ID_ALL, &exhibitMessage);
			}

			uint8 hostLEDColor;
			switch (nbrDataGet(&nbrDataMode)) {
			case MODE_FOLLOW: { hostLEDColor = LED_RED; break; }
			case MODE_FLOCK: { hostLEDColor = LED_GREEN; break; }
			case MODE_CLUSTER: { hostLEDColor = LED_BLUE; break; }
			case MODE_IDLE:
			default:{ hostLEDColor = LED_ALL; break; }
			}
			// flash the lights
			if(behaviorBuilding) {
				ledsSetPattern(hostLEDColor, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				remoteControlLedsSetPattern(hostLEDColor, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			} else {
				ledsSetPattern(hostLEDColor, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
				remoteControlLedsSetPattern(hostLEDColor, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			}

		} else {
			neighborsXmitEnable(TRUE);
			if (radioCommandReceive(&radioCmdExhibit, &exhibitMessage, 0) ) {
				// set the behavior mode
				nbrDataSet(&nbrDataMode, exhibitMessage.command.data[EXHIBIT_MESSAGE_MODE_IDX]);
				behaviorBuilding = exhibitMessage.command.data[EXHIBIT_MESSAGE_BUILDING_IDX];
			}

			if(printNbrs) {
				cprintf("mode=%d building=%d\n", nbrDataGet(&nbrDataMode), behaviorBuilding);
				nbrListPrint(&nbrListAll, "nbrs");
				nbrListPrint(&nbrList, "robots");
			}

			// update the broadcast messages to look for the team leaders
			broadcastMsgUpdateLeaderElection(&broadcastMsg, &nbrList);
			//TODO get mode from the radio now
			//broadcastMsgUpdateNbrData(&broadcastMsg, &nbrDataMode);

			// read the joystick.
			behRemoteControlCompass(&behRadio, joystickPtr, MOTION_TV);
			behOutput = behRadio;

			if(broadcastMsgIsSource(&broadcastMsg)) {
				// team leader
				if(printNbrs) cprintf("leader\n");

				switch (nbrDataGet(&nbrDataMode)) {
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
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
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
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);

//					orbitCentroid(&behOutput, joystickPtr, &nbrList);
//					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_IDLE:
				default: {
					behSetTvRv(&behOutput, 0, 0);
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					break;
				}
				}

			} else {
				// minion code
				if(printNbrs) cprintf("notleader.\n");
				switch (nbrDataGet(&nbrDataMode)) {
				case MODE_FOLLOW: {
					Beh behFollow, behCluster;
					behMoveForward(&behOutput, MOTION_TV);
					behFollowPredesessor(&behFollow, &nbrList, MOTION_TV, FTL_RANGE);
					behClusterBroadcast(&behCluster, &nbrList, MOTION_TV, &broadcastMsg);
					behSubsume(&behOutput, &behCluster, &behFollow);
					ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_FLOCK: {
					if(printNbrs) cprintf("Team Size: %d\n", nbrList.size);
					demoFlock(&behOutput, &behRadio, &nbrList);
					break;
				}
				case MODE_CLUSTER: {
					static int32 clusterVelocity;
					int32 clusterVelocityGoal;
					nbrPtr = nbrListFindSource(&nbrList, &broadcastMsg);
					if (nbrPtr) {
						clusterVelocityGoal = MOTION_TV/2;
						behOrbit(&behOutput, nbrPtr, clusterVelocity);
					} else {
						clusterVelocityGoal = MOTION_TV;
						behClusterBroadcast(&behOutput, &nbrList, clusterVelocity, &broadcastMsg);
					}
					clusterVelocity = rampVelocity(clusterVelocity, clusterVelocityGoal, 1);
//					orbitCentroid(&behOutput, joystickPtr, &nbrList);
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_IDLE:
				default:{
					behSetTvRv(&behOutput, 0, 0);
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
					break;
				}
				}
			}
			behChargeStop(&behCharge);
			behBumpAvoid(&behBump, behOutput.tv, 5);
			behIRObstacleAvoid_ExcludeRobots(&behIRObstacle, behOutput.tv, &nbrList, behIsActive(&behBump));
			behSubsume2(&behOutput, &behIRObstacle, &behBump, &behCharge);
			behChargeStopLights(&behCharge);
		}

		if (printNbrs) cprintf("\n\n");

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

