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
#define FOLLOW_LEADER_AVOID_RANGE		300

#define BEH_CLUSTER_RANGE				300



#define MODE_IDLE						0
#define MODE_FOLLOW						1
#define MODE_FLOCK						2
#define MODE_CLUSTER					3

#define FLOCK_CLUSTER_THRESHOLD			1
#define FLOCK_CLUSTER_RANGE				400

#define JOYSTICK_NUM_MSI				0

#define MUSEUM_RADIO_COMMAND_BEH_IDX	0
#define TEAM_LEADER_ON_TIME				4
#define TEAM_LEADER_TOTAL_TIME			25

#define MS_SECOND						1000
#define MS_MINUTE						(60 * MS_SECOND)

// time for a behavior fo "settle".  After this time, the user can select another behavior
#define BEHAVIOR_READY_TIME 			(10 * MS_SECOND)

// time for a behavior to time out.  After this time, the robots become idle
#define BEHAVIOR_SLOW_TIME				(1 * MS_MINUTE)
#define BEHAVIOR_IDLE_TIME				(3 * MS_MINUTE)


/****** Team Broadcast messages *******/
BroadcastMessage broadcastMsg;
NbrData nbrDataMode;

uint32 behaviorChangeTime = 0;
uint8 currentLED = LED_RED;


/******** user code ********/

#define TEAM_LEADER_ON_TIME			4
#define TEAM_LEADER_TOTAL_TIME		25


Beh* demoFlock(Beh* behOutputPtr, Beh* behRadioPtr, NbrList* nbrListPtr) {
	boolean farAway = FALSE;

	Nbr* nbrPtr = nbrListGetClosestNbr(nbrListPtr);
	if ((nbrListGetSize(nbrListPtr) <= FLOCK_CLUSTER_THRESHOLD) || (nbrPtr && (nbrGetRange(nbrPtr) > FLOCK_CLUSTER_RANGE))) {
		farAway = TRUE;
	}

	// If we are under the threshold for number of robots to flock or we
	// are too far away, cluster.
	if (farAway) {
		behClusterBroadcast(behOutputPtr, nbrListPtr, MOTION_TV, &broadcastMsg);
	} else {
		// otherwise flock
		behFlock(behOutputPtr, nbrListPtr, MOTION_TV_FLOCK);
		behOutputPtr->rv += behRadioPtr->rv;
		behOutputPtr->tv = MOTION_TV_FLOCK;
		behSetActive(behOutputPtr);
	}

	return behOutputPtr;
}


Beh* orbitCentroid(Beh* behOutputPtr, Joystick* joystickPtr, NbrList* nbrListPtr) {
	boolean farAway = FALSE;

//	if (roneID == 143) {
//		behSetTvRv(behOutputPtr, 0, 0);
//		return behOutputPtr;
//	}

//	Nbr* nbrPtr = nbrListGetClosestNbr(nbrListPtr);
//	if ((nbrListGetSize(nbrListPtr) < FLOCK_CLUSTER_THRESHOLD) || (nbrPtr && (nbrGetRange(nbrPtr) > FLOCK_CLUSTER_RANGE))) {
//		farAway = TRUE;
//	}

	if (nbrListGetSize(nbrListPtr) < FLOCK_CLUSTER_THRESHOLD) {
		farAway = TRUE;
	}

	// If we are under the threshold for number of robots to flock or we
	// are too far, cluster.
	if (farAway) {
		//behClusterBroadcast(behOutputPtr, nbrListPtr, MOTION_TV, &broadcastMsg);
		behSetTvRv(behOutputPtr, 0, 0);
		cprintf("faraway\n");
	} else {
		// otherwise orbit the centroid
		int32 centroidBearing = centroidLiteGetBearing();
		int32 centroidDistance = centroidLiteGetDistance();

		// mix in the remote control
		// compute the dot product between the joystick direction and the current heading
		int32 heading = encoderGetHeading();
		int32 headingJoystick = atan2MilliRad(joystickPtr->y, joystickPtr->x);
		int32 headingDiff = smallestAngleDifference(headingJoystick, heading);
		int32 tvGain = cosMilliRad(headingDiff) * 2; //TODO use some kind of joystick gain instead of 2
		//TODO: remember to divide by MILLIRAD_TRIG_SCALER when you use this

		behOrbitRangeRaw(behOutputPtr, centroidBearing, centroidDistance, MOTION_TV, BEH_CLUSTER_RANGE/3);

//		if (roneID == 143) {
//			behSetTvRv(behOutputPtr, 0, 0);
//		} else {
//			behOrbitRangeRaw(behOutputPtr, nbrGetBearing(nbrPtr), nbrGetRange(nbrPtr), MOTION_TV, BEH_CLUSTER_RANGE);
//			//behOrbitRangeRaw(behOutputPtr, centroidBearing, centroidDistance, MOTION_TV, BEH_CLUSTER_RANGE);
//		}
		cprintf("cent brg=%d dist=%d\n", centroidBearing, centroidDistance);

	}

	return behOutputPtr;
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
	boolean behaviorBuilding = FALSE;

	neighborsInit(NEIGHBOR_PERIOD);
	centroidLiteInit();
	broadcastMsgCreate(&broadcastMsg, MSI_DEMO_HOPS_MAX);
	nbrDataCreate(&nbrDataMode, "mode", 4, MODE_IDLE);

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
		printNbrs = FALSE;
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


		Joystick* joystickPtr = remoteControlGetJoystick(JOYSTICK_NUM_MSI);
		if (printNbrs) cprintf("joy %d,%d,%d\n", joystickPtr->x, joystickPtr->y, joystickPtr->buttons);
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
			if(joystickPtr->buttons & JOYSTICK_BUTTON_RED) {
				nbrDataSet(&nbrDataMode, MODE_FOLLOW);
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_GREEN) {
				nbrDataSet(&nbrDataMode, MODE_FLOCK);
			} else if(joystickPtr->buttons & JOYSTICK_BUTTON_BLUE) {
				nbrDataSet(&nbrDataMode, MODE_CLUSTER);
			}
		}

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
		}

		if(remoteControlIsSerialHost()) {
			neighborsXmitEnable(FALSE);
			behSetTvRv(&behOutput, 0, 0);

			switch (nbrDataGet(&nbrDataMode)) {
			case MODE_FOLLOW: { currentLED = LED_RED; break; }
			case MODE_FLOCK: { currentLED = LED_GREEN; break; }
			case MODE_CLUSTER: { currentLED = LED_BLUE; break; }
			case MODE_IDLE:
			default:{ currentLED = LED_ALL; break; }
			}
			// flash the lights
			if(behaviorBuilding) {
				ledsSetPattern(currentLED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
				remoteControlLedsSetPattern(currentLED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			} else {
				ledsSetPattern(currentLED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
				remoteControlLedsSetPattern(currentLED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			}

		} else {
			neighborsXmitEnable(TRUE);
			if(printNbrs) {
				nbrListPrint(&nbrListAll, "nbrs");
				nbrListPrint(&nbrList, "robots");
			}

			// update the broadcast messages to look for the team leaders
			broadcastMsgUpdateLeaderElection(&broadcastMsg, &nbrList);
			broadcastMsgUpdateNbrData(&broadcastMsg, &nbrDataMode);

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
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_CLUSTER: {
//					if(behIsActive(&behRadio)) {
//						behRemoteControlCompass(&behOutput, joystickPtr, MOTION_TV_ORBIT_CENTER);
//					}
//					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					orbitCentroid(&behOutput, joystickPtr, &nbrList);
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_IDLE:
				default: {
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_SLOW);
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
					ledsSetPattern(LED_GREEN, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_CLUSTER: {
//					nbrPtr = nbrListFindSource(&nbrList, &broadcastMsg);
//					if (nbrPtr) {
//						int32 orbitVelocity = MOTION_TV;
//						if (nbrGetRange(nbrPtr) < BEH_CLUSTER_RANGE) {
//							orbitVelocity = MOTION_TV/2;
//						}
//						behOrbit(&behOutput, nbrPtr, orbitVelocity);
//					} else {
//						behClusterBroadcast(&behOutput, &nbrList, MOTION_TV, &broadcastMsg);
//					}
//					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);

					orbitCentroid(&behOutput, joystickPtr, &nbrList);
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_MED);
					break;
				}
				case MODE_IDLE:
				default:{
					ledsSetPattern(LED_ALL, LED_PATTERN_PULSE, LED_BRIGHTNESS_HIGH, LED_RATE_SLOW);
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

