/*
 * @file externalPose.c
 *
 *
 * @since Nov 28, 2012
 * @author James McLurkin
 *
 */

#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "roneos.h"
#include "ronelib.h"

/******** Defines ********/

/* Sizes for the external pose serial command data. */
#define EP_ARG_COUNT_NOTHETA	3
#define EP_ARG_COUNT_WITHTHETA	4
#define EP_ARG_COUNT_MIN		EP_ARG_COUNT_NOTHETA
#define EP_ARG_COUNT_MAX		EP_ARG_COUNT_WITHTHETA
#define EP_ARG_SIZE				10

#define EXTERNAL_POSE_SIZEOF_ARG			4
#define EXTERNAL_POSE_MESSAGE_ID_IDX		0
#define EXTERNAL_POSE_MESSAGE_X_IDX			(EXTERNAL_POSE_MESSAGE_ID_IDX + EXTERNAL_POSE_SIZEOF_ARG)
#define EXTERNAL_POSE_MESSAGE_Y_IDX			(EXTERNAL_POSE_MESSAGE_X_IDX + EXTERNAL_POSE_SIZEOF_ARG)
#define EXTERNAL_POSE_MESSAGE_THETA_IDX		(EXTERNAL_POSE_MESSAGE_Y_IDX + EXTERNAL_POSE_SIZEOF_ARG)

#define EXTERNAL_POSE_HOST_TIMEOUT			2000

/******** Variables ********/

static SerialCmd serialCmdEP;
static RadioCmd externalPoseRadioCmd;
static ExternalPose externalPose;
static ExternalPose externalPoseNbrPoses[NEIGHBOR_MAX];
static uint32 externalPoseCommandTimeStamp = 0;
static boolean isHost = FALSE;
static boolean isActive = FALSE;


/******** Private Functions ********/

/*
 * @brief (Private) marshall the external pose data into a radio message
 * @param externalPosePtr the pose of a robot (unchanged)
 * @param radioCmdDataPtr message where the pose information is written
 *
 * @return void
 */
static void externalPoseMarshallRadioCmd(ExternalPose* externalPosePtr, char* radioCmdDataPtr) {
	radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_ID_IDX] = externalPosePtr->ID;
	*(int32*)(&radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_X_IDX]) = externalPosePtr->pose.x;
	*(int32*)(&radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_Y_IDX]) = externalPosePtr->pose.y;
	*(int32*)(&radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_THETA_IDX]) = externalPosePtr->pose.theta;
}


/*
 * @brief (Private) stores data sent over radio into a
 *        queue of structs. (one line, one robot)
 *
 * @param command pointer to the start of the  data
 *
 * @return void
 */
static void externalPoseSerialCmdFunc(char* command) {
	uint8 argcount, j;
	char* chrPtr;
	char args[EP_ARG_COUNT_MAX][EP_ARG_SIZE];
	boolean commandDone = FALSE;
	boolean commandError = FALSE;
	ExternalPose externalPoseTemp;
	RadioMessage radioMessage;
	char* radioCmdDataPtr;

	//TODO Hack warning!
	// move past the command text, skip the 'EP'
	//TODO need structured argument parsing
	chrPtr = command + 2;
	while ((*chrPtr == ' ') || (*chrPtr == ',')) {
		chrPtr++; /* skip leading whitespace */
	};

	/* Loop through each command argument. */
	argcount = 0;
	j = 0;
	while (commandDone == FALSE) {
		/* Fill in each argument. */
		if((*chrPtr == '\r') || (*chrPtr == '\n') || (*chrPtr == '\0')) {
			// all done processing this line
			commandDone = TRUE;
			// null terminate the current arg
			args[argcount][j] = '\0';
			argcount++;
			if (argcount < EP_ARG_COUNT_NOTHETA) {
				// not enough args.  Stop processing
				error("external pose: not enough args");
				commandDone = TRUE;
				commandError = TRUE;
			}
		} else if (*chrPtr == ','){
			// end of current arg.  null terminate it...
			args[argcount][j] = '\0';
			// ... and point to the next one
			argcount++;
			j = 0;
			if (argcount >= EP_ARG_COUNT_WITHTHETA) {
				// too many args.  Stop processing
				error("external pose: too many args");
				commandDone = TRUE;
				commandError = TRUE;
			}
		} else {
			// advance the arg char ptr
			args[argcount][j] = *chrPtr;
			j++;
			if (j >= EP_ARG_SIZE) {
				// arg too big.  cancel all processing.
				error("external pose arg too large");
				commandDone = TRUE;
				commandError = TRUE;
			}
		}
		// get the next char
		chrPtr++;
	}

	if (!commandError) {
		// Convert each string in the atCommand struct to integers and store them.
		externalPoseTemp.ID = atoi(&args[0][0]);
		externalPoseTemp.pose.x = atoi(&args[1][0]);
		externalPoseTemp.pose.y = atoi(&args[2][0]);
		if (argcount == EP_ARG_COUNT_WITHTHETA) {
			externalPoseTemp.pose.theta = atoi(&args[3][0]);
		} else {
			externalPoseTemp.pose.theta = 0;
		}

		// xmit a radio message with the external pose
		radioCmdDataPtr = radioCommandGetDataPtr(&radioMessage);
		externalPoseMarshallRadioCmd(&externalPoseTemp, radioCmdDataPtr);
		radioCommandXmit(&externalPoseRadioCmd, ROBOT_ID_ALL, &radioMessage);
		isHost = TRUE;
		externalPoseCommandTimeStamp = osTaskGetTickCount();
	}
}



/*
 * @brief (Private) unmarshall the external pose data from the char array (change from 16bit to 32bit)
 * @param externalPosePtr the pose of a robot (overwritten)
 * @param  timeStamp time written onto this pose
 * @param radioCmdDataPtr message containing pose information to add
 *
 * @return void
 */
static void externalPoseUnmarshallRadioCmd(ExternalPose* externalPosePtr, uint32 timeStamp, char* radioCmdDataPtr) {
	// TODO make the host side match this
	externalPosePtr->ID = radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_ID_IDX];
	externalPosePtr->pose.x = *(int32*)(&radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_X_IDX]);
	externalPosePtr->pose.y = *(int32*)(&radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_Y_IDX]);
	externalPosePtr->pose.theta = *(int32*)(&radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_THETA_IDX]);
	externalPosePtr->timeStamp = timeStamp;
	externalPosePtr->active = TRUE;
}


/*
 * @brief (Private) callback to be used every time a radio message is received
 * @param radioCmdPtr the radio command
 * @param radioMsgPtr relevant data in this command
 *
 * @return void
 */
static void externalPoseRadioCallback(RadioCmd* radioCmdPtr, RadioMessage* radioMsgPtr) {
	uint8 robotID, i;
	char* radioCmdDataPtr;

	// see who this external pose message is for
	radioCmdDataPtr = radioCommandGetDataPtr(radioMsgPtr);
	robotID = radioCmdDataPtr[EXTERNAL_POSE_MESSAGE_ID_IDX];

	// is this message for me?
	if (robotID == roneID) {
		externalPoseUnmarshallRadioCmd(&externalPose, radioMsgPtr->command.timeStamp, radioCmdDataPtr);
		isActive = TRUE;
	} else {
		// check to see if this message if for a neighbor is on my nbr list
		for (i = 0; i < NEIGHBOR_MAX; ++i) {
			if (externalPoseNbrPoses[i].ID == robotID) {
				externalPoseUnmarshallRadioCmd(&externalPoseNbrPoses[i], radioMsgPtr->command.timeStamp, radioCmdDataPtr);
				break;
			}
		}
	}
}


/*
 * @brief (Private) clears pose information (usually called once a robot has not been seen for a long time)
 * @param extPosePtr pointer to the robot
 *
 * @return void
 */
static void externalPoseClear(ExternalPose* extPosePtr) {
	extPosePtr->ID = ROBOT_ID_NULL;
	extPosePtr->timeStamp = 0;
	extPosePtr->pose = poseOrigin;
	extPosePtr->active = FALSE;
}


/*
 * @brief (Private) update the external pose neighbor list with the actual neighbor list.
 * This is called once per neighbor cycle, after the new neighbors have been processed
 * It clears the external poses, then copies them back to the external pose structure
 * TODO: this is not mutex with the
 *
 * @param ndPtr incoming data
 * @return void
 */
static void externalPoseNbrCallback(NbrDatabase* ndPtr) {
	Nbr* nbrPtr;
	uint8 i, j;
	// a double buffer to make external poses mutex
	ExternalPose externalPoseNbrPosesTemp[NEIGHBOR_MAX];
	NbrList nbrList;
	boolean foundPose;

	// Copy the present neighbors Make a nbr list and check to see which external pose neighbors are still present
	nbrListCreate(&nbrList);
	for (i = 0; i < NEIGHBOR_MAX; ++i) {
		if (i < nbrListGetSize(&nbrList)) {
			nbrPtr = nbrListGetNbr(&nbrList, i);
			foundPose = FALSE;
			for (j = 0; j < NEIGHBOR_MAX; ++j) {
				if (nbrGetID(nbrPtr) == externalPoseNbrPoses[j].ID) {
					// this external pose is for an active neighbor.  keep it.
					externalPoseNbrPosesTemp[i] = externalPoseNbrPoses[j];
					foundPose = TRUE;
					break;
				}
			}
			if(!foundPose) {
				// no pose, but we have a neighbor. make an inactive poes with the correct id
				externalPoseClear(&externalPoseNbrPosesTemp[i]);
				externalPoseNbrPosesTemp[i].ID = nbrGetID(nbrPtr);
			}
		} else {
			// no more neighbors. clear the remaining external poses
			externalPoseClear(&externalPoseNbrPosesTemp[i]);
		}
		//i++;
	}

	// copy temp to actual poses
	for (i = 0; i < NEIGHBOR_MAX; ++i) {
		externalPoseNbrPoses[i] = externalPoseNbrPosesTemp[i];
	}

	uint32 tc = osTaskGetTickCount();
	uint32 ta = (externalPoseCommandTimeStamp + EXTERNAL_POSE_HOST_TIMEOUT);
	//if (osTaskGetTickCount() > (externalPoseCommandTimeStamp + EXTERNAL_POSE_HOST_TIMEOUT)) {
	if (tc > ta) {
		isHost = FALSE;
	}

	if (osTaskGetTickCount() > (externalPoseRadioCmd.lastTimeStamp + EXTERNAL_POSE_HOST_TIMEOUT)) {
		isActive = FALSE;
	}
}

// public functions --------------------------------------------------------

/*
 * @brief determines if this is "the Host" (the robot connected over serial to external system)
 * @return TRUE if this is the host or FALSE
 */
boolean externalPoseIsHost(void) {
	return isHost;
}


/*
 * @brief Is external pose information active?
 *
 * @returns TRUE if external pose is active or FALSE if not
 */
boolean externalPoseIsActive(void) {
	return isActive;
}


void externalPoseInit(void) {
	uint8 i;

	// clear all the external poses
	externalPoseClear(&externalPose);
	for (i = 0; i < NEIGHBOR_MAX; ++i) { //clear all neighbors
		externalPoseClear(&externalPoseNbrPoses[i]);
	}
	// init the callback
	radioCommandAddCallback(&externalPoseRadioCmd, "externalPoseMsg", externalPoseRadioCallback);

	// register EP = 'ExternalPose'
	serialCommandAdd(&serialCmdEP, "EP", externalPoseSerialCmdFunc);

	// sauce
	neighborsAddReceiveCallback(externalPoseNbrCallback);
}


boolean externalPoseGet(ExternalPose* poseArg) {
	boolean returnVal = FALSE;

	if(externalPoseIsActive()) {
		*poseArg = externalPose;
		returnVal = TRUE;
	}
	return returnVal;
}


/*
 * @brief Gets the pose of a given neighbor.
 *
 * @param poseArg (overwritten with external pose data if external pose is active and a neighbor)
 * @param nbrPtr robot we want to know the external pose of
 * @returns TRUE if external pose is active AND this neighbor exists
 */
boolean externalPoseGetNbr(ExternalPose* poseArg, Nbr* nbrPtr) {
	boolean returnVal = FALSE;
	uint8 i;

	if(externalPoseIsActive()) {
		for (i = 0; i < NEIGHBOR_MAX; ++i) {
			if ((externalPoseNbrPoses[i].ID == nbrGetID(nbrPtr)) && (externalPoseNbrPoses[i].active)) {
				*poseArg = externalPoseNbrPoses[i];
				returnVal = TRUE;
				break;
			}
		}
	}
	return returnVal;
}


/*
 * @brief Gets the distance to a given neighbor's external pose.
 *
 * @param nbrPtr for robot we want to know the distance from
 * @returns the distance to the given neighbor's external pose.
 */
int32 externalPoseGetNbrRange(Nbr* nbrPtr) {
	int32 d = 0;
	int32 dx, dy;
	ExternalPose poseExt;
	ExternalPose nbrPoseExt;

	if(nbrPtr) {
		if(externalPoseGet(&poseExt) && externalPoseGetNbr(&nbrPoseExt, nbrPtr)) {
			dx = poseExt.pose.x - nbrPoseExt.pose.x;
			dy = poseExt.pose.y - nbrPoseExt.pose.y;
			d = sqrtInt(dx*dx + dy*dy);
		}
	}
	return d;
}


boolean externalPoseGetRelativePose(Nbr *nbrPtr, NbrPose *nbrPose) {
	int32 theta;
	int32 dx, dy;
	ExternalPose poseExt;
	ExternalPose nbrPoseExt;

	if (nbrPtr) {
		if (externalPoseGet(&poseExt) && externalPoseGetNbr(&nbrPoseExt, nbrPtr)) {
			dx = poseExt.pose.x - nbrPoseExt.pose.x;
			dy = poseExt.pose.y - nbrPoseExt.pose.y;
			nbrPose->distance = sqrtInt(dx * dx + dy * dy);

			if (nbrPoseExt.pose.y > poseExt.pose.y) {
				dy = nbrPoseExt.pose.y - poseExt.pose.y;
				dx = poseExt.pose.x - nbrPoseExt.pose.x;
			} else {
				dy = poseExt.pose.y - nbrPoseExt.pose.y;
				dx =  nbrPoseExt.pose.x - poseExt.pose.x;
			}
			nbrPose->theta2 = atan2MilliRad(dy, dx);
			nbrPose->theta = normalizeAngleMilliRad2(nbrPose->theta2);


		//	nbrPose->bearing = normalizeAngleMilliRad(nbrPose->theta + poseExt.pose.theta) - MILLIRAD_PI;
			nbrPose->bearing = normalizeAngleMilliRad2(nbrPose->theta - poseExt.pose.theta);
			nbrPose->orientation = normalizeAngleMilliRad2(-MILLIRAD_PI + nbrPose->theta - nbrPoseExt.pose.theta);
			return TRUE;
		}
	}
	return FALSE;
}


