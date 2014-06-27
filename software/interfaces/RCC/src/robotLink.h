/**
 * robotLink.h
 *
 * Header file for robotLink.c
 */
#ifndef RLINK_H_
#define RLINK_H_

#define NUMBUFFER 64
#define MAXROBOTID 256
#define SBUFSIZE 256
#define SLEEPTIME 1000
#define GRACETIME 5000

#define LOCAL 		0
#define REMOTE		1
#define HOST		2
#define UNKNOWN		3

/* Contains info needed to be passed to the managing thread */
struct commInfo
{
	HANDLE *hSerial;	// Serial handle
	int port;			// COM port
};

/* Robot buffer */
struct commCon
{
	int id; 									// Robot ID
	int blacklisted;							// Is this robot ID blacklisted?
	HANDLE *hSerial;							// Serial handle if connected
	time_t up;									// Last time we saw the robot
	int type;									// Type of robot
	union
	{
		int port;								// If local, the COM port
		int host;								// If remote, the host robot
	};
	char buffer[NUMBUFFER][BUFFERSIZE + 13];	// Rotating buffer array
	int head;									// Head of buffer
	CRITICAL_SECTION mutex;						// Mutex for this robot
};

/* Data on the remote robots attached */
struct remoteRobots
{
	int n;					// Number of remote robots
	int ids[MAXROBOTID];	// IDs of remote robots
};

/* Array of robot buffers */
extern struct commCon robots[MAXROBOTID];

void initRobots();
void commManager(void *vargp);
int initCommCommander(int port);
void commCommander(void *vargp);
void activateRobot(int robotID, struct commInfo *info, int isHost);
void insertBuffer(int robotID, char *buffer);

#endif
