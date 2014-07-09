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
#define SLEEPTIME 3000
#define GRACETIME 10000

/* Robot states */
#define LOCAL 		0
#define REMOTE		1
#define HOST		2
#define UNKNOWN		3

/**
 * Connection information
 */
struct commInfo
{
	HANDLE *hSerial;	// Serial handle
	int port;			// COM port
};

/**
 * Robot data buffer and state information holder
 */
struct commCon
{
	int id; 									// Robot ID
	int aid;									// AprilTag ID
	int blacklisted;							// Is this robot ID blacklisted?
	int log;									// Log this robots data?
	HANDLE *hSerial;							// Serial handle if connected
	HANDLE logH;								// Logfile
	time_t up;									// Last time we saw the robot
	int type;									// Type of robot
	int port;									// If local, the COM port
	int host;									// If remote, the host robot
	char buffer[NUMBUFFER][BUFFERSIZE + APRILTAG_BUFFERSIZE + 16]; // buffer
	int head;									// Head of buffer
	CRITICAL_SECTION mutex;						// Mutex for this robot
};

extern struct commCon robots[MAXROBOTID];	// Robot data

void initRobots();
void commManager(void *vargp);
int initCommCommander(int port);
void commCommander(void *vargp);
void activateRobot(int robotID, struct commInfo *info);
void insertBuffer(int robotID, char *buffer);

#endif
