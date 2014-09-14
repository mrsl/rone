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
#define GRACETIME 15000

/* Robot states */
#define LOCAL 		0
#define REMOTE		1
#define HOST		2
#define DISTANT		3
#define SATELLITE	4
#define UNKNOWN		5

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
	int port;									// If local, the COM port
	int host;									// If remote, the host robot
	int blacklisted;							// Is this robot ID blacklisted?
	int log;									// Log this robots data?
	HANDLE *hSerial;							// Serial handle if connected
	HANDLE logH;								// Logfile
	long up;									// Last time we saw the robot
	long lup;									// Prev time we saw the robot
	int type;									// Type of robot
	int subnet;									// Radio subnet of robot
	int display;								// Show extra data?
	char buffer[NUMBUFFER][BUFFERSIZE + APRILTAG_BUFFERSIZE + 16]; // buffer
	float bps[NUMBUFFER]; 						// transfer speeds buffer
	int head;									// Head of buffer
	int count;									// Items in buffer
	GLfloat xP;
	GLfloat yP;
	long upP;
	CRITICAL_SECTION mutex;						// Mutex for this robot
};

extern struct commCon robots[MAXROBOTID];	// Robot data
extern int timestamps;						// Use timestamps?
extern int logging;							// Log data?
extern int hostData;						// Filter host robot data?

extern int ATsatID;

void initRobots();
void commManager(void *vargp);
int initCommCommander(int port);
void commCommander(void *vargp);
void activateRobot(int robotID, struct commInfo *info);
void insertBuffer(int robotID, char *buffer, int extraBytes);

#endif
