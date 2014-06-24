/**
 * robotLink.h
 */
#ifndef RLINK_H_
#define RLINK_H_

#define NUMBUFFER 64
#define MAXROBOTID 256
#define SBUFSIZE 256
#define SLEEPTIME 5000
#define GRACETIME 7000

#define LOCAL 	0
#define REMOTE	1
#define HOST	2
#define UNKNOWN	3

/* Contains info needed to be passed to the managing thread */
struct commInfo {
	HANDLE *hSerial;	/* Serial handle */
	int port;			/* Comm port */
};

/* Robot buffer */
struct commCon {
	int id;				/* Robot ID */
	HANDLE *hSerial;	/* Serial handle if connected via serial */
	time_t up;			/* Last time we confirmed this robot's existence */
	int type;			/* Type of robot */
	union {
		int port;		/* If local, the comm port */
		int host;		/* If remote, the host robot */
	};
	char buffer[NUMBUFFER][BUFFERSIZE + 13]; /* Rotating buffer array */
	int head;			/* Head of buffer */
	pthread_mutex_t mutex;
};

/* Data on the remote robots attached */
struct remoteRobots {
	int n;					/* Number of remote robots */
	int ids[MAXROBOTID];	/* IDs of remote robots */
};

/* Array of robot buffers */
extern struct commCon robots[MAXROBOTID];

void initRobots();
void *commManager(void *vargp);
int initCommCommander(int port);
void *commCommander(void *vargp);
void activateRobot(int robotID, struct commInfo *info, int isHost);
void insertBuffer(int robotID, char *buffer);

#endif
