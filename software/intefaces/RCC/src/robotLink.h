/**
 * robotLink.h
 */
#ifndef RLINK_H_
#define RLINK_H_

#define NUMBUFFER 64
#define MAXROBOTID 256

#define LOCAL 	0
#define REMOTE	1
#define HOST	2
#define UNKNOWN	3

/* Contains info needed to be passed to the managing thread */
struct commInfo {
	HANDLE *hSerial;
	int port;
};

/* Robot buffer */
struct commCon {
	int id;
	HANDLE *hSerial;
	time_t up;
	int type;
	union {
		int port;
		int host;
	};
	char buffer[NUMBUFFER][BUFFERSIZE + 1];
	int head;
	pthread_mutex_t mutex;
};

/* Data on the remote robots attached */
struct remoteRobots {
	int n;
	int ids[MAXROBOTID];
};

/* Array of robot buffers */
extern struct commCon robots[MAXROBOTID];

void initRobots();
void *commManager(void *vargp);
int initCommCommander(int port);
void *commCommander(void *vargp);
void activateRobot(int robotID, struct commInfo *info);
void insertBuffer(int robotID, char *buffer);

#endif
