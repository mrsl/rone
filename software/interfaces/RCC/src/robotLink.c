/**
 * robotLink.c
 *
 * Robot serial connection and data management
 */
#include "rcc.h"

struct commCon robots[MAXROBOTID]; /* Robot buffers */

/**
 * Initialize the robot buffers
 */
void initRobots()
{
	int i;

	/* Initialize values in the struct */
	for (i = 0; i < MAXROBOTID; i++) {
		robots[i].id = i;
		robots[i].blacklisted = 0;
		robots[i].hSerial = NULL;
		robots[i].up = 0;
		robots[i].head = 0;
		robots[i].type = UNKNOWN;
		mutexInit(&robots[i].mutex);
	}

	makeThread(&commManager, 0);
}

/**
 * Performs tasks on all robot links in intervals
 */
void commManager(void *vargp)
{
	int i;

	/* Get rid of annoying unused variable compiler errors */
	vargp = (void *) vargp;

	/* Run in detached mode */
	//Pthread_detach(pthread_self());
	/* Iterate through robot list indefinitely */
	for (;;) {
		for (i = 0; i < MAXROBOTID; i++) {
			mutexLock(&robots[i].mutex);

			/* If a remote robot has been inactive for a while, deactivate. */
			if (robots[i].up + GRACETIME < clock() && robots[i].type == REMOTE) {
				robots[i].up = 0;
				robots[i].head = 0;
			}

			/* Ping all host robots for updated remote robots. */
			if (robots[i].up && robots[i].type == HOST
				&& !robots[i].blacklisted)
				hprintf(robots[i].hSerial, "rt\n");

			mutexUnlock(&robots[i].mutex);
		}
		/* Sleep for a while. */
		Sleep(SLEEPTIME);
	}
}

/**
 * Spawn a thread to manage a serial connection
 */
int initCommCommander(int port)
{
	struct commInfo *info = Malloc(sizeof(struct commInfo));
	HANDLE *hSerial = Malloc(sizeof(HANDLE));

	/* Connect to the port */
	if (serialConnect(hSerial, port) < 0) {
		if (verbose)
			fprintf(stderr, "ERROR: Failed to connect serial\n");
		return (-1);
	}

	/* Fill in info struct and pass to thread */
	info->hSerial = hSerial;
	info->port = port;

	makeThread(&commCommander, info);

	return (0);
}

/**
 * Thread to manage a serial connection and input data into robot buffers
 */
void commCommander(void *vargp)
{
	int i, j, err;
	int id = 0;							// Robot ID
	int initialized = 0;				// Have we handshaked with the robot?
	int isHost = 0;						// Is this robot a rprintf host?
	int rid;							// Remote robot ID
	char buffer[BUFFERSIZE + 1];		// Buffers
	char rbuffer[BUFFERSIZE + 1];
	char sbuf[BUFFERSIZE + 1];
	char *bufp = buffer;				// Pointer to buffers
	char *sbufp;
	struct commInfo *info;				// Information on robot connection
	struct remoteRobots rr;				// Host info on remote robots
	struct serialIO sio;				// Robust IO on serial buffer

	/* Get info from argument */
	info = ((struct commInfo *) vargp);

	/* Query the robot for its id, and if it is a host */
	hprintf(info->hSerial, "rr\n");

	/* Initialize robust IO on the serial */
	serialInitIO(&sio, info->hSerial);

	/* Manage connection indefinitely */
	for (;;) {
		/* Read a line */
		if ((err = serialReadline(&sio, bufp, BUFFERSIZE)) < 0) {
			if (verbose)
				fprintf(stderr, "S%02d: Serial read error\n", id);
			break;
		} else if (err == 0) {
			continue;
		}

		/* Read in more data if we haven't completed a line */
		if (bufp[err - 1] != '\n') {
			bufp += err;
			continue;
		} else {
			bufp = buffer;
		}

		/* Loop until ID and host data has been obtained from the robot */
		if (!initialized) {
			/* Parse in hex data of reverse endianness from the robots */
			if (strncmp(buffer, "rr", 2) == 0) {
				sbufp = buffer + 3;
				/* Get the two arguments */
				for (i = 0; i < 2; i++) {
					j = 0;
					while (*sbufp != ',') {
						if (*sbufp == '\r' && *(sbufp + 1) == '\n') {
							sbuf[j] = '\0';
							break;
						} else if (j == SBUFSIZE) {
							sbuf[j] = '\0';
							break;
						}
						sbuf[j] = *sbufp;

						j++;
						sbufp++;
					}
					sbufp++;

					/* Convert hex number in buffer to usable values */
					if (i == 0)
						id = convertASCIIHexWord(sbuf);
					else
						isHost = convertASCIIHexWord(sbuf);
				}

				if (verbose)
					printf("S%02d: Connected to robot ID %02d\n", id, id);

				/* Initializing */
				initialized = 1;
				activateRobot(id, info, isHost);
				bufp = buffer;
			}
			continue;
		}
		/* Insert the read line into robot's buffer. */
		insertBuffer(id, buffer);

		err = 0;

		/* If we get a status line from a host robot. */
		if (strncmp(buffer, "rts", 3) == 0) {
			/* This robot is a host robot. */
			robots[id].type = HOST;
			isHost = 1;

			/* Get Number or robots */
			if (sscanf(buffer, "rts,%d%s", &rr.n, rbuffer) < 1)
				continue;

			/* Handle bad numbers */
			if (rr.n > MAXROBOTID || rr.n < 0)
				continue;

			/* Get IDs of connected robots */
			for (i = 0; i < rr.n; i++) {
				if (sscanf(rbuffer, ",%d%s", &rr.ids[i], rbuffer) < 1)
					break;

				if (rr.ids[i] > MAXROBOTID || rr.ids[i] < 0)
					break;

				rid = rr.ids[i];

				/* Handle bad numbers */
				if (rid > MAXROBOTID || rid < 0)
					break;

				mutexLock(&robots[rid].mutex);

				/* Ignore if this is already connected as a local robot */
				if (robots[rid].hSerial != NULL) {
					mutexUnlock(&robots[rid].mutex);
					continue;
				}

				/* Update robot */
				robots[rid].type = REMOTE;
				robots[rid].up = clock();
				robots[rid].host = id;

				mutexUnlock(&robots[rid].mutex);
			}
		}

		/* If we get a data line */
		if (strncmp(buffer, "rtd", 3) == 0) {
			/* Get the beginning of the remote message */
			if ((bufp = strpbrk(buffer, " ")) == NULL) {
				bufp = buffer;
				continue;
			}
			*bufp = "\0";

			/* Scan ID and data */
			if (sscanf(buffer, "rtd,%d", &rid) < 1)
				continue;

			/* If we aren't already connected via serial, put data in buffer. */
			if (robots[rid].hSerial == NULL) {
				/* Insert parsed line into remote robot's buffer. */
				insertBuffer(rid, bufp + 1);
				robots[rid].type = REMOTE;
				robots[rid].host = id;

				bufp = buffer;
			}
		}
	}
	robots[id].hSerial = NULL;

	/* If blacklisted, should break out of loop due to serial read error. */
	if (robots[id].blacklisted) {
		if (verbose)
			printf("S%02d: Blacklisted!\n", id);
	/* If we exited from something else */
	} else {
		if (verbose)
			printf("S%02d: Done!\n", id);

		/* Clean up */
		commToNum[info->port] = 0;
		robots[id].type = UNKNOWN;
		CloseHandle(*info->hSerial);
	}

	Free(info->hSerial);
	Free(info);
}

/**
 * Initialize a robot buffer
 */
void activateRobot(int robotID, struct commInfo *info, int isHost)
{
	mutexLock(&robots[robotID].mutex);

	commToNum[info->port] = robotID;
	robots[robotID].port = info->port;

	if (isHost)
		robots[robotID].type = HOST;
	else
		robots[robotID].type = LOCAL;

	robots[robotID].hSerial = info->hSerial;
	robots[robotID].up = clock();

	mutexUnlock(&robots[robotID].mutex);
}

/**
 * Insert a line in a robot's buffer
 */
void insertBuffer(int robotID, char *buffer)
{
	/* Lock the robot buffer */
	mutexLock(&robots[robotID].mutex);

	robots[robotID].up = clock();

	/* Add new message to rotating buffer */
	robots[robotID].head = (robots[robotID].head + 1) % NUMBUFFER;

	sprintf(robots[robotID].buffer[robots[robotID].head], "[%11ld] %s", clock(),
		buffer);

	/* Unlock the robot buffer */
	mutexUnlock(&robots[robotID].mutex);
}
