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

	CloseHandle((HANDLE)_beginthread(&commManager, 0, 0));
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
				fcprintf(robots[i].hSerial, "rt\n");

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

	CloseHandle((HANDLE)_beginthread(&commCommander, 0, info));

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
	char sbuf[SBUFSIZE];
	char *bufp = buffer;				// Pointer to buffers
	char *sbufp;
	struct commInfo *info;				// Information on robot connection
	struct remoteRobots *rr, *newRR;	// Host info on remote robots
	struct serialIO sio;				// Robust IO on serial buffer

	/* Get info from argument */
	info = ((struct commInfo *) vargp);

	/* Query the robot for its id, and if it is a host */
	fcprintf(info->hSerial, "rr\n");

	/* Initialize robust IO on the serial */
	serial_readinitb(&sio, info->hSerial);

	/* Manage connection indefinitely */
	for (;;) {
		/* Read a line */
		if ((err = serial_readlineb(&sio, bufp, BUFFERSIZE)) < 0) {
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
			if (buffer[0] == 'r' && buffer[1] == 'r') {
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
		if (buffer[0] == 'r' && buffer[1] == 't' && buffer[2] == 's') {
			/* This robot is a host robot. */
			robots[id].type = HOST;
			isHost = 1;

			newRR = Malloc(sizeof(struct remoteRobots));

			/* Get Number or robots */
			if (sscanf(buffer, "rts,%d%s", &newRR->n, rbuffer) < 1) {
				Free(newRR);
				continue;
			}

			/* Handle bad numbers */
			if (newRR->n > MAXROBOTID || newRR->n < 0) {
				Free(newRR);
				continue;
			}

			/* Get IDs of connected robots */
			for (i = 0; i < newRR->n; i++) {
				if (sscanf(rbuffer, ",%d%s", &newRR->ids[i], rbuffer) < 1) {
					err = 1;
					break;
				}
				if (newRR->ids[i] > MAXROBOTID || newRR->ids[i] < 0) {
					err = 1;
					break;
				}
			}

			/* Clean up and ignore if error */
			if (err) {
				Free(newRR);
				continue;
			}

			/* Mark each robot as active */
			for (i = 0; i < newRR->n; i++) {
				rid = newRR->ids[i];

				/* Handle bad numbers */
				if (rid > MAXROBOTID || rid < 0) {
					err = 1;
					break;
				}

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

			/* Clean up and ignore if error */
			if (err) {
				Free(newRR);
				continue;
			}

			/* Free old list */
			if (rr)
				Free(rr);

			rr = newRR;
		}

		/* If we get a data line */
		if (buffer[0] == 'r' && buffer[1] == 't' && buffer[2] == 'd') {
			/* Scan ID and data */
			if (sscanf(buffer, "rtd,%d", &rid) < 1)
				continue;

			/* If we aren't already connected via serial, put data in buffer. */
			if (robots[rid].hSerial == NULL) {
				/* Get the beginning of the remote message */
				if ((bufp = strpbrk(buffer, " ")) == NULL) {
					bufp = buffer;
					continue;
				}

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

	sprintf(robots[robotID].buffer[robots[robotID].head], "[%10ld] %s", clock(),
		buffer);

	/* Unlock the robot buffer */
	mutexUnlock(&robots[robotID].mutex);
}
