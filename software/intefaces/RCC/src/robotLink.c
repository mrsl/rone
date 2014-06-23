/**
 * robotLink.c
 */

#include "rcc.h"

struct commCon robots[MAXROBOTID]; /* Robot buffers */

/**
 * Initialize the robot buffers
 */
void
initRobots()
{
	int i;
	pthread_t tid;

	/* Initialize values in the struct */
	for (i = 0; i < MAXROBOTID; i++) {
		robots[i].id = i;
		robots[i].hSerial = NULL;
		robots[i].up = 0;
		robots[i].head = 0;
		robots[i].type = UNKNOWN;
		Pthread_mutex_init(&robots[i].mutex, NULL);
	}

	Pthread_create(&tid, NULL, commManager, NULL);
}

void
*commManager(void *vargp)
{
	int i;

	vargp = (void *)vargp;

	Pthread_detach(pthread_self());

	for (;;) {
		for (i = 0; i < MAXROBOTID; i++) {
			Pthread_mutex_lock(&robots[i].mutex);

			if (robots[i].up + 5000 < clock() && robots[i].type == REMOTE) {
				robots[i].up = 0;
				robots[i].head = 0;
			}

			Pthread_mutex_unlock(&robots[i].mutex);
		}
		Sleep(5000);
	}

	return (NULL);
}

/**
 * Spawn a thread to manage a serial connection
 */
int
initCommCommander(int port)
{
	pthread_t tid;
	struct commInfo *info = Malloc(sizeof(struct commInfo));
	HANDLE *hSerial = Malloc(sizeof(HANDLE));

	/* Connect to the port */
	if (serialConnect(hSerial, port) < 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: Failed to connect serial\n");
		return (-1);
	}

	info->hSerial = hSerial;
	info->port = port;

	Pthread_create(&tid, NULL, commCommander, info);

	return (0);
}

/**
 * Thread to manage a serial connection and input data into robot buffers
 */
void
*commCommander(void *vargp)
{
	int i, j, err;
	int id = 0;
	int isHost = 0;
	int timer, ttimer;
	int rid;
	char buffer[BUFFERSIZE + 1], rbuffer[BUFFERSIZE + 1], sbuf[SBUFSIZE];
	char *bufp;
	struct commInfo *info;
	struct remoteRobots *rr, *newRR;
	serial_t sio;

	/* Run the thread as detached */
	Pthread_detach(pthread_self());

	/* Get struct from arg */
	info = ((struct commInfo *)vargp);

	/* Query the robot for its id, and if it is a host */
	fcprintf(info->hSerial, "rr\n");

	/* Initialize robust IO on the serial */
	serial_readinitb(&sio, info->hSerial);

	/* Loop until ID and host data has been obtained from the robot */
	for (;;) {
		buffer[BUFFERSIZE] = '\0';

		/* Read a line from the serial */
		if ((err = serial_readlineb(&sio, buffer, BUFFERSIZE)) < 0) {
			if (VERBOSE)
			fprintf(stderr, "ERROR: Serial read error\n");
			Free(info->hSerial);
			Free(info);
			return (NULL);
		} else if (err == 0) {
			continue;
		}

		/* Get ID and host status if correct line */
		if (buffer[0] == 'r' && buffer[1] == 'r') {
			bufp = buffer + 3;
			for (i = 0; i < 2; i++) {
				j = 0;
				while (*bufp != ',') {
					if (*bufp == '\r' && *(bufp + 1) == '\n') {
						sbuf[j] = '\0';
						break;
					} else if (j == SBUFSIZE) {
						sbuf[j] = '\0';
						break;
					}
					sbuf[j] = *bufp;

					j++;
					bufp++;
				}
				bufp++; /* Skip comma */
				if (i == 0)
					id = convertASCIIHexWord(sbuf);
				else
					isHost = convertASCIIHexWord(sbuf);
			}
			break;
		}
	}

	if (VERBOSE)
	printf("S%02d: Connected to robot ID %02d\n", id, id);

	/* Initializing */
	activateRobot(id, info);

	/* If the robot is a host */
	if (isHost) {
		timer = clock();
		robots[id].type = HOST;
		for (;;) {
			buffer[BUFFERSIZE] = '\0';

			/* Every so many seconds, query again for status */
			ttimer = clock();
			if (timer + 3000 < ttimer) {
				fcprintf(info->hSerial, "rt\n");
				timer = ttimer;
			}

			/* Read a line */
			if ((err = serial_readlineb(&sio, buffer, BUFFERSIZE)) < 0) {
				if (VERBOSE)
				fprintf(stderr, "S%02d: Serial read error\n", id);
				break;
			} else if (err == 0) {
				continue;
			}

			err = 0;

			/* Insert the line into the host robot's buffer */
			insertBuffer(id, buffer);

			/* If we get a status line */
			if (buffer[0] == 'r' && buffer[1] == 't' && buffer[2] == 's') {
				newRR = Malloc(sizeof(struct remoteRobots));

				/* Get Number or robots */
				if (sscanf(buffer, "rts,%d%s", &newRR->n, rbuffer) < 1) {
					Free(newRR);
					continue;
				}

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

				if (err) {
					Free(newRR);
					continue;
				}

				/* Mark each robot as active */
				for (i = 0; i < newRR->n; i++) {
					rid = newRR->ids[i];

					if (rid > MAXROBOTID || rid < 0) {
						err = 1;
						break;
					}

					Pthread_mutex_lock(&robots[rid].mutex);
					if (robots[rid].hSerial != NULL) {
						Pthread_mutex_unlock(&robots[rid].mutex);
						continue;
					}

					robots[rid].type = REMOTE;
					robots[rid].up = clock();
					robots[rid].host = id;

					Pthread_mutex_unlock(&robots[rid].mutex);
				}

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
				if (sscanf(buffer, "rtd,%d %s", &rid, rbuffer) < 1) {
					continue;
				}

				sprintf(buffer, "%s\n", rbuffer);

				/* If we aren't already connected via serial, put data in
				 * the robot buffer */
				if (robots[rid].hSerial == NULL) {
					insertBuffer(rid, buffer);
					robots[rid].host = id;
				}
			}
		}

	/* If it is just a regular robot */
	} else {
		/* Loop while still connected */
		for (;;) {
			buffer[BUFFERSIZE] = '\0';

			/* Read a line from the serial */
			if ((err = serial_readlineb(&sio, buffer, BUFFERSIZE)) < 0) {
				if (VERBOSE)
				fprintf(stderr, "S%02d: Serial read error\n", id);
				break;
			} else if (err == 0) {
				continue;
			}


			/* Insert the line into the robot buffer */
			insertBuffer(id, buffer);
		}
	}

	if (VERBOSE)
	printf("S%02d: Done!\n", id);

	/* Clean up */
	commToNum[info->port] = 0;
	robots[id].up = 0;
	robots[id].hSerial = NULL;
	CloseHandle(*info->hSerial);
	Free(info->hSerial);
	Free(info);

	return (NULL);
}

/**
 * Initialize a robot buffer
 */
void
activateRobot(int robotID, struct commInfo *info)
{
	Pthread_mutex_lock(&robots[robotID].mutex);

	commToNum[info->port] = robotID;
	robots[robotID].port = info->port;
	robots[robotID].type = LOCAL;
	robots[robotID].hSerial = info->hSerial;
	robots[robotID].up = clock();

	Pthread_mutex_unlock(&robots[robotID].mutex);
}

/**
 * Insert a line in a robot's buffer
 */
void
insertBuffer(int robotID, char *buffer)
{
	int i;

	/* Lock the robot buffer */
	Pthread_mutex_lock(&robots[robotID].mutex);

	robots[robotID].up = clock();

	/* Add new message to rotating buffer */
	robots[robotID].head = (robots[robotID].head + 1) % NUMBUFFER;

	for (i = 0; i < BUFFERSIZE; i++) {
		robots[robotID].buffer[robots[robotID].head][i] = buffer[i];

		if (robots[robotID].buffer[robots[robotID].head][i] == '\n') {
			robots[robotID].buffer[robots[robotID].head][i] = '\r';
			robots[robotID].buffer[robots[robotID].head][i + 1] = '\n';
			robots[robotID].buffer[robots[robotID].head][i + 2] = '\0';
			break;
		}
	}

	/* Unlock the robot buffer */
	Pthread_mutex_unlock(&robots[robotID].mutex);
}
