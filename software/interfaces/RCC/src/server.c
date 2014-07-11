/**
 * server.c
 *
 * Web server handlers and related functions
 */
#include "rcc.h"

char ipAddress[15];
int aprilTagConnected = 0;
int maxAprilTag = 0;
struct aprilTag aprilTagData[MAX_APRILTAG];

/**
 * Creates the server component of the RCC. Spawns listening thread and
 * worker threads.
 */
int createServer(int port)
{
	int *listenfd;		// The fd we are listening on
	WSADATA wsaData;	// Things for winsock setup
	WORD version;

	/* Initialize the Windows socket interface */
	version = MAKEWORD(2, 0);
	if (WSAStartup(version, &wsaData) != 0)
		Error("WSAStartup error");

	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 0) {
		WSACleanup();
		Error("WSAStartup error");
	}

	/* Get and store the local IP address */
	getIPAddress();

	listenfd = Malloc(sizeof(int));

	/* Open a port to listen on */
	if ((*listenfd = openListenFD(port)) < 0)
		return (-1);

	/* Create incoming connection handler */
	makeThread(&incomingHandler, listenfd);

	if (verbose)
		printf("T00: Server initialized on port %d\n", port);

	/* Success */
	return (0);
}

/**
 * Gets the local IP address and stores it as a string.
 */
void getIPAddress()
{
	int i;
	char name[80];
	struct hostent *hostname;
	struct in_addr addr;

	if (gethostname(name, sizeof(name)) == SOCKET_ERROR)
		return;

	/* Just use gethostbyname as we don't need to worry about reentrance */
	hostname = gethostbyname(name);
	if (hostname == 0)
		return;

	for (i = 0; hostname->h_addr_list[i] != 0; ++i) {
		memcpy(&addr, hostname->h_addr_list[i], sizeof(struct in_addr));
		strcpy(ipAddress, inet_ntoa(addr));
	}
}

/**
 * Opens and binds a port to listen for connections on
 */
SOCKET openListenFD(int port)
{
	SOCKET listenfd;
	int optval = 1;
	struct sockaddr_in serveraddr;

	/* Create a socket descriptor */
	if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		return (-1);

	/* Eliminates "Address already in use" error from bind */
	if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &optval,
		sizeof(int)) < 0)
		return (-1);

	memset((char *) &serveraddr, 0, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons((unsigned short) port);

	if (bind(listenfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0)
		return (-1);

	/* Make it a listening socket ready to accept connection requests */
	if (listen(listenfd, LISTENQ) < 0)
		return (-1);

	return (listenfd);
}

SOCKET openClientFD(char *hostname, char *port)
{
	SOCKET client = INVALID_SOCKET;
	struct addrinfo *result = NULL, *ptr = NULL;

	// Resolve the server address and port
	if (getaddrinfo(hostname, port, NULL, &result) != 0) {
		if (verbose)
			fprintf(stderr, "MAS: Bad AprilTag Address!\n");
		return (INVALID_SOCKET);
	}

	/* Attempt to connect to an address until one succeeds */
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
		client = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
		if (client == INVALID_SOCKET)
			Error("socket failure: %ld\n", WSAGetLastError());

		/* Connect to server. */
		if (connect(client, ptr->ai_addr,
			(int) ptr->ai_addrlen) == SOCKET_ERROR) {
			closesocket(client);
			client = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	return (client);
}

/**
 * Thread to handle all incoming connection requests
 */
void incomingHandler(void *vargp)
{
	int listenfd; 				// fd we are listening on
	int len;
	int count = 0;
	struct sockaddr addr;
	struct Connection *conn;	// Pointer to connection info

	/* Grab the open listening file descriptor from the args */
	listenfd = *((int *) vargp);
	Free(vargp);

	/* Continuously listen and accept connections */
	for (;;) {
		conn = Malloc(sizeof(struct Connection));

		if ((conn->fd = accept(listenfd, &addr, &len)) == INVALID_SOCKET) {
			Free(conn);
			continue;
		}

		if (verbose)
			printf("T00: New client connection, request number %d\n", count);

		conn->n = count++;

		/* Spawn a thread to handle the connection */
		makeThread(&connectionHandler, conn);
	}
}

/**
 * Thread to handle a connection, feeds data to the port
 */
void connectionHandler(void *vargp)
{
	int tid;						// Thread ID
	int id;							// Requested robot ID
	int n;
	int aid;						// AprilTag ID
	int head;						// Last location of robot buffer viewed
	int up, bl;						// Flag
	struct Connection *conn;		// Connection information
	char buffer[BUFFERSIZE];		// General use buffer
	char inBuffer[BUFFERSIZE];		// Input buffer from client
	struct socketIO socketio;		// Robust IO buffer for socket
	fd_set read_set, ready_set;		// Read set for select
	struct timeval tv = { 0, 1 };	// Timeout for select

	conn = (struct Connection *) vargp;
	tid = conn->n;

	if (verbose)
		printf("T%02d: Handler thread initialized\n", tid);

	/* Initialize robust IO on the socket */
	socketInitIO(&socketio, conn->fd);

	if (verbose)
		printf("T%02d: [%d] Processing new client connection\n", tid, conn->n);

	/* Query user for robot ID */
	id = -1;
	while (id == -1) {
		if (aprilTagConnected) {
			if (socketWrite(conn->fd,
				"Enter the robot ID you wish to view (0 for AprilTag only): ",
				59) < 0)
				break;
		} else {
			if (socketWrite(conn->fd, "Enter the robot ID you wish to view: ",
				37) < 0)
				break;
		}

		if (socketReadline(&socketio, buffer, BUFFERSIZE) == 0)
			break;

		if (sscanf(buffer, "%d\n", &id) != 1) {
			if (socketWrite(conn->fd, "Invalid ID! Try Again.\r\n", 24) < 0)
				break;
			continue;
		}

		/* Handle bad numbers */
		if (id >= MAXROBOTID || id < 0) {
			id = -1;
			if (socketWrite(conn->fd, "Invalid ID! Try Again.\r\n", 24) < 0)
				break;
			continue;
		}

		if (id == 0 && !aprilTagConnected) {
			id = -1;
			if (socketWrite(conn->fd, "Invalid ID! Try Again.\r\n", 24) < 0)
				break;
			continue;
		}

		mutexLock(&robots[id].mutex);
		up = robots[id].up;
		bl = robots[id].blacklisted && !(robots[id].type == REMOTE);
		mutexUnlock(&robots[id].mutex);

		if (up && !bl) {
			if (socketWrite(conn->fd, "Connected!\r\n", 12) < 0)
				break;
		} else {
			id = -1;
			if (socketWrite(conn->fd, "Robot ID not connected!\r\n", 25) < 0)
				break;
		}
	}

	/* Close if the connection broke */
	if (id == -1) {
		if (verbose)
			printf("T%02d: [%d] Done!\n", tid, conn->n);

		Close(conn->fd);
		Free(conn);
		return;
	}

	if (verbose)
		printf("T%02d: [%d] Connected to robot %02d\n", tid, conn->n, id);

	aid = robots[id].aid;
	if (aprilTagConnected) {
		while (aid == -1) {
			if (socketWrite(conn->fd, "Robot AprilTag (Enter for none): ", 33)
				< 0)
				break;

			if (socketReadline(&socketio, buffer, BUFFERSIZE) == 0)
				break;

			if (sscanf(buffer, "%d\n", &aid) != 1) {
				aid = -1;
				break;
			}

			/* Handle bad numbers */
			if (aid >= MAX_APRILTAG || aid < 0) {
				aid = -1;
				if (socketWrite(conn->fd, "Invalid AprilTag! Try Again.\r\n",
					30) < 0)
					break;
				continue;
			}

			if (aprilTagData[aid].active) {
				if (socketWrite(conn->fd, "AprilTag linked!\r\n", 18) < 0)
					break;

				if (verbose)
					printf("T%02d: [%d] Connected to AprilTag %d\n", tid,
						conn->n, aid);
			} else {
				if (socketWrite(conn->fd, "AprilTag not seen!\r\n", 20) < 0)
					break;
				aid = -1;
			}
		}
	}

	/* Connected to a robot / main AprilTag feed */
	if (id != 0 || (id == 0 && aid == -1)) {
		mutexLock(&robots[id].mutex);
		head = robots[id].head;
		mutexUnlock(&robots[id].mutex);

		/* Initialize stuff for select */
		FD_ZERO(&read_set);
		FD_SET(conn->fd, &read_set);

		for (;;) {
			ready_set = read_set;

			/* Check if there is data available from the client. */
			if (select(conn->fd + 1, &ready_set, NULL, NULL, &tv) < 0)
				break;

			/* Is there is data to be read? */
			if (FD_ISSET(conn->fd, &ready_set)) {
				if (socketRead(&socketio, inBuffer, BUFFERSIZE) != 0) {
					/* Write data out to serial port if the robot is local */
					mutexLock(&robots[id].mutex);
					if (robots[id].type == LOCAL || robots[id].type == HOST)
						hprintf(robots[id].hSerial, inBuffer);
					mutexUnlock(&robots[id].mutex);
				} else {
					break;
				}
			}
			/* Extensively use the mutex to prevent all data-races */
			mutexLock(&robots[id].mutex);
			/* If there is new data in the robot buffer */
			while (head != robots[id].head) {
				mutexUnlock(&robots[id].mutex);

				fetchData(buffer, id, head, aid, -1);

				if ((n = socketWrite(conn->fd, buffer, strlen(buffer))) < 0)
					break;

				head = (head + 1) % NUMBUFFER;
				mutexLock(&robots[id].mutex);
			}

			/* If true, connection is broken so exit out. */
			if (n < 0)
				break;

			/* Check if the robot is disconnected. */
			if ((robots[id].blacklisted
				&& !(robots[id].type == REMOTE))
				|| !robots[id].up) {
				socketWrite(conn->fd, "Robot ID disconnected!\r\n", 24);
				mutexUnlock(&robots[id].mutex);
				break;
			}
			mutexUnlock(&robots[id].mutex);
		}
	/* Connected to an AprilTag */
	} else {
		int rid;

		mutexLock(&aprilTagData[aid].mutex);
		head = aprilTagData[aid].head;
		mutexUnlock(&aprilTagData[aid].mutex);

		for (;;) {
			mutexLock(&aprilTagData[aid].mutex);
			/* If there is new data in the robot buffer */
			while (head != aprilTagData[aid].head) {
				rid = aprilTagData[aid].rid;
				mutexUnlock(&aprilTagData[aid].mutex);
				fetchData(buffer, rid, -1, aid, -1);

				if ((n = socketWrite(conn->fd, buffer, strlen(buffer))) < 0)
					break;

				head = (head + 1) % NUMBUFFER_APRILTAG;
				mutexLock(&aprilTagData[aid].mutex);
			}
			mutexUnlock(&aprilTagData[aid].mutex);

			/* If true, connection is broken so exit out. */
			if (n < 0)
				break;
		}
	}

	if (verbose)
		printf("T%02d: [%d] Done!\n", tid, conn->n);

	/* Clean up. */
	Close(conn->fd);
	Free(conn);

}

void initAprilTag()
{
	int i;
	for (i = 0; i < MAX_APRILTAG; i++) {
		aprilTagData[i].id = i;
		aprilTagData[i].up = 0;
		aprilTagData[i].rid = -1;
		aprilTagData[i].display = 0;
		aprilTagData[i].head = 0;
		aprilTagData[i].active = 0;
		mutexInit(&aprilTagData[i].mutex);
	}
}

int connectAprilTag()
{
	SOCKET fd;
	char *port, hostname[MAX_TEXTBOX_LENGTH];
	struct Connection *conn;

	strcpy(hostname, aprilTagURL.message);

	/* Parse out port from hostname */
	if ((port = strpbrk(hostname, ":")) != NULL)
		*(port++) = '\0';
	else
		port = "2001";

	if ((fd = openClientFD(hostname, port)) == INVALID_SOCKET)
		return (-1);

	conn = Malloc(sizeof(struct Connection));
	conn->fd = fd;
	conn->n = 0;

	aprilTagConnected = 1;
	robots[0].type = UNKNOWN;
	robots[0].up = 1;
	aprilTagURL.isActive = 0;
	makeThread(&aprilTagHandler, conn);

	return (0);
}

void aprilTagHandler(void *vargp)
{
	int i, id, rid, n, oldhead;
	int tid;						// Thread ID
	struct Connection *conn;		// Connection information
	struct socketIO socketio;		// Robust IO buffer for socket
	char buffer[APRILTAG_BUFFERSIZE], *bufp;
	char lbuffer[BUFFERSIZE + APRILTAG_BUFFERSIZE + 16];
	fd_set read_set, ready_set;		// Read set for select
	struct timeval tv = { 0, 1 };	// Timeout for select
	GLfloat x, y, t;				// Data from server
	int ts;							// Timestamp from server

	conn = (struct Connection *) vargp;
	tid = conn->n;

	if (verbose)
		printf("A%02d: Handler thread initialized\n", tid);

	/* Initialize robust IO on the socket */
	socketInitIO(&socketio, conn->fd);

	if (verbose)
		printf("A%02d: Processing April Tag Data\n", tid);

	/* Initialize stuff for select */
	FD_ZERO(&read_set);
	FD_SET(conn->fd, &read_set);

	for (;;) {
		memset(buffer, 0, APRILTAG_BUFFERSIZE);
		ready_set = read_set;

		/* Check if there is data available from the client. */
		if (select(conn->fd + 1, &ready_set, NULL, NULL, &tv) < 0)
			break;

		/* Is there is data to be read? */
		if (FD_ISSET(conn->fd, &ready_set)) {
			if ((n = socketReadline(&socketio, buffer, APRILTAG_BUFFERSIZE - 1))
				!= 0) {
				/* Get the beginning of the remote message */
				if ((bufp = strpbrk(buffer, " ")) == NULL) {
					bufp = buffer;
					continue;
				}
				/* Format end to be CRLF */
				while (buffer[n - 1] == '\r' || buffer[n - 1] == '\n') {
					buffer[n--] = '\0';
				}
				buffer[n] = '\r';
				buffer[n + 1] = '\n';
				insertBuffer(0, buffer, 0);

				/* Parse AprilTag ID */
				*(bufp++) = '\0';
				if (sscanf(buffer, "%d,", &id) < 1)
					continue;

				if (id < 0 || id >= MAX_APRILTAG)
					continue;

				if (id > maxAprilTag)
					maxAprilTag = id;

				mutexLock(&aprilTagData[id].mutex);
				aprilTagData[id].active = 1;
				strcpy(aprilTagData[id].buffer[aprilTagData[id].head], bufp);

				if (sscanf(bufp, "%f, %f, %f, %d", &x, &y, &t, &ts) == 4) {
					aprilTagData[id].x = x;
					aprilTagData[id].y = y;
					aprilTagData[id].t = t;

					if (x > 2 * aprilTagX)
						aprilTagX = x / 2. + 25;

					if (y > 2 * aprilTagY)
						aprilTagY = y / 2. + 25;
				}

				aprilTagData[id].up = clock();

				oldhead = aprilTagData[id].head;
				aprilTagData[id].head = (aprilTagData[id].head + 1)
					% NUMBUFFER_APRILTAG;

				if (aprilTagData[id].log) {
					if ((rid = aprilTagData[id].rid) != -1) {
						mutexUnlock(&aprilTagData[id].mutex);
						fetchData(lbuffer, rid, -1, id, -1);
						mutexLock(&aprilTagData[id].mutex);
						hprintf(&aprilTagData[id].logH, lbuffer);
					} else {
						hprintf(&aprilTagData[id].logH, "%11d, %s", clock(),
							aprilTagData[id].buffer[oldhead]);
					}
				}
				mutexUnlock(&aprilTagData[id].mutex);
			} else {
				break;
			}
		}
	}

	if (verbose)
		printf("A%02d: AprilTag connection closed!\n", tid);

	for (i = 0; i < MAX_APRILTAG; i++) {
		mutexLock(&aprilTagData[i].mutex);
		aprilTagData[i].active = 0;
		aprilTagData[i].up = 0;
		if (aprilTagData[i].log) {
			aprilTagData[i].log = 0;
			CloseHandle(aprilTagData[i].logH);
		}
		mutexUnlock(&aprilTagData[i].mutex);
	}

	/* Clean up */
	aprilTagConnected = 0;
	Close(conn->fd);
	Free(conn);
}

/**
 * Creates a combination of robot and AprilTag data
 */
int fetchData(char *buffer, int rid, int rhead, int aid, int ahead)
{
	int n = 2;
	char *bufp;

	if (rid != -1) {
		mutexLock(&robots[rid].mutex);
		if (rhead == -1) {
			rhead = ((robots[rid].head - 1) % NUMBUFFER
					+ NUMBUFFER) % NUMBUFFER;
		}

		n = sprintf(buffer, "%s", robots[rid].buffer[rhead]);
		mutexUnlock(&robots[rid].mutex);

		if (n < 0)
			return (-1);
	}

	if (aid != -1) {
		bufp = buffer + n - 2;

		mutexLock(&aprilTagData[aid].mutex);
		if (ahead == -1) {
			ahead = ((aprilTagData[aid].head - 1) % NUMBUFFER_APRILTAG
					+ NUMBUFFER_APRILTAG) % NUMBUFFER_APRILTAG;
		}

		if (rid != -1)
			n = sprintf(bufp, ", %s", aprilTagData[aid].buffer[ahead]);
		else
			n = sprintf(bufp, "%s", aprilTagData[aid].buffer[ahead]);
		mutexUnlock(&aprilTagData[aid].mutex);

		if (n < 0) {
			return (-1);
		/* If there was no data, rewrite the CRLF */
		} else if (n == 2) {
			if (sprintf(bufp, "\r\n") < 0)
				return (-1);
		}
	}

	return (0);
}

/**
 * Robustly write data to a socket. Adapted from CSAPP.
 */
ssize_t socketWrite(int fd, char *usrbuf, size_t n)
{
	size_t nleft = n;
	ssize_t nwritten;
	char *bufp = usrbuf;

	/* Write data until finished or error */
	while (nleft > 0) {
		if ((nwritten = send(fd, bufp, nleft, 0)) < 0)
			return (-1);

		nleft -= nwritten;
		bufp += nwritten;
	}

	/* Return bytes written */
	return (n);
}

/**
 * Robustly read data from a socket. Adapted from CSAPP.
 */
ssize_t socketRead(struct socketIO *sp, char *usrbuf, size_t n)
{
	int cnt;

	/* Refill the buffer if no data available */
	while (sp->count <= 0) {
		sp->count = recv(sp->fd, sp->buffer, sizeof(sp->buffer), 0);

		if (sp->count == 0) { /* EOF */
			return (0);
		} else {
			sp->bufp = sp->buffer; /* Reset buffer ptr */
		}
	}

	cnt = n;

	/* Copy data over to user buffer */
	if (sp->count < n)
		cnt = sp->count;

	memcpy(usrbuf, sp->bufp, cnt);
	sp->bufp += cnt;
	sp->count -= cnt;

	/* Return size */
	return (cnt);
}

/**
 * Initializes robust IO. Adapted from CSAPP.
 */
void socketInitIO(struct socketIO *sp, int fd)
{
	sp->fd = fd;
	sp->count = 0;
	sp->bufp = sp->buffer;
}

/**
 * Robustly reads a line delimited by '\n' into a buffer. Adapted from CSAPP.
 */
ssize_t socketReadline(struct socketIO *sp, char *usrbuf, size_t maxlen)
{
	int rc;
	unsigned int n;
	char c, *bufp = usrbuf;

	/* Read data until newline or error */
	for (n = 1; n < maxlen; n++) {
		if ((rc = socketRead(sp, &c, 1)) == 1) {
			*bufp++ = c;

			if (c == '\n') {
				n++;
				break;
			}
		} else if (rc == 0) {
			if (n == 1)
				return (0);
			else
				break;
		} else {
			return (-1);
		}
	}
	*bufp = 0;

	/* Return size */
	return (n - 1);
}

