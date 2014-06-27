/**
 * server.c
 *
 * Web server handlers and related functions
 */
#include "rcc.h"

char ipAddress[15];

/**
 * Creates the server component of the RCC. Spawns listening thread and
 * worker threads.
 */
int createServer(int port)
{
	int listenfd;		// The fd we are listening on
	int *ti;
	WSADATA wsaData;	// Things for winsock setup
	WORD version;

	if (verbose)
		printf("MAS: Thread buffer initialized\n");

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

	/* Open a port to listen on */
	if ((listenfd = openListenFD(port)) < 0)
		return (-1);

	/* Create incoming connection handler */
	ti = Malloc(sizeof(int));
	*ti = listenfd;
	CloseHandle((HANDLE)_beginthread(&incomingHandler, 0, ti));

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
int openListenFD(int port)
{
	int listenfd, optval = 1;
	struct sockaddr_in serveraddr;

	/* Create a socket descriptor */
	if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
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

	return listenfd;
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

		conn->n = count;

		/* Spawn a thread to handle the connection */
		CloseHandle((HANDLE)_beginthread(&connectionHandler, 0, conn));

		count++;
	}
}

/**
 * Thread to handle a connection, feeds data to the port
 */
void connectionHandler(void *vargp)
{
	int tid;						// Thread ID
	int id;							// Requested robot ID
	int head;						// Last location of robot buffer viewed
	int err, bl;					// Flag
	clock_t timer;					// Event timer
	struct Connection *conn;		// Connection information
	char buffer[BUFFERSIZE];		// General use buffer
	struct socketIO socketio;		// Robust IO buffer for socket
	fd_set read_set, ready_set;		// Read set for select
	struct timeval tv = { 0, 1 };	// Timeout for select
	char inBuffer[BUFFERSIZE];		// Input buffer from client

	conn = (struct Connection *) vargp;
	tid = conn->n;

	if (verbose)
		printf("T%02d: Handler thread initialized\n", tid);

	/* Initialize robust IO on the socket */
	socketInitIO(&socketio, conn->fd);

	if (verbose)
		printf("T%02d: [%d] Processing new client connection\n", tid, conn->n);

	/* Query user for robot ID */
	id = 0;
	while (id == 0) {
		if (socketWrite(conn->fd, "Enter the robot ID you wish to view: ", 37)
			< 0)
			break;

		if (socketReadline(&socketio, buffer, BUFFERSIZE) == 0)
			break;

		if (sscanf(buffer, "%d\n", &id) != 1) {
			if (socketWrite(conn->fd, "Invalid ID! Try Again.\r\n", 24) < 0)
				break;
			continue;
		}

		/* Handle bad numbers */
		if (id >= MAXROBOTID || id < 0) {
			id = 0;
			if (socketWrite(conn->fd, "Invalid ID! Try Again.\r\n", 24) < 0)
				break;
			continue;
		}

		mutexLock(&robots[id].mutex);
		err = robots[id].up;
		bl = robots[id].blacklisted;
		mutexUnlock(&robots[id].mutex);

		if (err && !bl) {
			if (socketWrite(conn->fd, "Connected!\r\n", 12) < 0)
				break;
		} else {
			id = 0;
			if (socketWrite(conn->fd, "Robot ID not connected!\r\n", 25) < 0)
				break;
		}
	}

	/* Close if the connection broke */
	if (id == 0) {
		Close(conn->fd);
		Free(conn);
		return;
	}

	if (verbose)
		printf("T%02d: [%d] Connected to robot %02d\n", tid, conn->n, id);

	/* Feed new data from the robot to the client until close */
	head = robots[id].head;
	timer = clock();

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

		/* If there is new data in the robot buffer */
		while (head != robots[id].head) {
			mutexLock(&robots[id].mutex);
			if (sprintf(buffer, "%s", robots[id].buffer[head]) < 0)
				break;
			mutexUnlock(&robots[id].mutex);

			if ((err = socketWrite(conn->fd, buffer, strlen(buffer))) < 0)
				break;

			head = (head + 1) % NUMBUFFER;
		}
		if (err < 0)
			break;

		/* Check if the robot is actually disconnected. */
		if (timer + 1000 < clock()) {
			mutexLock(&robots[id].mutex);
			err = robots[id].up;
			mutexUnlock(&robots[id].mutex);

			if (!err) {
				socketWrite(conn->fd, "Robot ID disconnected!\r\n", 24);
				break;
			}

			timer = 0;
		}

		if (robots[id].blacklisted) {
			socketWrite(conn->fd, "Robot ID blacklisted!\r\n", 23);
			break;
		}
	}

	if (verbose)
		printf("T%02d: [%d] Done!\n", tid, conn->n);

	/* Clean up */
	Close(conn->fd);
	Free(conn);
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

