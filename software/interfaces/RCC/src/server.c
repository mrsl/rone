/**
 * server.c
 *
 * Web server handlers and related functions
 */
#include "rcc.h"

char ipAddress[15];
struct Buffer connectionBuffer;

/**
 * Creates the server component of the RCC. Spawns listening thread and
 * worker threads.
 */
int
createServer(int port)
{
	int listenfd;		/* The fd we are listening on */
	int i, *ti;
	WSADATA wsaData;	/* Things for winsock setup */
	WORD version;
	pthread_t tid;		/* Thread ID */

	/* Initialize the accepted connection buffer */
	buffer_init(&connectionBuffer, CONNECTIONBUFFERSIZE);

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
	if((listenfd = openListenFD(port)) < 0)
		return (-1);

	/* Create incoming connection handler */
	ti = Malloc(sizeof(int));
	*ti = listenfd;
	Pthread_create(&tid, NULL, incomingHandler, ti);

	if (verbose)
	printf("T00: Server initialized on port %d\n", port);

	/* Create worker threads */
	for (i = 1; i <= NTHREADS; i++) {
		/* Malloc to prevent data races */
		ti = Malloc(sizeof(int));
		*ti = i;
		Pthread_create(&tid, NULL, connectionHandler, ti);
	}

	/* Success */
	return (0);
}

/**
 * Gets the local IP address and stores it as a string.
 */
void
getIPAddress()
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
int
openListenFD(int port) 
{
	int listenfd, optval = 1;
	struct sockaddr_in serveraddr;

	/* Create a socket descriptor */
	if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
		return (-1);
 
	/* Eliminates "Address already in use" error from bind */
	if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, 
	    (const void *)&optval , sizeof(int)) < 0)
		return (-1);

	memset((char *) &serveraddr, 0, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET; 
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY); 
	serveraddr.sin_port = htons((unsigned short)port); 

	if (bind(listenfd, (struct sockaddr *)&serveraddr,
		sizeof(serveraddr)) < 0)
		return (-1);

	/* Make it a listening socket ready to accept connection requests */
	if (listen(listenfd, LISTENQ) < 0)
		return (-1);

	return listenfd;
}

/**
 * Thread to handle all incoming connection requests
 */
void
*incomingHandler(void *vargp)
{
	int listenfd;				/* FD we are listening on */
	int len;
	int count = 0;
	struct sockaddr addr;
	struct Connection *conn;	/* Pointer to connection info */

	/* Grab the open listening file descriptor from the args */
	listenfd = *((int *)vargp);
	Free(vargp);

	/* Run thread as detached */
	Pthread_detach(pthread_self());

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

		/* Put the connection in the buffer */
		buffer_put(&connectionBuffer, (void *)conn);
		count++;
	}

	return (NULL);
}

/**
 * Thread to handle a connection, feeds data to the port
 */
void
*connectionHandler(void *vargp) 
{
	int tid;					/* Thread ID */
	int id;						/* Requested robot ID */
	int head;					/* Last location of robot buffer viewed */
	int err;					/* Flag */
	clock_t timer;				/* Event timer */
	struct Connection *conn;	/* Connection information */
	char buffer[BUFFERSIZE];	/* General use buffer */
	struct socketIO socketio;	/* Robust IO buffer for socket */
	fd_set read_set, ready_set;	/* Read set for select */
	struct timeval tv = {0, 1};	/* Timeout for select */
	char inBuffer[BUFFERSIZE];	/* Input buffer from client */

	/* Grab thread ID from argument */
	tid = *((int *)vargp);
	Free(vargp);

	/* Run thread as detached */
	Pthread_detach(pthread_self());

	if (verbose)
	printf("T%02d: Handler thread initialized\n", tid);

	/* Continuously wait for a new connection in the buffer and then handle */
	for (;;) {
		/* Grab a new connection */
		conn = (struct Connection *)buffer_pop(&connectionBuffer);

		/* Initialize robust IO on the socket */
		socket_readinitb(&socketio, conn->fd);

		if (verbose)
		printf("T%02d: [%d] Processing new client connection\n", tid, conn->n);

		/* Query user for robot ID */
		id = 0;
		while (id == 0) {
			if (socket_writen(conn->fd,
				"Enter the robot ID you wish to view: ", 37) < 0)
				break;

			if (socket_readlineb(&socketio, buffer, BUFFERSIZE) == 0)
				break;

			if (sscanf(buffer, "%d\n", &id) != 1) {
				if (socket_writen(conn->fd,
					"Invalid ID! Try Again.\r\n", 24) < 0)
					break;
				continue;
			}

			/* Handle bad numbers */
			if (id >= MAXROBOTID || id < 0) {
				id = 0;
				if (socket_writen(conn->fd,
					"Invalid ID! Try Again.\r\n", 24) < 0)
					break;
				continue;
			}

			Pthread_mutex_lock(&robots[id].mutex);
			err = robots[id].up;
			Pthread_mutex_unlock(&robots[id].mutex);

			if (err) {
				if (socket_writen(conn->fd, "Connected!\r\n", 12) < 0)
					break;
			} else {
				id = 0;
				if (socket_writen(conn->fd,
					"Robot ID not connected!\r\n", 25) < 0)
					break;
			}
		}

		/* Close if the connection broke */
		if (id == 0) {
			Close(conn->fd);
			Free(conn);
			continue;
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
				if (socket_read(&socketio, inBuffer, BUFFERSIZE) != 0) {
					/* Write data out to serial port if the robot is local */
					Pthread_mutex_lock(&robots[id].mutex);
					if(robots[id].type == LOCAL || robots[id].type == HOST)
						fcprintf(robots[id].hSerial, inBuffer);

					Pthread_mutex_unlock(&robots[id].mutex);
				} else {
					break;
				}
			}

			/* If there is new data in the robot buffer */
			while (head != robots[id].head) {
				Pthread_mutex_lock(&robots[id].mutex);
				if (sprintf(buffer, "%s", robots[id].buffer[head]) < 0)
					break;
				Pthread_mutex_unlock(&robots[id].mutex);

				if ((err = socket_writen(conn->fd,
					buffer, strlen(buffer))) < 0)
					break;

				head = (head + 1) % NUMBUFFER;
			}
			if (err < 0)
				break;

			/* Check if the robot is actually disconnected. */
			if (timer + 1000 < clock()) {
				Pthread_mutex_lock(&robots[id].mutex);
				err = robots[id].up;
				Pthread_mutex_unlock(&robots[id].mutex);

				if (!err) {
					socket_writen(conn->fd, "Robot ID disconnected!\r\n", 24);
					break;
				}

				timer = 0;
			}
		}

		if (verbose)
		printf("T%02d: [%d] Done!\n", tid, conn->n);

		/* Clean up */
		Close(conn->fd);
		Free(conn);
	}

	return (NULL);
}

/**
 * Initialize the thread connection buffer
 */
void
buffer_init(struct Buffer *buf, int n)
{
	buf->array = Calloc(n, sizeof(void *));
	buf->put_index = 0;
	buf->pop_index = 0;
	buf->count = 0;
	buf->size = n;

	Pthread_mutex_init(&buf->mutex, NULL);
	Pthread_cond_init(&buf->empty, NULL);
	Pthread_cond_init(&buf->full, NULL);

}

/**
 * Put a pointer into the buffer
 */
void
buffer_put(struct Buffer *buf, void *conn)
{
	Pthread_mutex_lock(&buf->mutex);	/* Lock the buffer. */

	/* Wait until the buffer has space to begin filling. */
	while (buf->count == buf->size)
		Pthread_cond_wait(&buf->empty, &buf->mutex);

	/* Put the pointer in the array and update put position and count. */
	buf->array[buf->put_index] = conn;
	buf->put_index = (buf->put_index + 1) % buf->size;
	buf->count = buf->count + 1;

	/* Broadcast that there is something in the buffer. */
	Pthread_cond_broadcast(&buf->full);

	Pthread_mutex_unlock(&buf->mutex);	/* Unlock the buffer. */
}

/**
 * Pop a pointer from the buffer
 */
void
*buffer_pop(struct Buffer *buf)
{
	struct Connection *conn;

	Pthread_mutex_lock(&buf->mutex);	/* Lock the buffer. */

	/* Wait until something is in the buffer. */
	while (buf->count == 0)
		Pthread_cond_wait(&buf->full, &buf->mutex);

	/* Pop a pointer from the buffer and update read position and count. */
	conn = buf->array[buf->pop_index];
	buf->pop_index = (buf->pop_index + 1) % buf->size;
	buf->count = buf->count - 1;

	/* Signal that there is now room to fill in the buffer. */
	if (buf->count == 0)
		Pthread_cond_signal(&buf->empty);

	Pthread_mutex_unlock(&buf->mutex);	/* Unlock the buffer. */

	return (conn);
}

/**
 * Robustly write data to a socket. Adapted from CSAPP.
 */
ssize_t
socket_writen(int fd, char *usrbuf, size_t n)
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
ssize_t
socket_read(struct socketIO *sp, char *usrbuf, size_t n)
{
	int cnt;

	/* Refill the buffer if no data available */
	while (sp->count <= 0) {
		sp->count = recv(sp->fd, sp->buffer, sizeof(sp->buffer), 0);

		if (sp->count == 0) {	/* EOF */
			return (0);
		} else {
			sp->bufp = sp->buffer;	/* Reset buffer ptr */
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
void
socket_readinitb(struct socketIO *sp, int fd)
{
	sp->fd = fd;
	sp->count = 0;
	sp->bufp = sp->buffer;
}

/**
 * Robustly reads a line delimited by '\n' into a buffer. Adapted from CSAPP.
 */
ssize_t
socket_readlineb(struct socketIO *sp, char *usrbuf, size_t maxlen)
{
	int rc;
	unsigned int n;
	char c, *bufp = usrbuf;

	/* Read data until newline or error */
	for (n = 1; n < maxlen; n++) {
		if ((rc = socket_read(sp, &c, 1)) == 1) {
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

