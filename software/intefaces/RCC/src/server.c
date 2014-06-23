#include "rcc.h"

char ipAddress[15];
struct Buffer connectionBuffer;

/**
 * Creates the webserver component of the RCC. Spawns listening thread and
 * worker threads.
 */
int
createServer(int port)
{
	int listenfd, i, *ti;
	WSADATA wsaData;
	WORD version;
	pthread_t tid;

	/* Initialize the accepted connection buffer */
	buffer_init(&connectionBuffer, CONNECTIONBUFFERSIZE);

	if (VERBOSE)
	printf("MAS: Thread buffer initialized\n");

	/* Initialize the Windows socket interface */
	version = MAKEWORD(2, 0);
	if (WSAStartup(version, &wsaData) != 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: WSAStartup error\n");
		exit (-1);
	}

	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 0) {
		WSACleanup();
		if (VERBOSE)
		fprintf(stderr, "ERROR: WSAStartup error\n");
		exit (-1);
	}

	getIPAddress();

	/* Open a port to listen on */
	if((listenfd = open_listenfd(port)) < 0)
		return (-1);

	/* Create incoming connection handler */
	Pthread_create(&tid, NULL, incomingHandler, &listenfd);

	if (VERBOSE)
	printf("T00: Server initialized on port %d\n", port);

	/* Create worker threads */
	for (i = 1; i <= NTHREADS; i++) {
		ti = Malloc(sizeof(int));
		*ti = i;
		Pthread_create(&tid, NULL, connectionHandler, ti);
	}

	return (0);
}

void
getIPAddress()
{
	int i;
    char ac[80];
    struct hostent *phe;
    struct in_addr addr;

    if (gethostname(ac, sizeof(ac)) == SOCKET_ERROR)
        return;

    phe = gethostbyname(ac);
    if (phe == 0)
        return;

    for (i = 0; phe->h_addr_list[i] != 0; ++i) {
        memcpy(&addr, phe->h_addr_list[i], sizeof(struct in_addr));
        strcpy(ipAddress, inet_ntoa(addr));
    }
}

/**
 * Opens and binds a port to listen for connections on
 */
int
open_listenfd(int port) 
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
	int listenfd;
	int len;
	int ccount = 0;
	struct sockaddr addr;
	struct Connection *conn;

	/* Grab the open listening file descriptor from the args */
	listenfd = *((int *)vargp);

	/* Run thread as detached */
	Pthread_detach(pthread_self());

	/* Continuously listen and accept connections */
	for (;;) {
		conn = Malloc(sizeof(struct Connection));

		if ((conn->fd = accept(listenfd, &addr, &len)) == INVALID_SOCKET) {
			Free(conn);
			continue;
		}

		if (VERBOSE)
		printf("T00: New client connection, request number %d\n", ccount);

		conn->n = ccount;

		/* Put the connection in the buffer */
		buffer_put(&connectionBuffer, (void *)conn);
		ccount++;
	}

	return (NULL);
}

/**
 * Thread to handle a connection, feeds data to the port
 */
void
*connectionHandler(void *vargp) 
{
	int tid;
	int id;
	int head;
	int err;
	time_t timer;
	struct Connection *conn;
	char buffer[BUFFERSIZE];
	socketio_t socketio;

	/* Grab thread ID from arg */
	tid = *((int *)vargp);
	Free(vargp);

	/* Run thread as detached */
	Pthread_detach(pthread_self());

	if (VERBOSE)
	printf("T%02d: Handler thread initialized\n", tid);

	/* Continuously wait for a new connection in the buffer and then handle */
	for (;;) {
		/* Grab a new connection */
		conn = (struct Connection *)buffer_pop(&connectionBuffer);

		/* Initialize robust IO on the socket */
		socket_readinitb(&socketio, conn->fd);

		if (VERBOSE)
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

		if (VERBOSE)
		printf("T%02d: [%d] Connected to robot %02d\n", tid, conn->n, id);

		/* Feed new data from the robot to the client until close */
		head = robots[id].head;
		timer = clock();

		int n;
		char inBuffer[BUFFERSIZE];
		char *bufp;
		fd_set read_set, ready_set;
		fd_set write_set;

		FD_ZERO(&read_set);
		FD_SET(conn->fd, &read_set);

		struct timeval tv = {0, 5};

		bufp = inBuffer;

		for (;;) {
			ready_set = read_set;

			if (select(conn->fd + 1, &ready_set, &write_set, NULL, &tv) < 0)
				break;

			/* If there is data to be read. */
			if (FD_ISSET(conn->fd, &ready_set)) {
				if ((n = socket_read(&socketio, bufp, BUFFERSIZE)) != 0) {
					bufp += n;
					Pthread_mutex_lock(&robots[id].mutex);
					if(robots[id].type == LOCAL || robots[id].type == HOST) {
						fcprintf(robots[id].hSerial, inBuffer);
						bufp = inBuffer;
						n = 0;
					}
					Pthread_mutex_unlock(&robots[id].mutex);

				} else
					break;
			}

			/* If there is new data in the robot buffer */
			while (head != robots[id].head) {
				Pthread_mutex_lock(&robots[id].mutex);
				sprintf(buffer, "[%ld] %s", clock(), robots[id].buffer[head]);
				Pthread_mutex_unlock(&robots[id].mutex);

				if ((err = socket_writen(conn->fd,
					buffer, strlen(buffer))) < 0)
					break;

				head = (head + 1) % NUMBUFFER;
			}
			if (err < 0)
				break;

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

		if (VERBOSE)
		printf("T%02d: [%d] Done!\n", tid, conn->n);

		/* Clean up */
		Close(conn->fd);
		Free(conn);
	}

	return (NULL);
}

/**
 * Robustly write data to a socket
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
 * Robustly read data from a socket
 */
ssize_t
socket_read(socketio_t *sp, char *usrbuf, size_t n)
{
	int cnt;

	/* Refill the buffer if no data available */
	while (sp->s_cnt <= 0) {
		sp->s_cnt = recv(sp->s_fd, sp->s_buf, sizeof(sp->s_buf), 0);

		if (sp->s_cnt < 0) {
			if (errno != EINTR)	/* Interrupted by sig handler */
				return (-1);
		} else if (sp->s_cnt == 0) {	/* EOF */
			return (0);
		} else {
			sp->s_bufptr = sp->s_buf;	/* Reset buffer ptr */
		}
	}

	cnt = n;

	/* Copy data over to user buffer */
	if (sp->s_cnt < n)
		cnt = sp->s_cnt;

	memcpy(usrbuf, sp->s_bufptr, cnt);
	sp->s_bufptr += cnt;
	sp->s_cnt -= cnt;

	/* Return size */
	return (cnt);
}

/**
 * Initializes robust IO
 */
void
socket_readinitb(socketio_t *sp, int fd)
{
	sp->s_fd = fd;
	sp->s_cnt = 0;
	sp->s_bufptr = sp->s_buf;
}

/**
 * Robustly reads a line delimited by '\n' into a user buffer
 */
ssize_t
socket_readlineb(socketio_t *sp, char *usrbuf, size_t maxlen)
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
 * Put a buffer struct into the buffer
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
 * Pop a buffer struct from the buffer
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

