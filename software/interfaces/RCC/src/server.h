/**
 * server.h
 */
#ifndef SERVER_H_
#define SERVER_H_

#define NTHREADS				20
#define LISTENQ					1024
#define CONNECTIONBUFFERSIZE	64
#define BUFFERSIZE				8192

/**
 * Connection information struct that is passed into request buffer
 */
struct Connection {
	int n;		/* Request number */
	SOCKET fd;	/* File descriptor */
};

/**
 * Robust IO struct used for socket IO. Adapted from CSAPP.
 */
struct socketIO {
	int fd;					/* File descriptor */
	int count;				/* Count */
	char *bufp;				/* Buffer pointer */
	char buffer[BUFFERSIZE];/* Buffer */
};

/**
 * Connection buffer used by the web server
 */
struct Buffer {
	void **array;			/* Array of pointers, the buffer */
	int put_index;			/* The current put index */
	int pop_index;			/* The current pop index */
	int count;				/* The count of items in the buffer */
	int size;				/* The current size of the buffer */
	pthread_mutex_t mutex;	/* Mutex lock */
	pthread_cond_t empty;	/* Condition variable when nothing in buffer */
	pthread_cond_t full;	/* Condition variable when buffer is full */
};

extern char ipAddress[15];				/* String form of local IP address */
extern struct Buffer connectionBuffer;	/* Connection buffer */

int createServer(int port);
void getIPAddress();
int openListenFD(int port);

void *incomingHandler(void *vargp);
void *connectionHandler(void *vargp);

void buffer_init(struct Buffer *buf, int n);
void buffer_put(struct Buffer *buf, void *conn);
void *buffer_pop(struct Buffer *buf);

ssize_t socket_writen(int fd, char *usrbuf, size_t n);
ssize_t socket_read(struct socketIO *sp, char *usrbuf, size_t n);
void socket_readinitb(struct socketIO *sp, int fd);
ssize_t socket_readlineb(struct socketIO *sp, char *usrbuf, size_t maxlen);

#endif
