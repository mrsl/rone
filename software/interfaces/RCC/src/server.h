/**
 * server.h
 *
 * Header for server.c
 */
#ifndef SERVER_H_
#define SERVER_H_

#define LISTENQ					1024
#define CONNECTIONBUFFERSIZE	64
#define BUFFERSIZE				8192

/**
 * Connection information struct that is passed to the handlers
 */
struct Connection
{
	int n;		// Request number
	SOCKET fd;	// File descriptor
};

/**
 * Robust IO struct used for socket IO. Adapted from CSAPP.
 */
struct socketIO
{
	int fd;						// File descriptor
	int count;					// Count
	char *bufp;					// Buffer pointer
	char buffer[BUFFERSIZE];	// Buffer
};

extern char ipAddress[15];				// String form of local IP address
extern struct Buffer connectionBuffer;	// Connection buffer

int createServer(int port);
void getIPAddress();
int openListenFD(int port);

void incomingHandler(void *vargp);
void connectionHandler(void *vargp);

ssize_t socketWrite(int fd, char *usrbuf, size_t n);
ssize_t socketRead(struct socketIO *sp, char *usrbuf, size_t n);
void socketInitIO(struct socketIO *sp, int fd);
ssize_t socketReadline(struct socketIO *sp, char *usrbuf, size_t maxlen);

#endif
