/**
 * server.h
 */
#ifndef SERVER_H_
#define SERVER_H_

#define NTHREADS 5
#define LISTENQ 1024
#define CONNECTIONBUFFERSIZE 64
#define BUFFERSIZE 8192

struct Connection {
	int n;	/* Request number */
	SOCKET fd;	/* File descriptor */
};

typedef struct {
	int s_fd;
	int s_cnt;
	char *s_bufptr;
	char s_buf[BUFFERSIZE];
} socketio_t;

struct Buffer {
	void **array;
	int put_index;
	int pop_index;
	int count;
	int size;
	pthread_mutex_t mutex;
	pthread_cond_t empty;
	pthread_cond_t full;
};

extern char ipAddress[15];
extern struct Buffer connectionBuffer;

int createServer(int port);

int open_listenfd(int port);

void *incomingHandler(void *vargp);
void *connectionHandler(void *vargp);

ssize_t socket_writen(int fd, char *usrbuf, size_t n);
ssize_t socket_read(socketio_t *sp, char *usrbuf, size_t n);
void socket_readinitb(socketio_t *sp, int fd);
ssize_t socket_readlineb(socketio_t *sp, char *usrbuf, size_t maxlen);

void buffer_init(struct Buffer *buf, int n);
void buffer_put(struct Buffer *buf, void *conn);
void *buffer_pop(struct Buffer *buf);

#endif
