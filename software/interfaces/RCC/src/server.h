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

#define MAX_APRILTAG			1024
#define	APRILTAG_BUFFERSIZE		64
#define NUMBUFFER_APRILTAG		16

/**
 * Connection information struct that is passed to the handlers
 */
struct Connection
{
	int n;		// Request number
	SOCKET fd;	// File descriptor
};

/**
 * AprilTag data buffer
 */
struct aprilTag
{
	int id;
	int rid;												// Robot ID linked
	time_t up;
	int active;												// Been seen?
	int head;												// Head of buffer
	char buffer[NUMBUFFER_APRILTAG][APRILTAG_BUFFERSIZE]; 	// Buffer
	int log;												// Log this tag
	HANDLE logH;											// Logfile
	int display;											// Show data
	GLfloat x;												// X Coordinate
	GLfloat y;												// Y Coordinate
	GLfloat t;												// Theta
	CRITICAL_SECTION mutex;									// Mutex
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
extern int aprilTagConnected;			// Connected to the AprilTag server?
extern int maxAprilTag;					// Highest AprilTag seen
extern struct aprilTag aprilTagData[MAX_APRILTAG];	// AprilTag data

int createServer(int port);
void getIPAddress();
SOCKET openListenFD(int port);
SOCKET openClientFD(char *hostname, char *port);

void incomingHandler(void *vargp);
void connectionHandler(void *vargp);
void initAprilTag();
int connectAprilTag();
void aprilTagHandler(void *vargp);
int appendAprilTagData(char *buffer, int n, int aid);

ssize_t socketWrite(int fd, char *usrbuf, size_t n);
ssize_t socketRead(struct socketIO *sp, char *usrbuf, size_t n);
void socketInitIO(struct socketIO *sp, int fd);
ssize_t socketReadline(struct socketIO *sp, char *usrbuf, size_t maxlen);

#endif
