/**
 * util.c
 *
 * Utility functions such as robust IO
 */
#include "rcc.h"

uint8
convertASCIIHexNibble(char val)
{
       if (val >= '0' && val <= '9')
               return (val - '0');
       else if (val >= 'A' && val <= 'F')
               return (val - 'A' + 10);
       else if (val >= 'a' && val <= 'f')
               return (val - 'a' + 10);
       else
               return (0);
}

uint8
convertASCIIHexByte(char *val)
{
       uint8 temp;
       temp = convertASCIIHexNibble(*val) * 16;
       temp = temp + convertASCIIHexNibble(*(val + 1));
       return (temp);
}

uint16
convertASCIIHexWord(char *val)
{
       uint16 temp;

       temp = (uint16)(convertASCIIHexByte(val)) * 256;
       temp = temp + (uint16)(convertASCIIHexByte(val + 2));
       return (temp);
}

uint32
convertASCIIHexLong(char *val)
{
       uint32 temp;
       temp = (uint32)convertASCIIHexWord(val) * 65536;
       temp = temp + (uint32)convertASCIIHexWord(val + 4);
       return (temp);
}

/**
 * Write data to a file handle
 */
void
fcprintf(HANDLE *hSerialPtr, const char *fmt, ...)
{
	char text[256];
	va_list ap;

	if (fmt == NULL)
		return;

	va_start(ap, fmt);
		vsprintf(text, fmt, ap);
	va_end(ap);

	DWORD dwBytesWritten = 0;
	WriteFile(*hSerialPtr, text, strlen(text), &dwBytesWritten, NULL);
}

void
Close(int fd)
{
	if (closesocket(fd) < 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: close failure\n");
		exit (-1);
	}
}

void
*Malloc(size_t size)
{
	void *p;

	if ((p = malloc(size)) == NULL) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: malloc failure\n");
		exit (-1);
	}

	return (p);
}

void
*Calloc(size_t nmemb, size_t size)
{
	void *p;

	if ((p = calloc(nmemb, size)) == NULL) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: calloc failure\n");
		exit (-1);
	}

	return (p);
}

void
Free(void *p)
{
	free(p);
}

/**
 * Creates a new pthread. Exits program with error if failed.
 */
void
Pthread_create(pthread_t *tidp, pthread_attr_t *attrp,
    void *(*routine)(void *), void *argp)
{
	int rc;

	if ((rc = pthread_create(tidp, attrp, routine, argp)) != 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: pthread_create failure\n");
		exit (-1);
	}
}

void
Pthread_detach(pthread_t tid)
{
	int rc;

	if ((rc = pthread_detach(tid)) != 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: pthread_detach failure\n");
		exit (-1);
	}
}

void
Pthread_mutex_init(pthread_mutex_t *mp, pthread_mutexattr_t *attr)
{
	if (pthread_mutex_init(mp, attr) < 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: pthread_mutex_init failure\n");
		exit (-1);
	}
}

void
Pthread_cond_init(pthread_cond_t *mp, pthread_condattr_t *attr)
{
	if (pthread_cond_init(mp, attr) < 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: pthread_cond_init failure\n");
		exit (-1);
	}
}

void
Pthread_mutex_lock(pthread_mutex_t *mp)
{
	if (pthread_mutex_lock(mp) == 0)
		return;

	if (VERBOSE)
	fprintf(stderr, "ERROR: pthread_mutex_lock failure\n");
	exit (-1);
}

void
Pthread_mutex_unlock(pthread_mutex_t *mp)
{
	if (pthread_mutex_unlock(mp) == 0)
		return;

	if (VERBOSE)
	fprintf(stderr, "ERROR: pthread_mutex_unlock failure\n");
	exit (-1);
}

void
Pthread_cond_broadcast(pthread_cond_t *cp)
{
	if (pthread_cond_broadcast(cp) == 0)
		return;

	if (VERBOSE)
	fprintf(stderr, "ERROR: pthread_cond_broadcast failure\n");
	exit (-1);
}

void
Pthread_cond_signal(pthread_cond_t *cp)
{
	if (pthread_cond_signal(cp) == 0)
		return;

	if (VERBOSE)
	fprintf(stderr, "ERROR: pthread_cond_signal failure\n");
	exit (-1);
}


void
Pthread_cond_wait(pthread_cond_t *cp, pthread_mutex_t *mp)
{
	if (pthread_cond_wait(cp, mp) == 0)
		return;

	if (VERBOSE)
	fprintf(stderr, "ERROR: Pthread_cond_wait failure\n");
	exit (-1);
}
