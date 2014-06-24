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

/**
 * Write data to a file handle
 */
void
fcprintf(HANDLE *hSerialPtr, const char *fmt, ...)
{
	char text[BUFFERSIZE];
	va_list ap;

	if (fmt == NULL)
		return;

	va_start(ap, fmt);
		vsprintf(text, fmt, ap);
	va_end(ap);

	DWORD dwBytesWritten = 0;
	WriteFile(*hSerialPtr, text, strlen(text), &dwBytesWritten, NULL);
}

/**
 * Print a nice message and exit on error
 */
void
Error(const char *fmt, ...)
{
	char text[BUFFERSIZE];
	va_list ap;

	if (verbose) {
		if (fmt == NULL)
			return;

		va_start(ap, fmt);
			vsprintf(text, fmt, ap);
		va_end(ap);

		fprintf(stderr, "ERROR: %s\n", text);
	}

	exit (-1);
}

/**
 * Function wrappers
 */
void
Close(int fd)
{
	if (closesocket(fd) < 0)
		Error("close failure");
}

void
*Malloc(size_t size)
{
	void *p;

	if ((p = malloc(size)) == NULL)
		Error("malloc failure");

	return (p);
}

void
*Calloc(size_t nmemb, size_t size)
{
	void *p;

	if ((p = calloc(nmemb, size)) == NULL)
		Error("calloc failure");

	return (p);
}

void
Free(void *p)
{
	free(p);
}

void
Pthread_create(pthread_t *tidp, pthread_attr_t *attrp,
    void *(*routine)(void *), void *argp)
{
	int rc;

	if ((rc = pthread_create(tidp, attrp, routine, argp)) != 0)
		Error("pthread_create failure");
}

void
Pthread_detach(pthread_t tid)
{
	int rc;

	if ((rc = pthread_detach(tid)) != 0)
		Error("pthread_detach failure");
}

void
Pthread_mutex_init(pthread_mutex_t *mp, pthread_mutexattr_t *attr)
{
	if (pthread_mutex_init(mp, attr) < 0)
		Error("pthread_mutex_init failure");
}

void
Pthread_cond_init(pthread_cond_t *mp, pthread_condattr_t *attr)
{
	if (pthread_cond_init(mp, attr) < 0)
		Error("pthread_cond_init failure");
}

void
Pthread_mutex_lock(pthread_mutex_t *mp)
{
	if (pthread_mutex_lock(mp) == 0)
		return;

	Error("pthread_mutex_lock failure");
}

void
Pthread_mutex_unlock(pthread_mutex_t *mp)
{
	if (pthread_mutex_unlock(mp) == 0)
		return;

	Error("pthread_mutex_unlock failure");
}

void
Pthread_cond_broadcast(pthread_cond_t *cp)
{
	if (pthread_cond_broadcast(cp) == 0)
		return;

	Error("pthread_cond_broadcast failure");
}

void
Pthread_cond_signal(pthread_cond_t *cp)
{
	if (pthread_cond_signal(cp) == 0)
		return;

	Error("pthread_cond_signal failure");
}

void
Pthread_cond_wait(pthread_cond_t *cp, pthread_mutex_t *mp)
{
	if (pthread_cond_wait(cp, mp) == 0)
		return;

	Error("pthread_cond_wait failure");
}
