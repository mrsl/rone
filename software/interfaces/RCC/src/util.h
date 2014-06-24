/**
 * util.h
 */
#ifndef UTIL_H_
#define UTIL_H_

#define SBUFSIZE 256

typedef signed char 	int8;
typedef signed short 	int16;
typedef signed long 	int32;

typedef unsigned char	uint8;
typedef unsigned short	uint16;
typedef unsigned long	uint32;

/* Function Declarations */
uint8 convertASCIIHexNibble(char val);
uint8 convertASCIIHexByte(char *val);
uint16 convertASCIIHexWord(char *val);
uint32 convertASCIIHexLong(char *val);

void fcprintf(HANDLE *hSerialPtr, const char *fmt, ...);

void Close(int fd);

void *Malloc(size_t size);
void *Calloc(size_t nmemb, size_t size);
void Free(void *p);

void Pthread_create(pthread_t *tidp, pthread_attr_t *attrp,
	void * (*routine)(void *), void *argp);
void Pthread_detach(pthread_t tid);
void Pthread_mutex_init(pthread_mutex_t *mp, pthread_mutexattr_t *attr);
void Pthread_cond_init(pthread_cond_t *mp, pthread_condattr_t *attr);
void Pthread_mutex_lock(pthread_mutex_t *mp);
void Pthread_mutex_unlock(pthread_mutex_t *mp);
void Pthread_cond_broadcast(pthread_cond_t *cp);
void Pthread_cond_signal(pthread_cond_t *cp);
void Pthread_cond_wait(pthread_cond_t *cp, pthread_mutex_t *mp);

#endif
