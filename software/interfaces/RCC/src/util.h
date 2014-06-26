/**
 * util.h
 *
 * Header file for util.c
 */
#ifndef UTIL_H_
#define UTIL_H_


typedef signed char 	int8;
typedef signed short 	int16;

typedef unsigned char	uint8;
typedef unsigned short	uint16;

typedef CRITICAL_SECTION pthread_mutex_t;

/* Function Declarations */
uint8 convertASCIIHexNibble(char val);
uint8 convertASCIIHexByte(char *val);
uint16 convertASCIIHexWord(char *val);

void fcprintf(HANDLE *hSerialPtr, const char *fmt, ...);

void Error(const char *fmt, ...);

void Close(int fd);
void *Malloc(size_t size);
void *Calloc(size_t nmemb, size_t size);
void Free(void *p);

void mutexInit(HANDLE *mutex);

void Pthread_mutex_init(pthread_mutex_t *mp);
void Pthread_mutex_lock(pthread_mutex_t *mp);
void Pthread_mutex_unlock(pthread_mutex_t *mp);

#endif
