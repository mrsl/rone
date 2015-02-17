/**
 * util.h
 *
 * Header file for util.c
 */
#ifndef UTIL_H_
#define UTIL_H_

typedef signed char int8;
typedef signed short int16;

typedef unsigned char uint8;
typedef unsigned short uint16;

/* Function Declarations */
uint8 convertASCIIHexNibble(char val);
uint8 convertASCIIHexByte(char *val);
uint16 convertASCIIHexWord(char *val);

void datestr(char *buffer);
//int round(GLfloat x);
void strins(char* str, char c, int index);
void strdel(char* str, int index);

void makeThread(void *function, void *args);

void hprintf(HANDLE *hSerialPtr, const char *fmt, ...);
HANDLE createRegularFile(char *fileName);
void executeProgram(char *name, char *args);

void Error(const char *fmt, ...);

void Close(int fd);
void *Malloc(size_t size);
void *Calloc(size_t nmemb, size_t size);
void Free(void *p);

void mutexLock(CRITICAL_SECTION *mp);
void mutexUnlock(CRITICAL_SECTION *mp);
void mutexInit(CRITICAL_SECTION *mp);

#endif
