/**
 * serial.h
 *
 * Header file for serial.c
 */
#ifndef SERIAL_H_
#define SERIAL_H_

/**
 *  Robust IO buffer struct
 */
struct serialIO
{
	HANDLE *handle;				// Serial handle
	DWORD count;				// Count
	char *bufp;					// Buffer pointer
	char buffer[BUFFERSIZE];	// Buffer
};

int serialConnect(HANDLE *hSerialPtr, int comPort);
void serialInitIO(struct serialIO *sp, HANDLE *hSerialPtr);
ssize_t serialRead(struct serialIO *sp, char *usrbuf, size_t n);
ssize_t serialReadline(struct serialIO *sp, char *usrbuf, size_t maxlen);

#endif
