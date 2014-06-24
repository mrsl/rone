/**
 * serial.h
 */
#ifndef SERIAL_H_
#define SERIAL_H_

/* Robust IO buffer struct */
struct serialIO {
	HANDLE *handle;
	DWORD count;
	char *bufp;
	char buffer[BUFFERSIZE];
};


int serialConnect(HANDLE *hSerialPtr, int comPort);
void serial_readinitb(struct serialIO *sp, HANDLE *hSerialPtr);
ssize_t serial_read(struct serialIO *sp, char *usrbuf, size_t n);
ssize_t serial_readlineb(struct serialIO *sp, char *usrbuf, size_t maxlen);

#endif
