/**
 * serial.h
 */
#ifndef SERIAL_H_
#define SERIAL_H_

/* Robust IO buffer struct */
typedef struct {
	HANDLE *serial_h;
	DWORD serial_cnt;
	char *serial_bufptr;
	char serial_buf[BUFFERSIZE];
} serial_t;


int serialConnect(HANDLE *hSerialPtr, int comPort);
void serial_readinitb(serial_t *sp, HANDLE *hSerialPtr);
ssize_t serial_read(serial_t *sp, char *usrbuf, size_t n);
ssize_t serial_readlineb(serial_t *sp, char *usrbuf, size_t maxlen);

#endif
