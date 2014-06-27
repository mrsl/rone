/**
 * serial.c
 *
 * Serial port connection and IO functions
 */
#include "rcc.h"

/**
 * Connect to a serial port
 */
int serialConnect(HANDLE *hSerialPtr, int comPort)
{
	char port[128];

	if (sprintf(port, "//./COM%d", comPort) < 0)
		return (-1);

	/* Try and open serial port */
	*hSerialPtr = CreateFile(port,
							 GENERIC_READ | GENERIC_WRITE,
							 0,
							 0,
							 OPEN_EXISTING,
							 FILE_ATTRIBUTE_NORMAL,
							 0);

	if (*hSerialPtr == INVALID_HANDLE_VALUE) {
		return (-1);
	} else {
		/* Set serial parameters for the robot */
		DCB dcbSerialParams = { 0 };
		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

		if (!GetCommState(*hSerialPtr, &dcbSerialParams))
			return 0;

		dcbSerialParams.BaudRate = 230400;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;
		dcbSerialParams.fAbortOnError = 0;

		if (!SetCommState(*hSerialPtr, &dcbSerialParams))
			return 0;

		COMMTIMEOUTS timeouts = { 0 };

		timeouts.ReadIntervalTimeout = 2;
		timeouts.ReadTotalTimeoutConstant = 4;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 5;
		timeouts.WriteTotalTimeoutMultiplier = 0;

		if (!SetCommTimeouts(*hSerialPtr, &timeouts))
			return (-1);
	}
	return 0;
}

/**
 * Initialize robust IO over a serial connection. Adapted from CSAPP.
 */
void serialInitIO(struct serialIO *sp, HANDLE *hSerialPtr)
{
	sp->handle = hSerialPtr;
	sp->count = 0;
	sp->bufp = sp->buffer;
}

/**
 * Robustly read data from a serial port. Adapted from CSAPP.
 */
ssize_t serialRead(struct serialIO *sp, char *usrbuf, size_t n)
{
	int cnt;

	while (sp->count <= 0) {
		if (!ReadFile(*sp->handle, sp->buffer, BUFFERSIZE, &sp->count, NULL))
			return (-1);

		if (sp->count == 0) {
			return (0);
		} else {
			sp->bufp = sp->buffer;
		}
	}

	cnt = n;

	if (sp->count < n)
		cnt = sp->count;

	memcpy(usrbuf, sp->bufp, cnt);
	sp->bufp += cnt;
	sp->count -= cnt;

	return (cnt);
}

/**
 * Robustly read a line from a serial port. Adapted from CSAPP.
 */
ssize_t serialReadline(struct serialIO *sp, char *usrbuf, size_t maxlen)
{
	int rc;
	unsigned int n;
	char c, *bufp = usrbuf;

	for (n = 1; n < maxlen; n++) {
		if ((rc = serialRead(sp, &c, 1)) == 1) {
			*bufp++ = c;

			if (c == '\n') {
				n++;
				break;
			}
		} else if (rc == 0) {
			if (n == 1)
				return (0);
			else
				break;
		} else {
			return (-1);
		}
	}
	*bufp = 0;

	return (n - 1);
}
