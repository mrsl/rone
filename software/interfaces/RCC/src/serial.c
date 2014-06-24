/**
 * serial.c
 */
#include "rcc.h"

/**
 * Connect to a serial port
 */
int
serialConnect(HANDLE *hSerialPtr, int comPort)
{
	char port[128];

	sprintf(port, "//./COM%d", comPort);

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
		DCB dcbSerialParams = {0};
		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

		if(!GetCommState(*hSerialPtr, &dcbSerialParams))
			return 0;

		dcbSerialParams.BaudRate = 230400;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;

		if(!SetCommState(*hSerialPtr, &dcbSerialParams))
			return 0;

		COMMTIMEOUTS timeouts = {0};

		timeouts.ReadIntervalTimeout = 2;
		timeouts.ReadTotalTimeoutConstant = 4;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 5;
		timeouts.WriteTotalTimeoutMultiplier = 0;

		if(!SetCommTimeouts(*hSerialPtr, &timeouts))
			return (-1);
	}
	return 0;
}

/**
 * Initialize robust IO over a serial connection
 */
void
serial_readinitb(serial_t *sp, HANDLE *hSerialPtr)
{
	sp->serial_h = hSerialPtr;
	sp->serial_cnt = 0;
	sp->serial_bufptr = sp->serial_buf;
}

/**
 * Robustly read data from a serial port
 */
ssize_t
serial_read(serial_t *sp, char *usrbuf, size_t n)
{
	int cnt;

	while (sp->serial_cnt <= 0) {
		if (!ReadFile(*sp->serial_h, sp->serial_buf, BUFFERSIZE,
			&sp->serial_cnt, NULL))
			return (-1);

		if (sp->serial_cnt == 0) {
			return (0);
		} else {
			sp->serial_bufptr = sp->serial_buf;
		}
	}

	cnt = n;

	if (sp->serial_cnt < n)
		cnt = sp->serial_cnt;

	memcpy(usrbuf, sp->serial_bufptr, cnt);
	sp->serial_bufptr += cnt;
	sp->serial_cnt -= cnt;

	return (cnt);
}

/**
 * Robustly read a line from a serial port
 */
ssize_t
serial_readlineb(serial_t *sp, char *usrbuf, size_t maxlen)
{
	int rc;
	unsigned int n;
	char c, *bufp = usrbuf;

	for (n = 1; n < maxlen; n++) {
		if ((rc = serial_read(sp, &c, 1)) == 1) {
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
