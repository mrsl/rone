/*
 * serial.c
 *
 * Handling the serial port.
 *
 */


#include "gui.h"

#define MAX_COMM_NUMBER 255
#define VALUES_TO_READ 17

#define QUEUE_SIZE	2048
char queue[QUEUE_SIZE];
int nextChar = 0;
int msgAvail = 0;
int start = 0;

/*
 * Connect to the specified com port. If it can be connected, return 1.
 * Else, return 0.
 */
int
serialConnect(HANDLE *hSerialPtr, int comPort)
{
	HANDLE hSerial;
	char port[128];

	/* TODO: Figure out why //./COM%d rather than COM%d. */
	sprintf(port, "//./COM%d", comPort);
	hSerial = CreateFile(port,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	/*
	 * If the creation fails, then return 0.
	 *
	 * If it succeeds, then set communication
	 * parameters.
	 */
	if (hSerial == INVALID_HANDLE_VALUE) {
		return 0;
	} else {
		DCB dcbSerialParams = {0};
		dcbSerialParams.DCBlength=sizeof(dcbSerialParams);

		if(!GetCommState(hSerial, &dcbSerialParams))
			return 0;

		/*
		 * Can't use CBR_230400 because it doesn't have that specific
		 *      value.
		 *
		 * That may be a problem.
		 *
		 * TODO: Figure out if that is a problem.
		 */
		/* Old baud rate: CBR_115200 */
		//dcbSerialParams.BaudRate = CBR_230400;
		dcbSerialParams.BaudRate = 230400;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;

		if(!SetCommState(hSerial, &dcbSerialParams)) {
			return 0;
		}

		COMMTIMEOUTS timeouts={0};

//		timeouts.ReadIntervalTimeout = MAXDWORD;
//		timeouts.ReadTotalTimeoutConstant = 0;
//		timeouts.ReadTotalTimeoutMultiplier = 0;
//		timeouts.WriteTotalTimeoutConstant = 5;
//		timeouts.WriteTotalTimeoutMultiplier = 5;

		timeouts.ReadIntervalTimeout = 2;
		timeouts.ReadTotalTimeoutConstant = 4;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 5;
		timeouts.WriteTotalTimeoutMultiplier = 0;

		if(!SetCommTimeouts(hSerial, &timeouts)){
			return 0;
		}
	}
	*hSerialPtr = hSerial;
	return 1;
}


int serialSendData(HANDLE file, char * msg) {
//	pthread_mutex_lock(&serialPortMutex);
	DWORD dwBytesWritten = 0;
	WriteFile(file, msg, strlen(msg), &dwBytesWritten, NULL);
//	pthread_mutex_unlock(&serialPortMutex);
	return 1;
}

/* Print to the serial port. */
GLvoid cprintf(const char *fmt, ...) {
	char text[256];
	va_list ap;

	if (fmt == NULL) {
		return;
	}

	va_start(ap, fmt);
		vsprintf(text, fmt, ap);
	va_end(ap);

	serialSendData(hSerial, text);
}


/*
 * Output files to the queue.
 * Returns zero if failed, nonzero if success.
 */
//#define READ_BUFFER		QUEUE_SIZE
#define READ_BUFFER		1024
int serialDataHandler(HANDLE file) {
	/* Read characters in the serial port to a data file. */
	char filedata[READ_BUFFER + 1] = {0};
	DWORD dwBytesRead = 0;
	if (!ReadFile(file, filedata, READ_BUFFER, &dwBytesRead, NULL)) {
		serialConnected = 0;
		CloseHandle(hSerial);
		return 0; /* Return zero if fail to read.*/
	}

	/* Input the characters into the queue. */
	int i;
	char ch;
	for (i = 0; i < dwBytesRead; i++) {
		ch = filedata[i];
		queue[nextChar] = ch;

		/* Increase message count in case of newline. */
		if (ch == '\n') {
			msgAvail++;
		}

		/* modulo-increase the next character index */
		nextChar = (nextChar + 1) % QUEUE_SIZE;
	}
	return 1;
}

/* Get the next newline-terminated message */
int serialMessageGet(char *buffer, int buflen) {
	int i;
	if (buflen < 1 || msgAvail < 1) {
		return -1;
	}

	for (i=0; i < buflen; i++) {
		buffer[i] = queue[start];
		start = (start+1) % QUEUE_SIZE;

		if (buffer[i] == '\n') {
			msgAvail--;
			if (i == buflen - 1) {
				buffer[i+1] = '\0';
			}
			return i+1;
		}
	}
	msgAvail--;
	return 1;// buflen;
}
