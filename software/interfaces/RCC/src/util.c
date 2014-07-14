/**
 * util.c
 *
 * Miscellaneous utility functions
 */
#include "rcc.h"

/**
 * Convert hex values from the robot into the correct endianness
 */
uint8 convertASCIIHexNibble(char val)
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

uint8 convertASCIIHexByte(char *val)
{
	uint8 temp;
	temp = convertASCIIHexNibble(*val) * 16;
	temp = temp + convertASCIIHexNibble(*(val + 1));
	return (temp);
}

uint16 convertASCIIHexWord(char *val)
{
	uint16 temp;

	temp = (uint16) (convertASCIIHexByte(val)) * 256;
	temp = temp + (uint16) (convertASCIIHexByte(val + 2));
	return (temp);
}

/**
 *
 */
void datestr(char *buffer)
{
	SYSTEMTIME st;

    GetLocalTime(&st);

    sprintf(buffer, "%d-%d-%d-T%d%d%d",
    	st.wYear, st.wMonth, st.wDay,  st.wHour, st.wMinute, st.wSecond);
}

/**
 * Insert character into the string at the position.
 * There should be room in the string.
 */
void strins(char* str, char c, int index)
{
	int i, temp1, temp2;
	temp1 = str[index];
	for (i = index + 1; str[i] != '\0'; i++) {
		temp2 = str[i];
		str[i] = temp1;
		temp1 = temp2;
	}
	str[i] = temp1;
	str[i + 1] = '\0';
	str[index] = c;
}

/**
 * Delete the character at index and shift everything to the left.
 */
void strdel(char* str, int index)
{
	int i;
	for (i = index; str[i] != '\0'; i++)
		str[i] = str[i + 1];
}

/**
 * Creates a new native Windows thread that runs detached
 */
void makeThread(void *function, void *args)
{
	//CloseHandle((HANDLE) _beginthread(function, 0, args));
	_beginthread(function, 0, args);
}

/**
 * Write data to a file handle
 */
void hprintf(HANDLE *hSerialPtr, const char *fmt, ...)
{
	DWORD dwBytesWritten;
	char text[BUFFERSIZE + APRILTAG_BUFFERSIZE + 16];
	va_list ap;

	if (fmt == NULL)
		return;

	va_start(ap, fmt);
		vsprintf(text, fmt, ap);
	va_end(ap);

	WriteFile(*hSerialPtr, text, strlen(text), &dwBytesWritten, NULL);
}

/**
 * Print a nice message and exit on error
 */
void Error(const char *fmt, ...)
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

	exit(-1);
}

/**
 * Function wrappers
 */
void Close(int fd)
{
	if (closesocket(fd) < 0)
		Error("close failure");
}

void *Malloc(size_t size)
{
	void *p;

	if ((p = malloc(size)) == NULL)
		Error("malloc failure");

	return (p);
}

void *Calloc(size_t nmemb, size_t size)
{
	void *p;

	if ((p = calloc(nmemb, size)) == NULL)
		Error("calloc failure");

	return (p);
}

void Free(void *p)
{
	free(p);
}

void mutexLock(CRITICAL_SECTION *m)
{
	EnterCriticalSection(m);
}

void mutexUnlock(CRITICAL_SECTION *m)
{
	LeaveCriticalSection(m);
}

void mutexInit(CRITICAL_SECTION *m)
{
	InitializeCriticalSection(m);
}

