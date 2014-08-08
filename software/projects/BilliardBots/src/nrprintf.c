/**
 * @file nrprintf.c
 * @brief Formatting printing to a remote robot terminal
 * @since 2014
 * @author Zak Kingston
 */

#include "nrprintf.h"

static boolean nrprintfIsInit = FALSE;			// Is nrprintf initialized?
static osSemaphoreHandle nrprintfWriteMutex;	// Local write buffer mutex
static osSemaphoreHandle nrprintfSendMutex;		// Radio send buffer mutex

static char nrprintfWriteBuffer[NRPRINTF_BUFFER_SIZE];	// Local write buffer
static char nrprintfSendBuffer[NRPRINTF_BUFFER_SIZE];	// Radio send buffer

static int nrprintfWriteBufferLength;	// Length of string in write buffer
static int nrprintfSendBufferLength;	// Length of string in send buffer

static boolean nrprintfDataReady;		// Is data ready to be sent over radio?

/**
 * @brief Inserts a formatted string into the remote terminal buffer
 *
 * @returns void
 */
void nrprintf(const char *format, ...) {
	int i, n, maxLength;
	va_list arguments;

	// If not initialized, return out
	if (!nrprintfIsInit) {
		return;
	}

	va_start(arguments, format);

	// Maximum length of a string that can still be inserted in the buffer
	maxLength = NRPRINTF_BUFFER_SIZE - nrprintfWriteBufferLength - 1;

	// Safely lock the write buffer
	osSemaphoreTake(nrprintfWriteMutex, portMAX_DELAY);

	// Write string with arg formatting into write buffer
	n = vsnprintf((char *) nrprintfWriteBuffer + nrprintfWriteBufferLength,
		maxLength, format, arguments);

	// Unlock the write buffer
	osSemaphoreGive(nrprintfWriteMutex);

	// Check for overflow
	if ((nrprintfWriteBufferLength += n) >= maxLength) {
		error("nrprintf buffer overflow");
		nrprintfWriteBufferLength = maxLength;
	}

	// Clean up arguments.
	va_end(arguments);
}

/**
 * @brief Flushes the remote terminal buffer and prepares the remote terminal
 * buffer to be sent over radio. Prints out the buffer over serial as well.
 *
 * @returns void
 */
void nrprintfFlush() {
	// Safely lock the write buffer
	osSemaphoreTake(nrprintfWriteMutex, portMAX_DELAY);
	// Safely lock the send buffer
	osSemaphoreTake(nrprintfSendMutex, portMAX_DELAY);

	// Copy the buffer over, plus one to include the null terminator
	memcpy(nrprintfSendBuffer, nrprintfWriteBuffer,
		nrprintfWriteBufferLength + 1);

	// Print over serial the buffer that will be sent out
	cprintf(nrprintfSendBuffer);

	// Unlock the send buffer
	osSemaphoreGive(nrprintfSendMutex);
	// Unlock the write buffer
	osSemaphoreGive(nrprintfWriteMutex);

	nrprintfSendBufferLength = nrprintfWriteBufferLength;
	nrprintfWriteBufferLength = 0;
	nrprintfDataReady = TRUE;
}

/**
 * @brief Initializes remote printing with nrprintf
 *
 * @returns void
 */
void nrprintfInit() {
	nrprintfIsInit = TRUE;
	nrprintfWriteMutex = osSemaphoreCreateMutex();
	nrprintfSendMutex = osSemaphoreCreateMutex();

	// Initialize other variables
	nrprintfWriteBufferLength = 0;
	nrprintfDataReady = FALSE;
}
