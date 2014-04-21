/*
 * logger.c
 *
 *  Created on: Aug 3, 2012
 *      Author: Jeremy Hunt
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "roneos.h"
//#include "../../SerialIO/snprintf.h"

// Defines
#define LOGGER_AFTER_SYNC_SLEEP_TIME 30

static osSemaphoreHandle loggerPrintMutex;
static osSemaphoreHandle loggerBufferMutex;

static osSemaphoreHandle loggerSyncBeginSemaphore;
static osSemaphoreHandle loggerSyncDoneSemaphore;

char * loggerPrintBuffer;
uint8 * loggerWriteBuffer;
volatile static uint32 loggerWriteBufferSize;
volatile static uint32 loggerWriteBufferUsed;
volatile static boolean loggerBeginChangeFile;
volatile static boolean loggerBeginSync;
static char * loggerChangeFilePtr;

volatile static LoggerStatus loggerStatus;

boolean loggerPrintfOverRunError = FALSE;


/*=========================== Function Prototypes ===========================*/
void loggerTask(void * parameters);


LoggerStatus loggerGetStatus(void){
	return loggerStatus;
}

/**
 * @breif Initialized the SD Card Logger.
 *
 * Initialized the SD Card Logger with the logger output set to the given input
 * file.
 *
 * @param filename The absolute path to the file where the log will be stored.
 *
 */
LoggerStatus loggerInit(char * filename, uint32 writeBufferSize){
	// Create the mutexes
	loggerPrintMutex = osSemaphoreCreateMutex();
	loggerBufferMutex = osSemaphoreCreateMutex();

	// Create the semaphores
	osSemaphoreCreateBinary(loggerSyncDoneSemaphore);
	osSemaphoreCreateBinary(loggerSyncBeginSemaphore);
	//Take the semaphore to start so that it doesn't write the buffer the first time
	osSemaphoreTake(loggerSyncBeginSemaphore, portMAX_DELAY);

	loggerBeginSync = FALSE;
	loggerStatus = LOGGER_OK;

	// Initialize the write buffer
	loggerWriteBufferSize = writeBufferSize;
	loggerWriteBuffer = malloc(sizeof(uint8)*writeBufferSize);
	if(loggerWriteBuffer == NULL){
		loggerStatus = LOGGER_NOINIT;
		error("\nCannot allocate memory for logger write buffer. Try decreasing the buffer size.\n");
		return loggerStatus;
	}
	loggerWriteBufferUsed = 0;

	// Initialize the print buffer
	loggerPrintBuffer = malloc(sizeof(char)*LOGGER_PRINTF_TEXT_STRING_SIZE);
	if(loggerPrintBuffer == NULL){
		loggerStatus = LOGGER_NOINIT;
		error("\nCannot allocate memory for logger print buffer. Try decreasing the LOGGER_PRINTF_TEXT_STRING_SIZE parameter.\n");
		return loggerStatus;
	}


	// Create the parameters to send to the thread
	LoggerTaskParams * loggerTaskInitParams = malloc(sizeof(LoggerTaskParams));
	if(loggerTaskInitParams == NULL){
		loggerStatus = LOGGER_NOINIT;
		error("\nCannot allocate memory for logger parameters...\n");
		return loggerStatus;
	}
	loggerTaskInitParams->filename = filename;

	// Start the logger thread
	if(osTaskCreate(loggerTask, "logger", 2048, loggerTaskInitParams, LOGGER_TASK_PRIORITY) != pdPASS){
		loggerStatus = LOGGER_NOINIT;
		error("\nError creating logger thread...\n");
		return loggerStatus;
	}

	return loggerStatus;
}

void loggerTask(void * parameters){
	LoggerTaskParams * loggerTaskInitParams = (LoggerTaskParams *)parameters;

	FATFS loggerFs;
	FIL loggerActiveFileObj;
	FIL * loggerActiveFile = &loggerActiveFileObj;
	uint32 bytesDone = 0;

	if(f_mount(0, &loggerFs) != FR_OK){
		loggerStatus = LOGGER_NOINIT;
		error("\nError Mounting SD Card.\n");
	}

	if(loggerStatus == LOGGER_OK){
		if(f_open(loggerActiveFile, loggerTaskInitParams->filename, (FA_OPEN_ALWAYS |FA_WRITE)) != FR_OK){
			loggerStatus = LOGGER_NOINIT;
			error("\nError Opening File on SD Card.\n");
		}
	}


	for(;;){
		// Wait until you are asked to begin syncing (usually when the buffer is full)
		osSemaphoreTake(loggerSyncBeginSemaphore, portMAX_DELAY);
		if(loggerStatus != LOGGER_NOINIT){
			if(loggerBeginSync){
				osSemaphoreTake(loggerBufferMutex, portMAX_DELAY);

				loggerStatus = LOGGER_OK;

				//Write at the end of the file.
				if(f_lseek(loggerActiveFile, f_size(loggerActiveFile)) != FR_OK){
					loggerStatus = LOGGER_ERROR;
					error("Error moving pointer in logger file.\n");
				}
				if(f_write(loggerActiveFile, loggerWriteBuffer, loggerWriteBufferUsed, (UINT *)(&bytesDone)) ||
						bytesDone != loggerWriteBufferUsed){
					loggerStatus = LOGGER_ERROR;
					error("Error writing to logger file.\n");
				}
				if(f_sync(loggerActiveFile) != FR_OK){
					loggerStatus = LOGGER_ERROR;
					error("Error syncing logger file.\n");
				}

				loggerWriteBufferUsed = 0;
				loggerBeginSync = FALSE;
				osSemaphoreGive(loggerBufferMutex);
			} else if(loggerBeginChangeFile){
				// Write the current file buffer before switching files
				osSemaphoreTake(loggerBufferMutex, portMAX_DELAY);

				loggerStatus = LOGGER_OK;

				//Write at the end of the file.
				if(f_lseek(loggerActiveFile, f_size(loggerActiveFile)) != FR_OK){
					loggerStatus = LOGGER_ERROR;
					error("Error moving pointer in logger file.\n");
				}
				if(f_write(loggerActiveFile, loggerWriteBuffer, loggerWriteBufferUsed, (UINT *)(&bytesDone)) ||
						bytesDone != loggerWriteBufferUsed){
					loggerStatus = LOGGER_ERROR;
					error("Error writing to logger file.\n");
				}
				if(f_sync(loggerActiveFile) != FR_OK ||
						loggerStatus != LOGGER_OK){
					loggerStatus = LOGGER_ERROR;
					error("Error syncing logger file.\n");
				}

				loggerWriteBufferUsed = 0;
				loggerBeginSync = FALSE;
				osSemaphoreGive(loggerBufferMutex);

				// Close the current file
				if(f_close(loggerActiveFile) != FR_OK){
					loggerStatus = LOGGER_ERROR;
					error("Error closing logger file.\n");
				}

				loggerTaskInitParams->filename = loggerChangeFilePtr;

				// Open the new file
				if(f_open(loggerActiveFile, loggerTaskInitParams->filename, (FA_OPEN_ALWAYS |FA_WRITE)) != FR_OK){
					loggerStatus = LOGGER_ERROR;
					error("Error Opening File on SD Card.\n");
				}

				loggerBeginChangeFile = FALSE;
			}
		}

		// Sleep after a sync for a little while to give other things a
		// chance to talk
		osTaskDelay(LOGGER_AFTER_SYNC_SLEEP_TIME);

		// Give the Done Mutex to signal doneness
		osSemaphoreGive(loggerSyncDoneSemaphore);
	}

	free(loggerWriteBuffer);
	free(loggerTaskInitParams);
}

/*
 * Write whatever is left in the buffer to the end of the file immediately.
 */
LoggerStatus loggerSync(void){
	// Return if the card wasn't initialized
	if(loggerStatus == LOGGER_NOINIT){
		return loggerStatus;
	}

	// Make sure to finish any current writing process first
	osSemaphoreTake(loggerSyncDoneSemaphore, portMAX_DELAY);
	osSemaphoreTake(loggerBufferMutex, portMAX_DELAY);
	osSemaphoreGive(loggerSyncDoneSemaphore);

	osSemaphoreTake(loggerSyncDoneSemaphore, portMAX_DELAY); // Make the sync not done
	loggerBeginSync = TRUE;
	osSemaphoreGive(loggerSyncBeginSemaphore);

	osSemaphoreGive(loggerBufferMutex);

	return loggerStatus;
}

/*
 * Changes the active file that the logger prints to. Closes and syncs the file
 * that is currently active, then switches to the new file. If the new file
 * does not exist, it will be created.
 */
LoggerStatus loggerChangeFile(char * filename){
	// Return if the card wasn't initialized
	if(loggerStatus == LOGGER_NOINIT){
		return loggerStatus;
	}

	// Make sure that there is no other print going on
	osSemaphoreTake(loggerPrintMutex, portMAX_DELAY);

	// Wait until any pending writes are done, but don't actually signal not doneness
	osSemaphoreTake(loggerSyncDoneSemaphore, portMAX_DELAY);
	osSemaphoreGive(loggerSyncDoneSemaphore);

	osSemaphoreTake(loggerSyncDoneSemaphore, portMAX_DELAY); // Make the sync not done
	loggerBeginChangeFile = TRUE;
	loggerChangeFilePtr = filename;
	osSemaphoreGive(loggerSyncBeginSemaphore);

	osSemaphoreGive(loggerBufferMutex);

	osSemaphoreGive(loggerPrintMutex);

	return loggerStatus;
}

/*
 * Add a character to the Logger Write Buffer. If the buffer is full, signal to
 * start the writing process.
 */
static void loggerPutChar(char c){
	// Return if the card wasn't initialized
	if(loggerStatus == LOGGER_NOINIT){
		return;
	}

	// Make sure to finish any current writing process first
	osSemaphoreTake(loggerSyncDoneSemaphore, portMAX_DELAY);
	osSemaphoreTake(loggerBufferMutex, portMAX_DELAY);
	osSemaphoreGive(loggerSyncDoneSemaphore);

	loggerWriteBuffer[loggerWriteBufferUsed] = c;
	loggerWriteBufferUsed++;

	if(loggerWriteBufferUsed >= loggerWriteBufferSize){
		osSemaphoreTake(loggerSyncDoneSemaphore, portMAX_DELAY); // Make the sync not done
		loggerBeginSync = TRUE;
		osSemaphoreGive(loggerSyncBeginSemaphore);
	}

	osSemaphoreGive(loggerBufferMutex);
}

extern uint8 CRLFMode;
/**
 *	@brief Prints a formatted output string to the SD card Logger.
 *
 *	Processes the input string into an output string rone understands.
 *	If the input string is too large, loggerPrintfOverRunError is set to TRUE.
 *
 *	@returns void
 */
LoggerStatus loggerPrintf(char *format, ...) {
	// Return if the card wasn't initialized
	if(loggerStatus == LOGGER_NOINIT){
		return loggerStatus;
	}

    va_list arguments;

    va_start(arguments, format);

	int32 potentialChars;
	char* charPtr;

	osSemaphoreTake(loggerPrintMutex, portMAX_DELAY);

	/* Process the input string and store the output in the 'outStringFormatted' */
	/* note that vsnprintf returns the number of characters that would have been
	 * written to the buffer if it were large enough. */
	potentialChars = vsnprintf(loggerPrintBuffer, LOGGER_PRINTF_TEXT_STRING_SIZE - 1, format, arguments);
	if (potentialChars >= (LOGGER_PRINTF_TEXT_STRING_SIZE - 1)) {
		loggerPrintfOverRunError = TRUE;
	}
	charPtr = loggerPrintBuffer;
	while (*charPtr != '\0') {
		if (*charPtr == '\n') {
			// change '\n' into the current CRLF mode
			switch (CRLFMode) {
			case CPRINTF_CRLF_CR:
				loggerPutChar('\r');
			break;
			case CPRINTF_CRLF_CRLF:
				loggerPutChar('\r');
				loggerPutChar('\n');
			break;
			case CPRINTF_CRLF_LFCR:
				loggerPutChar('\n');
				loggerPutChar('\r');
			break;
			case CPRINTF_CRLF_LF:
			default:
				loggerPutChar('\n');
			break;
			}
		} else {
			loggerPutChar(*charPtr);
		}
		charPtr++;
	}

	osSemaphoreGive(loggerPrintMutex);
	// clean up the argument list
    va_end(arguments);

    return loggerStatus;
}


