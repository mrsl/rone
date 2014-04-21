/*
 * logger.h
 *
 *  Created on: Aug 3, 2012
 *      Author: Jeremy Hunt
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#define LOGGER_TASK_PRIORITY				BACKGROUND_TASK_PRIORITY

#define LOGGER_WRITE_BUFFER_SIZE			1024
#define LOGGER_PRINTF_TEXT_STRING_SIZE		256

typedef struct {
	char * filename;
} LoggerTaskParams;

/* Results of Disk Functions */
typedef enum {
	LOGGER_OK = 0,		/* The function succeeded. */
	LOGGER_NOINIT,		/* The logger did not initialize correctly */
	LOGGER_ERROR		/* Any hard error occured during the write operation and could not recover it. */
} LoggerStatus;

/* Function Prototypes */
LoggerStatus loggerInit(char * filename, uint32 writeBufferSize);
LoggerStatus loggerGetStatus(void);
LoggerStatus loggerPrintf(char * format, ...);
LoggerStatus loggerSync(void);
LoggerStatus loggerChangeFile(char * filename);



#endif /* LOGGER_H_ */
