/**
 * @file nrprintf.h
 * @brief Formatting printing to a remote robot terminal
 * @since 2014
 * @author Zak Kingston
 */

#ifndef NRPRINTF_H_
#define NRPRINTF_H_
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "../src/SerialIO/snprintf.h"
#include "roneos.h"
#include "ronelib.h"

#define NRPRINTF_BUFFER_SIZE	CPRINTF_TEXT_STRING_SIZE

void nrprintf(const char *format, ...);
void nrprintfFlush();
void nrprintfInit();

#endif /* NRPRINTF_H_ */
