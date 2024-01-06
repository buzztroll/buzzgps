/*
 * Logging module
 *
 * This is a simple logger. It adds time and level information to log lines that are
 * then written to stderr
 */
#ifndef BUZZ_LOGGER_H
#define BUZZ_LOGGER_H

#include <stdarg.h>

typedef enum LOG_LEVEL_E {
   BUZZ_ERROR = 0,
   BUZZ_WARN,
   BUZZ_INFO,
   BUZZ_DEBUG
} LOG_LEVEL;

void buzz_logger(LOG_LEVEL level, const char* message, ...);

void buzz_set_log_level(const char * level);

#endif

