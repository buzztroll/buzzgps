#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include "buzz_logging.h"

#define MAX_LOG_LINE 1024
#define MAX_LEVEL_NAME_LEN 8

static char* _level_map[] = {
    "ERROR",
    "WARN",
    "INFO",
    "DEBUG",
    NULL
};

static char * trim(char * s) {
    size_t l = strlen(s);

    if (s[l-1] == '\n') {
        s[l-1] = '\0';
    }
    return s;
}

static int _g_level = BUZZ_INFO;

void buzz_set_log_level(const char * level) {
    int i;
    char upper_level[MAX_LEVEL_NAME_LEN];

    for (i = 0; i < strnlen(level, MAX_LEVEL_NAME_LEN); i++) {
        upper_level[i] = toupper(level[i]);
    }
    upper_level[i] = '\0';

    for(i = 0; _level_map[i] != NULL; i++) {
        if (strcmp(upper_level, _level_map[i]) == 0) {
            _g_level = i;
        }
    }
}

void buzz_logger(LOG_LEVEL level, const char* fmt, ...)
{
    time_t now;
    char * tag;
    char * time_str;

    if (level > BUZZ_DEBUG) {
        level = BUZZ_DEBUG;
    }
    if (level > _g_level) {
        return;
    }

    tag = _level_map[level];
    va_list a_list;
    va_start(a_list, fmt);

    time(&now);
    time_str = ctime(&now);
    trim(time_str);

    fprintf(stderr, "%s [%s]: ", time_str, tag);
    vfprintf(stderr, fmt, a_list);
    fprintf(stderr, "\n");

    va_end(a_list);
}

