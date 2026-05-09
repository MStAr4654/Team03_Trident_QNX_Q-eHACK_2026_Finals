/*
 * hmds_utils.c — Utility Functions
 * Logging and timing helpers for HMDS
 */

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <sys/neutrino.h>
#include "hmds_common.h"

// Logging function
void hmds_log(LogLevel level, const char *task, const char *fmt, ...) {
    const char *level_str[] = {
        [LOG_DEBUG] = "DEBUG  ",
        [LOG_INFO] = "INFO   ",
        [LOG_WARN] = "WARN   ",
        [LOG_ALERT] = "ALERT  ",
        [LOG_CRITICAL] = "CRITICAL"
    };
    
    char timestamp[32];
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    snprintf(timestamp, sizeof(timestamp), "%02d:%02d:%02d",
             t->tm_hour, t->tm_min, t->tm_sec);
    
    printf("[%s] [%s] [%-14s] ", level_str[level], timestamp, task);
    
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    
    printf("\n");
    fflush(stdout);
}

// High-resolution timestamp
uint64_t hmds_now_ns(void) {
#ifdef __QNX__
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
#endif
}
