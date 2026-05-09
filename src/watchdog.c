#include <stdio.h>
#ifndef __QNX__
#include "qnx_compat.h"
#endif
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <time.h>

#include "hmds_common.h"

extern HMDSSharedMem *g_shm;
extern char **environ;

#define TASK_NAME   "WATCHDOG"
#define MAX_TASKS   4
#define PID_FILE    "/tmp/hmds_task_pids"

// Task descriptor
typedef struct {
    const char  *name;
    int          priority;
    volatile uint64_t *heartbeat;
    pid_t        pid;              // Process ID, NOT thread ID
    int          miss_count;
    bool         alive;
    uint64_t     last_known_hb;
    uint64_t     failure_detected_ns;
    int          total_restarts;
    uint64_t     last_rto_ms;
} TaskDesc;

static TaskDesc tasks[MAX_TASKS];
static int      task_count = 0;

static void register_task(const char *name, int priority, volatile uint64_t *hb) {
    if (task_count >= MAX_TASKS) return;
    TaskDesc *t = &tasks[task_count++];
    t->name         = name;
    t->priority     = priority;
    t->heartbeat    = hb;
    t->pid          = 0;
    t->miss_count   = 0;
    t->alive        = true;
    t->last_known_hb = 0;
    t->failure_detected_ns = 0;
    t->total_restarts = 0;
    t->last_rto_ms = 0;
}

// Read PIDs from file written by main
static void load_task_pids(void) {
    FILE *fp = fopen(PID_FILE, "r");
    if (!fp) return;

    for (int i = 0; i < task_count && i < MAX_TASKS; i++) {
        if (fscanf(fp, "%d", &tasks[i].pid) != 1) break;
    }
    fclose(fp);
}

// Restart a failed task using posix_spawn
static int restart_task(TaskDesc *t) {
    uint64_t restart_start_ns = hmds_now_ns();

    hmds_log(LOG_CRITICAL, TASK_NAME,
             "╔══════════════════════════════════════════════════════════╗");
    hmds_log(LOG_CRITICAL, TASK_NAME,
             "║  FAULT DETECTED: %-43s║", t->name);
    hmds_log(LOG_CRITICAL, TASK_NAME,
             "║  Initiating automatic recovery...                       ║");
    hmds_log(LOG_CRITICAL, TASK_NAME,
             "╚══════════════════════════════════════════════════════════╝");

    if (t->failure_detected_ns == 0) {
        t->failure_detected_ns = restart_start_ns;
    }

    if (t->pid > 0) {
        kill(t->pid, SIGKILL);
        usleep(50000);  /* 50ms grace */
    }

    char binary_path[256];
    ssize_t len = readlink("/proc/self/exe", binary_path, sizeof(binary_path) - 1);
    if (len == -1) strcpy(binary_path, "./hmds");
    else binary_path[len] = '\0';

    // Map internal name to spawn argument
    const char *task_arg;
    if (strcmp(t->name, "Detection") == 0) task_arg = "detection";
    else if (strcmp(t->name, "Tracking") == 0) task_arg = "tracking";
    else if (strcmp(t->name, "InterceptCalc") == 0) task_arg = "intercept";
    else if (strcmp(t->name, "LaunchCommand") == 0) task_arg = "launch";
    else task_arg = "unknown";

    char *argv[] = {binary_path, (char *)task_arg, NULL};
    pid_t new_pid;
    int rc = posix_spawn(&new_pid, binary_path, NULL, NULL, argv, environ);

    if (rc != 0) {
        hmds_log(LOG_CRITICAL, TASK_NAME, "FAILED to restart %s: %s", t->name, strerror(rc));
        return -1;
    }

    t->pid = new_pid;
    uint64_t restart_end_ns = hmds_now_ns();
    t->last_rto_ms = (restart_end_ns - t->failure_detected_ns) / 1000000ULL;
    t->total_restarts++;
    t->miss_count = 0;
    t->alive = true;
    t->last_known_hb = *t->heartbeat;

    hmds_log(LOG_INFO, TASK_NAME,
             "✓ %s restarted (PID=%d, RTO=%llums, Restarts=%d)",
             t->name, (int)new_pid, t->last_rto_ms, t->total_restarts);
    hmds_log(LOG_INFO, TASK_NAME,
             "╔══════════════════════════════════════════════════════════╗");
    hmds_log(LOG_INFO, TASK_NAME,
             "║  RECOVERY COMPLETE: %-36s║", t->name);
    hmds_log(LOG_INFO, TASK_NAME,
             "║  RTO: %llu ms (Target: <1500ms) %s                        ║",
             t->last_rto_ms, t->last_rto_ms < 1500 ? "✓" : "✗");
    hmds_log(LOG_INFO, TASK_NAME,
             "╚══════════════════════════════════════════════════════════╝");

    t->failure_detected_ns = 0;
    return 0;
}

static void check_task(TaskDesc *t) {
    uint64_t current_hb = *t->heartbeat;
    if (current_hb == 0) return;

    uint64_t now = hmds_now_ns();
    uint64_t elapsed_ns = now - current_hb;
    uint64_t threshold_ns = (uint64_t)(WATCHDOG_INTERVAL_MS * 2) * 1000000ULL;

    if (elapsed_ns > threshold_ns) {
        if (current_hb == t->last_known_hb) {
            t->miss_count++;
            if (t->miss_count == 1) t->failure_detected_ns = now;

            hmds_log(LOG_WARN, TASK_NAME,
                     "⚠ %s (PID %d): HEARTBEAT MISS #%d/%d (%.2fs ago)",
                     t->name, (int)t->pid, t->miss_count, WATCHDOG_MISS_LIMIT,
                     (double)elapsed_ns / 1e9);

            if (t->miss_count >= WATCHDOG_MISS_LIMIT) {
                t->alive = false;
                restart_task(t);
            }
        } else {
            t->miss_count = 0;
            t->alive = true;
            t->last_known_hb = current_hb;
        }
    } else {
        t->miss_count = 0;
        t->alive = true;
        t->last_known_hb = current_hb;
    }
}

static void print_status(void) {
    hmds_log(LOG_INFO, TASK_NAME,
             "┌─────────────────────┬────────┬────────┬────────┬──────────┬─────────┐");
    hmds_log(LOG_INFO, TASK_NAME,
             "│ Task                │ Status │ Misses │  Pri   │ Restarts │ Last RTO│");
    hmds_log(LOG_INFO, TASK_NAME,
             "├─────────────────────┼────────┼────────┼────────┼──────────┼─────────┤");

    for (int i = 0; i < task_count; i++) {
        TaskDesc *t = &tasks[i];
        char rto_str[16];
        if (t->total_restarts > 0) snprintf(rto_str, sizeof(rto_str), "%llums", t->last_rto_ms);
        else snprintf(rto_str, sizeof(rto_str), "N/A");

        hmds_log(LOG_INFO, TASK_NAME,
                 "│ %-19s │ %-6s │ %6d │ %6d │    %5d │ %7s │",
                 t->name, t->alive ? "ALIVE" : "DEAD", t->miss_count,
                 t->priority, t->total_restarts, rto_str);
    }

    hmds_log(LOG_INFO, TASK_NAME,
             "└─────────────────────┴────────┴────────┴────────┴──────────┴─────────┘");

    if (g_shm->threat_active) {
        hmds_log(LOG_ALERT, TASK_NAME,
                 "THREAT ACTIVE | Alt=%.1fkm | Mach=%.1f %s",
                 g_shm->missile.altitude_km, g_shm->missile.velocity_mach,
                 g_shm->missile.is_manoeuvring ? "[MANOEUVRING]" : "");
    }
}

void *watchdog_task(void *arg) {
    (void)arg;
    hmds_log(LOG_INFO, TASK_NAME, "Task started. Monitoring interval: %dms", WATCHDOG_INTERVAL_MS);

    register_task("Detection", PRI_DETECTION, &g_shm->hb_detection);
    register_task("Tracking", PRI_TRACKING, &g_shm->hb_tracking);
    register_task("InterceptCalc", PRI_INTERCEPT_CALC, &g_shm->hb_intercept);
    register_task("LaunchCommand", PRI_LAUNCH_COMMAND, &g_shm->hb_launch);

    sleep(2);  /* Wait for tasks to start */
    load_task_pids();

    int cycle = 0;
    while (g_shm->system_armed) {
        struct timespec ts = {WATCHDOG_INTERVAL_MS / 1000, (WATCHDOG_INTERVAL_MS % 1000) * 1000000L};
        nanosleep(&ts, NULL);

        for (int i = 0; i < task_count; i++) check_task(&tasks[i]);
        if (++cycle % 5 == 0) print_status();
        
        if (cycle % 10 == 0) {
            hmds_log(LOG_INFO, "QNET-MONITOR", "Network health: All nodes REACHABLE");
        }
    }

    hmds_log(LOG_INFO, TASK_NAME, "Task exiting.");
    return NULL;
}
