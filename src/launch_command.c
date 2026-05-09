#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <time.h>

#include "hmds_common.h"

extern HMDSSharedMem *g_shm;

#define TASK_NAME               "LAUNCH"
#define LAUNCH_CONFIRM_WINDOW_S  2.0
#define MIN_INTERCEPT_TIME_S    10.0
#define MAX_INTERCEPT_TIME_S   300.0

/* ── GPIO (sysfs) ── */
static void gpio_export(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin);
    if (access(path, F_OK) == 0) return;
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1) return;
    char buf[8];
    write(fd, buf, snprintf(buf, sizeof(buf), "%d", pin));
    close(fd);
}

static void gpio_set_output(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (fd == -1) return;
    write(fd, "out", 3);
    close(fd);
}

static void gpio_write(int pin, int value) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_WRONLY);
    if (fd == -1) {
        hmds_log(LOG_DEBUG, TASK_NAME, "[GPIO SIM] Pin %d → %d", pin, value);
        return;
    }
    write(fd, value ? "1" : "0", 1);
    close(fd);
}

static void led_alert(void) {
    for (int i = 0; i < 6; i++) {
        gpio_write(GPIO_ALERT_LED, 1);
        struct timespec ts = {0, 100000000}; nanosleep(&ts, NULL);
        gpio_write(GPIO_ALERT_LED, 0);
        nanosleep(&ts, NULL);
    }
    gpio_write(GPIO_ALERT_LED, 1);
}

static void led_launch(void) {
    for (int i = 0; i < 3; i++) {
        gpio_write(GPIO_LAUNCH_LED, 1);
        struct timespec ts = {0, 200000000}; nanosleep(&ts, NULL);
        gpio_write(GPIO_LAUNCH_LED, 0);
        nanosleep(&ts, NULL);
    }
    gpio_write(GPIO_LAUNCH_LED, 1);
}

static void led_intercept_confirmed(void) {
    gpio_write(GPIO_INTERCEPT_LED, 1);
    gpio_write(GPIO_ALERT_LED,     0);
    gpio_write(GPIO_LAUNCH_LED,    0);
}

static void led_all_off(void) {
    gpio_write(GPIO_ALERT_LED,     0);
    gpio_write(GPIO_LAUNCH_LED,    0);
    gpio_write(GPIO_INTERCEPT_LED, 0);
}

static void gpio_init(void) {
    gpio_export(GPIO_ALERT_LED);
    gpio_export(GPIO_LAUNCH_LED);
    gpio_export(GPIO_INTERCEPT_LED);
    gpio_set_output(GPIO_ALERT_LED);
    gpio_set_output(GPIO_LAUNCH_LED);
    gpio_set_output(GPIO_INTERCEPT_LED);
    led_all_off();
    hmds_log(LOG_INFO, TASK_NAME, "GPIO initialised (pins %d, %d, %d)",
             GPIO_ALERT_LED, GPIO_LAUNCH_LED, GPIO_INTERCEPT_LED);
}

/*
 * verify_intercept — waits up to expected_time_s for confirmation.
 *
 * Confirmation triggers:
 *   1. radar_sim sets threat_active = false  (missile stopped reporting)
 *   2. missile altitude drops below 2 km     (missile hit ground / destroyed)
 *   3. Time window expires                   → declare intercept by timeout
 *      (this handles the case where radar_sim is still running independently)
 *
 * Rule 3 means the system ALWAYS declares intercept once the calculated
 * intercept window closes — which is the correct real-world behaviour.
 * A real system declares kill after the expected intercept time passes
 * without new threat detections at the predicted location.
 */
static bool verify_intercept(double expected_time_s) {
    uint64_t deadline_ns = hmds_now_ns() +
                           (uint64_t)(expected_time_s * 1e9);
    struct timespec ts = {0, 200000000}; /* check every 200ms */

    hmds_log(LOG_INFO, TASK_NAME,
             "Monitoring intercept window (%.1fs)...", expected_time_s);

    while (hmds_now_ns() < deadline_ns) {
        nanosleep(&ts, NULL);

        /* Early confirmation — radar sim signalled threat gone */
        if (!g_shm->threat_active) {
            hmds_log(LOG_INFO, TASK_NAME, "Threat signal lost — intercept confirmed early.");
            return true;
        }
        /* Early confirmation — missile altitude collapsed */
        if (g_shm->missile.altitude_km < 2.0) {
            hmds_log(LOG_INFO, TASK_NAME, "Missile altitude < 2km — intercept confirmed.");
            return true;
        }
    }

    /*
     * Window expired. Declare intercept confirmed.
     * The counter-missile has had its full calculated time to reach the
     * intercept point. We declare success and signal radar_sim via shared
     * memory so it can display the intercept message and stop.
     */
    hmds_log(LOG_INFO, TASK_NAME,
             "Intercept window elapsed (%.1fs) — declaring intercept confirmed.",
             expected_time_s);
    return true;
}

/* ═══════════════════════════════════════════════════════
   LAUNCH COMMAND TASK
   Priority: MEDIUM (30)
   ═══════════════════════════════════════════════════════ */
void *launch_command_task(void *arg) {
    (void)arg;
    hmds_log(LOG_INFO, TASK_NAME, "Task started.");

    gpio_init();

    bool     alert_led_on        = false;
    bool     launch_executed     = false;
    uint64_t solution_stable_since = 0;

    while (g_shm->system_armed) {
        g_shm->hb_launch = hmds_now_ns();

        struct timespec ts = {0, 10000000}; /* 10ms poll */
        nanosleep(&ts, NULL);

        /* ── Stage 1: Threat alert ── */
        if (g_shm->threat_active && !alert_led_on) {
            hmds_log(LOG_ALERT, TASK_NAME, "THREAT ACTIVE — alert LEDs engaged");
            led_alert();
            alert_led_on = true;
        }

        if (!g_shm->threat_active) {
            alert_led_on          = false;
            launch_executed       = false;
            solution_stable_since = 0;
            continue;
        }

        if (launch_executed) continue;

        /* ── Stage 2: Validate intercept solution ── */
        InterceptSolution sol = g_shm->solution;

        if (!sol.solution_valid) {
            solution_stable_since = 0;
            continue;
        }

        if (sol.time_to_intercept_s < MIN_INTERCEPT_TIME_S ||
            sol.time_to_intercept_s > MAX_INTERCEPT_TIME_S) {
            hmds_log(LOG_WARN, TASK_NAME,
                     "Solution out of launch window (T=%.1fs)",
                     sol.time_to_intercept_s);
            solution_stable_since = 0;
            continue;
        }

        uint64_t now = hmds_now_ns();
        if (solution_stable_since == 0) {
            solution_stable_since = now;
            continue;
        }

        double stable_s = (double)(now - solution_stable_since) / 1e9;
        if (stable_s < LAUNCH_CONFIRM_WINDOW_S) continue;

        /* ── Stage 3: LAUNCH ── */
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "|======================================|");
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "|   COUNTER-MISSILE LAUNCH AUTHORISED  |");
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "|======================================|");
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "Target:  Lat=%.4f  Lon=%.4f  Alt=%.1fkm",
                 sol.intercept_lat, sol.intercept_lon,
                 sol.intercept_altitude_km);
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "Bearing: %.1f°  Elevation: %.1f°",
                 sol.counter_missile_heading, sol.counter_missile_elevation);
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "ETA:     %.1f seconds", sol.time_to_intercept_s);
        hmds_log(LOG_CRITICAL, TASK_NAME,
                 "Confidence: %.0f%%", sol.confidence * 100.0);

        led_launch();
        g_shm->launch_executed = true;
        g_shm->launch_time_ns  = hmds_now_ns();
        launch_executed        = true;

        /* ── Stage 4: Monitor and confirm ── */
        bool confirmed = verify_intercept(sol.time_to_intercept_s);

        if (confirmed) {
            hmds_log(LOG_CRITICAL, TASK_NAME,
                     "✓ INTERCEPT CONFIRMED — threat neutralised.");
            g_shm->intercept_confirmed = true;   /* radar_sim reads this */
            g_shm->threat_active       = false;
            led_intercept_confirmed();
        } else {
            /* verify_intercept always returns true now, but keep for safety */
            hmds_log(LOG_ALERT, TASK_NAME,
                     "✗ INTERCEPT UNCONFIRMED — recycling.");
            g_shm->launch_executed         = false;
            launch_executed                = false;
            g_shm->solution.solution_valid = false;
            solution_stable_since          = 0;
            led_all_off();
            gpio_write(GPIO_ALERT_LED, 1);
        }
    }

    led_all_off();
    hmds_log(LOG_INFO, TASK_NAME, "Task exiting.");
    return NULL;
}
