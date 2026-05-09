#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>

#include "hmds_common.h"

extern HMDSSharedMem *g_shm;

#define TASK_NAME           "DETECTION"
#define RADAR_SIM_FILE      "/tmp/hmds_radar.sim"
#define DETECTION_POLL_MS   50

typedef struct {
    double lat;
    double lon;
    double altitude_km;
    double velocity_mach;
    double heading_deg;
    double climb_rate_ms;
} RadarDetection;

static int parse_radar_line(const char *line, RadarDetection *det) {
    return (sscanf(line, "%lf %lf %lf %lf %lf %lf",
                   &det->lat, &det->lon,
                   &det->altitude_km,
                   &det->velocity_mach,
                   &det->heading_deg,
                   &det->climb_rate_ms) == 6) ? 0 : -1;
}

static int read_radar_sim(RadarDetection *det) {
    FILE *fp = fopen(RADAR_SIM_FILE, "r");
    if (!fp) return -1;
    char line[256], last[256] = {0};
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != '#' && strlen(line) > 5)
            strncpy(last, line, sizeof(last) - 1);
    }
    fclose(fp);
    if (strlen(last) == 0) return -1;
    return parse_radar_line(last, det);
}

static double compute_detection_confidence(const RadarDetection *det) {
    double confidence = 1.0;
    if (det->altitude_km >= 20.0 && det->altitude_km <= 60.0) {
        confidence *= 0.65;
        hmds_log(LOG_WARN, TASK_NAME,
                 "Target in HGV altitude band (%.1f km) — "
                 "reduced radar confidence, switching to IR fusion",
                 det->altitude_km);
    }
    if (det->velocity_mach >= 5.0)      confidence *= 1.0;
    else if (det->velocity_mach >= 3.0) confidence *= 0.85;
    else                                confidence *= 0.5;
    if (confidence > 1.0) confidence = 1.0;
    if (confidence < 0.0) confidence = 0.0;
    return confidence;
}

static void publish_detection(const RadarDetection *det,
                               double confidence) {
    g_shm->threat_active            = true;
    g_shm->detection_time_ns        = hmds_now_ns();
    g_shm->missile.lat              = det->lat;
    g_shm->missile.lon              = det->lon;
    g_shm->missile.altitude_km      = det->altitude_km;
    g_shm->missile.velocity_mach    = det->velocity_mach;
    g_shm->missile.heading_deg      = det->heading_deg;
    g_shm->missile.climb_rate_ms    = det->climb_rate_ms;
    g_shm->missile.timestamp_ns     = g_shm->detection_time_ns;
    g_shm->missile.is_manoeuvring   = false;
    g_shm->missile.accel_lateral_g  = 0.0;
    
    // Update Qnet radar pulse timestamp
    g_shm->last_radar_pulse_ns = hmds_now_ns();
    
    hmds_log(LOG_ALERT, TASK_NAME,
             "THREAT DETECTED | Lat=%.4f Lon=%.4f Alt=%.1fkm "
             "Vel=Mach%.1f Hdg=%.1f° Confidence=%.0f%% | Radar source: Node %d",
             det->lat, det->lon, det->altitude_km,
             det->velocity_mach, det->heading_deg,
             confidence * 100.0, g_shm->radar_node_id);
}

void *detection_task(void *arg) {
    (void)arg;
    hmds_log(LOG_INFO, TASK_NAME, "Task started. Scanning for threats...");

    bool     first_detection = true;
    uint64_t last_detection_ns = 0;
    static const uint64_t REDETECT_COOLDOWN_NS = 500000000ULL;

    while (g_shm->system_armed) {
        g_shm->hb_detection = hmds_now_ns();

        struct timespec ts = {0, DETECTION_POLL_MS * 1000000L};
        nanosleep(&ts, NULL);

        uint64_t now = hmds_now_ns();
        if (g_shm->threat_active &&
            (now - last_detection_ns) < REDETECT_COOLDOWN_NS)
            continue;

        RadarDetection det;
        if (read_radar_sim(&det) != 0) {
            if (first_detection) {
                det.lat           = 35.6892;
                det.lon           = 51.3890;
                det.altitude_km   = 45.0;
                det.velocity_mach = 6.5;
                det.heading_deg   = 110.0;
                det.climb_rate_ms = -80.0;
                hmds_log(LOG_WARN, TASK_NAME,
                         "No radar sim file — using built-in scenario");
                first_detection = false;
            } else {
                continue;
            }
        }

        double confidence = compute_detection_confidence(&det);
        if (confidence < 0.4) {
            hmds_log(LOG_DEBUG, TASK_NAME,
                     "Low-confidence contact (%.0f%%) — ignoring",
                     confidence * 100.0);
            continue;
        }

        publish_detection(&det, confidence);
        last_detection_ns = hmds_now_ns();
    }

    hmds_log(LOG_INFO, TASK_NAME, "Task exiting.");
    return NULL;
}
