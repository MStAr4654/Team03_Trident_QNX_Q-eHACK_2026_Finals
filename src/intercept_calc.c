#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include "hmds_common.h"
#include "kalman.h"

extern HMDSSharedMem *g_shm;

#define TASK_NAME   "INTERCEPT"
#define D2R         (M_PI / 180.0)
#define R2D         (180.0 / M_PI)

// Counter-missile performance
#define C_VEL_MACH  3.5
#define C_VEL_KMS   (C_VEL_MACH * MACH_1_MS / 1000.0)
#define C_MAX_KM    3000.0
#define C_MIN_CONF  0.55

/*
 * LAUNCH SITE — placed ahead of the missile's flight path.
 * Missile origin: 35.69N 51.39E, heading 110° (ESE toward India).
 * At Mach 6.5 (~2.23 km/s), it reaches lon ~56E in ~90 seconds.
 * We place the launch site at 34.5N 56.0E — directly in its path,
 * slightly south so the counter-missile intercepts head-on.
 */
#define L_LAT   34.5000
#define L_LON   56.0000
#define L_ALT   0.5

// Great-circle distance (km)
static double gc(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * D2R;
    double dlon = (lon2 - lon1) * D2R;
    double a = sin(dlat/2)*sin(dlat/2) +
               cos(lat1*D2R)*cos(lat2*D2R)*sin(dlon/2)*sin(dlon/2);
    return EARTH_RADIUS_KM * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// Bearing (degrees)
static double brg(double lat1, double lon1, double lat2, double lon2) {
    double dlon = (lon2 - lon1) * D2R;
    double y = sin(dlon) * cos(lat2 * D2R);
    double x = cos(lat1*D2R)*sin(lat2*D2R) -
               sin(lat1*D2R)*cos(lat2*D2R)*cos(dlon);
    return fmod(atan2(y, x) * R2D + 360.0, 360.0);
}

// Elevation angle (degrees above horizontal)
static double elev(double range_km, double alt_diff_km) {
    return (range_km < 0.001) ? 90.0 : atan2(alt_diff_km, range_km) * R2D;
}

/*
 * Binary-search intercept solver.
 * Finds time T such that the counter-missile (travelling at C_VEL_KMS)
 * can reach the missile's predicted position in exactly T seconds.
 * Uses the missile's current velocity vector for linear propagation.
 */
static bool solve(const MissileState *m, const KalmanFilter *kf,
                  InterceptSolution *s) {
    double rng = gc(L_LAT, L_LON, m->lat, m->lon);

    if (rng > C_MAX_KM) {
        hmds_log(LOG_WARN, TASK_NAME,
                 "Threat at %.0f km — beyond counter range (%.0f km)",
                 rng, C_MAX_KM);
        s->solution_valid = false;
        return false;
    }

    // Decompose missile velocity (km/s)
    double spd = m->velocity_mach * MACH_1_MS / 1000.0;
    double hr  = m->heading_deg * D2R;
    double vx  = spd * sin(hr);             // East  km/s
    double vy  = spd * cos(hr);             // North km/s
    double vz  = m->climb_rate_ms / 1000.0; // Up    km/s

    // Search bounds: [0, 6× time to cover current range]
    double lo = 0.0, hi = (rng / C_VEL_KMS) * 6.0;
    int found = 0;

    for (int i = 0; i < 200; i++) {
        double t = (lo + hi) / 2.0;

        // Predict missile position at time t
        double tlat = m->lat + (vy * t / EARTH_RADIUS_KM) * R2D;
        double tlon = m->lon + (vx * t /
                      (EARTH_RADIUS_KM * cos(m->lat * D2R))) * R2D;
        double talt = m->altitude_km + vz * t;

        // Range from launch site to predicted intercept point
        double d   = gc(L_LAT, L_LON, tlat, tlon);
        double dh  = talt - L_ALT;
        double r3  = sqrt(d*d + dh*dh);
        double tn  = r3 / C_VEL_KMS;
        double err = tn - t;

        if (fabs(err) < 1.0) {
            s->intercept_lat            = tlat;
            s->intercept_lon            = tlon;
            s->intercept_altitude_km    = talt;
            s->time_to_intercept_s      = t;
            s->counter_missile_heading  = brg(L_LAT, L_LON, tlat, tlon);
            s->counter_missile_elevation = elev(d, dh);

            s->confidence = 0.95;
            if (kf_manoeuvre_detected(kf))             s->confidence -= 0.20;
            if (talt < MIN_INTERCEPT_ALTITUDE_KM)      s->confidence -= 0.30;
            if (s->confidence < 0.0) s->confidence = 0.0;

            s->solution_valid  = (s->confidence >= C_MIN_CONF);
            s->computed_at_ns  = hmds_now_ns();
            found = 1;
            break;
        }

        if (err > 0) hi = t;
        else         lo = t;
    }

    return found && s->solution_valid;
}

/* =======================================================
   INTERCEPT CALCULATOR TASK
   Priority: HIGHEST (50) — preempts all other tasks
   ======================================================= */
void *intercept_calc_task(void *arg) {
    (void)arg;
    hmds_log(LOG_INFO, TASK_NAME,
             "Task started at HIGHEST priority (%d).", PRI_INTERCEPT_CALC);

    KalmanFilter kf;
    bool     ready = false;
    uint64_t last  = 0;

    while (g_shm->system_armed) {
        g_shm->hb_intercept = hmds_now_ns();

        if (!g_shm->threat_active) {
            struct timespec ts = {0, 10000000};
            nanosleep(&ts, NULL);
            continue;
        }

        uint64_t now = hmds_now_ns();
        if ((now - last) < (uint64_t)CONTROL_LOOP_NS) {
            struct timespec ts = {0, 5000000};
            nanosleep(&ts, NULL);
            continue;
        }
        last = now;

        MissileState m = g_shm->missile;

        if (!ready) {
            double v = m.velocity_mach * MACH_1_MS / 1000.0;
            double h = m.heading_deg * D2R;
            kf_init(&kf, 0, 0, 0, v*sin(h), v*cos(h), 0);
            ready = true;
        }

        kf_predict(&kf, (double)CONTROL_LOOP_MS / 1000.0);
        kf_update(&kf, 0, 0, 0);
        kf_set_manoeuvre_mode(&kf, m.is_manoeuvring ? 1 : 0);

        InterceptSolution s = {0};
        bool ok = solve(&m, &kf, &s);
        g_shm->solution = s;

        if (ok)
            hmds_log(LOG_ALERT, TASK_NAME,
                     "INTERCEPT SOLUTION | T+%.1fs | Alt=%.1fkm | "
                     "Hdg=%.1f° El=%.1f° | Conf=%.0f%% %s",
                     s.time_to_intercept_s, s.intercept_altitude_km,
                     s.counter_missile_heading, s.counter_missile_elevation,
                     s.confidence * 100.0,
                     m.is_manoeuvring ? "[MANOEUVRING]" : "");
        else
            hmds_log(LOG_WARN, TASK_NAME,
                     "No valid intercept solution (confidence=%.0f%%)",
                     s.confidence * 100.0);
    }

    hmds_log(LOG_INFO, TASK_NAME, "Task exiting.");
    return NULL;
}
