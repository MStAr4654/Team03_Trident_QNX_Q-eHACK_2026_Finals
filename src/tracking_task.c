#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>

#include "hmds_common.h"
#include "kalman.h"

extern HMDSSharedMem *g_shm;

#define TASK_NAME   "TRACKING"
#define DEG2RAD     (M_PI / 180.0)
#define RAD2DEG     (180.0 / M_PI)

static double ref_lat = 0.0, ref_lon = 0.0;
static bool   ref_set = false;

static void set_reference(double lat, double lon) {
    ref_lat = lat * DEG2RAD;
    ref_lon = lon * DEG2RAD;
    ref_set = true;
}

static void lla_to_enu(double lat, double lon, double alt_km,
                       double *E, double *N, double *U) {
    double latr = lat * DEG2RAD;
    double lonr = lon * DEG2RAD;
    *E = EARTH_RADIUS_KM * cos(ref_lat) * (lonr - ref_lon);
    *N = EARTH_RADIUS_KM * (latr - ref_lat);
    *U = alt_km;
}

static void enu_to_lla(double E, double N, double U,
                       double *lat, double *lon, double *alt_km) {
    *lat     = ref_lat * RAD2DEG + (N / EARTH_RADIUS_KM) * RAD2DEG;
    *lon     = ref_lon * RAD2DEG +
               (E / (EARTH_RADIUS_KM * cos(ref_lat))) * RAD2DEG;
    *alt_km  = U;
}

static void propagate_ballistic(MissileState *m, double dt_s) {
    double vel_ms  = m->velocity_mach * MACH_1_MS;
    double hdg_rad = m->heading_deg * DEG2RAD;
    double vx      = vel_ms * sin(hdg_rad);
    double vy      = vel_ms * cos(hdg_rad);
    double vz      = m->climb_rate_ms;

    vz -= GRAVITY_MS2 * dt_s;

    double rho_ratio   = exp(-m->altitude_km / 8.5);
    double drag_factor = 1.0 - (0.002 * rho_ratio * dt_s);
    vx *= drag_factor;
    vy *= drag_factor;

    double dx_km = vx * dt_s / 1000.0;
    double dy_km = vy * dt_s / 1000.0;
    double dz_km = vz * dt_s / 1000.0;

    m->lat         += (dy_km / EARTH_RADIUS_KM) * RAD2DEG;
    double clat = cos(m->lat * DEG2RAD);

    if (fabs(clat) < 1e-6)
        clat = 1e-6;

    m->lon += (dx_km / (EARTH_RADIUS_KM * clat)) * RAD2DEG;
    m->altitude_km += dz_km;

    double new_vel     = sqrt(vx*vx + vy*vy + vz*vz);
    m->velocity_mach   = new_vel / MACH_1_MS;
    m->climb_rate_ms   = vz;
    m->heading_deg     = atan2(vx, vy) * RAD2DEG;
    if (m->heading_deg < 0) m->heading_deg += 360.0;
    if (m->altitude_km < 0) m->altitude_km  = 0.0;
}

static bool detect_manoeuvre(const MissileState *prev,
                              const MissileState *curr,
                              double dt_s) {
    double dhdg = curr->heading_deg - prev->heading_deg;
    if (dhdg >  180.0) dhdg -= 360.0;
    if (dhdg < -180.0) dhdg += 360.0;
    double vel_ms      = curr->velocity_mach * MACH_1_MS;
    double omega       = (dhdg * DEG2RAD) / dt_s;
    double lat_accel_g = fabs(vel_ms * omega) / GRAVITY_MS2;
    return (lat_accel_g > 3.0);
}

void *tracking_task(void *arg) {
    (void)arg;
    hmds_log(LOG_INFO, TASK_NAME,
             "tracking_task thread entered.");
    hmds_log(LOG_INFO, TASK_NAME,
             "Task started. Control loop: %dms", CONTROL_LOOP_MS);
    hmds_log(LOG_INFO, TASK_NAME, "100ms deterministic timer armed.");

    KalmanFilter kf;
    bool         kf_initialized = false;
    MissileState prev_state     = {0};
    uint64_t     prev_ts        = 0;

    while (g_shm != NULL &&
           g_shm->system_armed) {
        // 100ms deterministic sleep — replaces QNX pulse timer
        struct timespec ts = {0, CONTROL_LOOP_NS};
        nanosleep(&ts, NULL);

        g_shm->hb_tracking = hmds_now_ns();

        if (!g_shm->threat_active) continue;
        if (g_shm->missile.velocity_mach <= 0.0)
            continue;

        uint64_t now_ns = hmds_now_ns();
        double dt_s = (prev_ts == 0) ? 0.1 :
                      (double)(now_ns - prev_ts) / 1e9;
        prev_ts = now_ns;
        if (dt_s <= 0.0 || dt_s > 1.0) dt_s = 0.1;

        MissileState cur = g_shm->missile;

        if (!kf_initialized) {
            if (!ref_set) set_reference(cur.lat, cur.lon);
            double vel_ms = cur.velocity_mach * MACH_1_MS / 1000.0;
            double hdg_r  = cur.heading_deg * DEG2RAD;
            kf_init(&kf,
                    0.0, 0.0, cur.altitude_km,
                    vel_ms * sin(hdg_r),
                    vel_ms * cos(hdg_r),
                    cur.climb_rate_ms / 1000.0);
            kf_initialized = true;
            prev_state = cur;
            hmds_log(LOG_INFO, TASK_NAME,
                     "Kalman filter initialised. Tracking active.");
            continue;
        }

        kf_predict(&kf, dt_s);
        propagate_ballistic(&cur, dt_s);

        double mE, mN, mU;
        lla_to_enu(cur.lat, cur.lon, cur.altitude_km, &mE, &mN, &mU);
        kf_update(&kf, mE, mN, mU);

        double kx, ky, kz, kvx, kvy, kvz;
        kf_get_position(&kf, &kx, &ky, &kz);
        kf_get_velocity(&kf, &kvx, &kvy, &kvz);

        double smooth_lat, smooth_lon, smooth_alt;
        enu_to_lla(kx, ky, kz, &smooth_lat, &smooth_lon, &smooth_alt);

        double vel_km_s = sqrt(kvx*kvx + kvy*kvy + kvz*kvz);
        double heading  = atan2(kvx, kvy) * RAD2DEG;
        if (heading < 0.0) heading += 360.0;

        bool manoeuving = detect_manoeuvre(&prev_state, &cur, dt_s);
        if (manoeuving != g_shm->missile.is_manoeuvring) {
            hmds_log(manoeuving ? LOG_ALERT : LOG_INFO, TASK_NAME,
                     "Manoeuvre %s! Boosting Kalman process noise.",
                     manoeuving ? "DETECTED" : "ENDED");
            kf_set_manoeuvre_mode(&kf, manoeuving ? 1 : 0);
        }

        g_shm->missile.lat             = smooth_lat;
        g_shm->missile.lon             = smooth_lon;
        g_shm->missile.altitude_km     = smooth_alt;
        g_shm->missile.velocity_mach   = (vel_km_s * 1000.0) / MACH_1_MS;
        g_shm->missile.heading_deg     = heading;
        g_shm->missile.climb_rate_ms   = kvz * 1000.0;
        g_shm->missile.timestamp_ns    = now_ns;
        g_shm->missile.is_manoeuvring  = manoeuving;

        prev_state = cur;
        g_shm->loop_count++;

        hmds_log(LOG_DEBUG, TASK_NAME,
                 "Loop #%d | Lat=%.4f Lon=%.4f Alt=%.1fkm "
                 "Vel=Mach%.2f Hdg=%.1f° %s",
                 g_shm->loop_count,
                 smooth_lat, smooth_lon, smooth_alt,
                 g_shm->missile.velocity_mach, heading,
                 manoeuving ? "[MANOEUVRING]" : "");
    }

    hmds_log(LOG_INFO, TASK_NAME, "Task exiting.");
    return NULL;
}
