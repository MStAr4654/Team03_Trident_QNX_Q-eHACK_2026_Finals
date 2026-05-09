#include <stdio.h>
#ifndef __QNX__
#include "qnx_compat.h"
#endif
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "hmds_common.h"

#define RADAR_SIM_FILE   "/tmp/hmds_radar.sim"
#define DETECT_CHID_FILE "/tmp/hmds_detect_chid"
#define UPDATE_MS        100
#define DEG2RAD          (3.14159265358979 / 180.0)
#define EARTH_R          6371.0

static volatile int running = 1;

static void sig_handler(int sig) {
    (void)sig;
    running = 0;
    printf("\n[RADAR SIM] Interrupted. Stopping.\n");
    fflush(stdout);
    exit(0);
}

/* ── Open shared memory (read-only) to watch intercept_confirmed ── */
static HMDSSharedMem *open_shm(void) {
    int fd = shm_open(SHM_NAME, O_RDONLY, 0);
    if (fd == -1) return NULL;
    HMDSSharedMem *shm = mmap(NULL, sizeof(HMDSSharedMem),
                               PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    return (shm == MAP_FAILED) ? NULL : shm;
}

/* ── Send QNX pulse to Detection Task (with Qnet support) ── */
static void send_detection_pulse(int target_node) {
    FILE *fp = fopen(DETECT_CHID_FILE, "r");
    if (!fp) return;
    pid_t pid; int chid;
    if (fscanf(fp, "%d %d", &pid, &chid) != 2) { fclose(fp); return; }
    fclose(fp);
    
    /* Use Qnet (ConnectAttach works for both local and remote nodes) */
    int coid = ConnectAttach(target_node, pid, chid,
                             _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        printf("[RADAR SIM] Qnet: ConnectAttach to node %d failed: %s\n",
               target_node, strerror(errno));
        fflush(stdout);
        return;
    }
    
    MsgSendPulse(coid, -1, _PULSE_CODE_MINAVAIL, 0);
    ConnectDetach(coid);
    
    /* Display Qnet activity counter (works in both single/multi-node) */
    static int qnet_msg_count = 0;
    qnet_msg_count++;
    
    if (qnet_msg_count % 50 == 0) { // Log every 5 seconds
        printf("\n[RADAR SIM] ╔═══════════════════════════════════════════╗\n");
        if (target_node == 0) {
            printf("[RADAR SIM] ║ Qnet: Sent %d pulses to HMDS (local)    ║\n", qnet_msg_count);
            printf("[RADAR SIM] ║ Single-node simulation mode              ║\n");
        } else {
            printf("[RADAR SIM] ║ Qnet: Sent %d pulses to HMDS node %d    ║\n",
                   qnet_msg_count, target_node);
            printf("[RADAR SIM] ║ Cross-node IPC active                    ║\n");
        }
        printf("[RADAR SIM] ╚═══════════════════════════════════════════╝\n");
        fflush(stdout);
    }
}

/* ── Write one radar return to sim file ── */
static void write_radar_return(double lat, double lon, double alt_km,
                                double vel_mach, double heading,
                                double climb_ms) {
    FILE *fp = fopen(RADAR_SIM_FILE, "a");
    if (!fp) return;
    fprintf(fp, "%.6f %.6f %.3f %.3f %.2f %.2f\n",
            lat, lon, alt_km, vel_mach, heading, climb_ms);
    fclose(fp);
}

/*
 * CHECK_INTERCEPT macro — used inside every scenario loop.
 * Polls shared memory for intercept_confirmed flag.
 * Prints the intercept message and returns from the scenario function.
 */
#define CHECK_INTERCEPT(shm, tick)                                          \
    if ((shm) && (shm)->intercept_confirmed) {                             \
        printf("\n╔══════════════════════════════════════════╗\n");        \
        printf("║  *** MISSILE INTERCEPTED ***             ║\n");          \
        printf("║  THREAT NEUTRALISED at T+%3ds            ║\n", (tick)/10); \
        printf("╚══════════════════════════════════════════╝\n\n");        \
        fflush(stdout);                                                     \
        munmap((shm), sizeof(HMDSSharedMem));                              \
        return;                                                             \
    }

/* ════════════════════════════════════════════════════════
   SCENARIO 1: Straight Hypersonic Cruise
   Mach 6.5, constant heading 110°, gentle descent.
   ════════════════════════════════════════════════════════ */
static void scenario_straight(void) {
    printf("┌──────────────────────────────────────────┐\n");
    printf("│  Scenario 1: Straight Hypersonic Cruise  │\n");
    printf("│  Origin:  35.69N 51.39E                  │\n");
    printf("│  Speed:   Mach 6.5  Heading: 110°        │\n");
    printf("│  Press Ctrl+C to stop                    │\n");
    printf("└──────────────────────────────────────────┘\n\n");

    double lat = 35.6892, lon = 51.3890;
    double alt = 55.0, mach = 6.5, hdg = 110.0, climb = -60.0;

    remove(RADAR_SIM_FILE);
    HMDSSharedMem *shm = open_shm();

    int first = 1;
    for (int tick = 0; running; tick++) {
        double vel_km_s = (mach * 343.0) / 1000.0;
        double dt       = UPDATE_MS / 1000.0;
        double hdg_r    = hdg * DEG2RAD;

        lat += (vel_km_s * cos(hdg_r) * dt / EARTH_R) * (180.0 / 3.14159);
        lon += (vel_km_s * sin(hdg_r) * dt /
                (EARTH_R * cos(lat * DEG2RAD))) * (180.0 / 3.14159);
        alt += (climb * dt) / 1000.0;
        if (alt < 20.0) { climb = 0.0; alt = 20.0; }

        write_radar_return(lat, lon, alt, mach, hdg, climb);

        CHECK_INTERCEPT(shm, tick);

        if (first) {
            printf("[RADAR SIM] Initial detection pulse firing...\n");
            send_detection_pulse(QNET_HMDS_NODE);
            first = 0;
        }

        if (tick % 10 == 0)
            printf("[RADAR SIM] T+%3ds | Lat=%.4f Lon=%.4f "
                   "Alt=%.1fkm Mach=%.1f Hdg=%.0f°\n",
                   tick / 10, lat, lon, alt, mach, hdg);

        struct timespec ts = {0, UPDATE_MS * 1000000L};
        nanosleep(&ts, NULL);
    }

    if (shm) munmap(shm, sizeof(HMDSSharedMem));
}

/* ════════════════════════════════════════════════════════
   SCENARIO 2: Manoeuvring HGV
   Starts straight, performs S-turn evasions at T+30s.
   Tests Kalman filter adaptation.
   ════════════════════════════════════════════════════════ */
static void scenario_manoeuvring(void) {
    printf("┌──────────────────────────────────────────┐\n");
    printf("│  Scenario 2: Manoeuvring HGV             │\n");
    printf("│  Evasive S-turns begin at T+30s          │\n");
    printf("│  Tests Kalman filter adaptation          │\n");
    printf("│  Press Ctrl+C to stop                    │\n");
    printf("└──────────────────────────────────────────┘\n\n");

    double lat = 35.6892, lon = 51.3890;
    double alt = 50.0, mach = 7.0, hdg = 110.0, climb = -50.0;

    remove(RADAR_SIM_FILE);
    HMDSSharedMem *shm = open_shm();

    int first = 1;
    for (int tick = 0; running; tick++) {
        double vel_km_s = (mach * 343.0) / 1000.0;
        double dt  = UPDATE_MS / 1000.0;
        double t_s = tick * dt;

        /* Evasive manoeuvres after 30s */
        if (t_s > 30.0) {
            hdg += (20.0 * sin(0.15 * (t_s - 30.0))) * dt;
            if (hdg >= 360.0) hdg -= 360.0;
            if (hdg <    0.0) hdg += 360.0;
        }

        double hdg_r = hdg * DEG2RAD;
        lat += (vel_km_s * cos(hdg_r) * dt / EARTH_R) * (180.0 / 3.14159);
        lon += (vel_km_s * sin(hdg_r) * dt /
                (EARTH_R * cos(lat * DEG2RAD))) * (180.0 / 3.14159);
        alt += (climb * dt) / 1000.0;
        if (alt < 15.0) { climb = 0.0; alt = 15.0; }

        write_radar_return(lat, lon, alt, mach, hdg, climb);

        CHECK_INTERCEPT(shm, tick);

        if (first) { send_detection_pulse(QNET_HMDS_NODE); first = 0; }

        if (tick % 10 == 0)
            printf("[RADAR SIM] T+%3.0fs | Lat=%.4f Lon=%.4f "
                   "Alt=%.1fkm Hdg=%.1f° %s\n",
                   t_s, lat, lon, alt, hdg,
                   t_s > 30.0 ? "[MANOEUVRING]" : "         ");

        struct timespec ts = {0, UPDATE_MS * 1000000L};
        nanosleep(&ts, NULL);
    }

    if (shm) munmap(shm, sizeof(HMDSSharedMem));
}

/* ════════════════════════════════════════════════════════
   SCENARIO 3: Cluster Munition Deployment
   Primary warhead releases submunitions at T+45s.
   Demonstrates multi-track architectural readiness.
   ════════════════════════════════════════════════════════ */
static void scenario_cluster(void) {
    printf("┌──────────────────────────────────────────┐\n");
    printf("│  Scenario 3: Cluster Munition Deploy     │\n");
    printf("│  Separation event at T+45s               │\n");
    printf("│  Multi-track architecture demo           │\n");
    printf("│  Press Ctrl+C to stop                    │\n");
    printf("└──────────────────────────────────────────┘\n\n");

    double lat = 35.6892, lon = 51.3890;
    double alt = 55.0, mach = 6.0, hdg = 110.0, climb = -70.0;

    remove(RADAR_SIM_FILE);
    HMDSSharedMem *shm = open_shm();

    int first = 1;
    for (int tick = 0; running; tick++) {
        double vel_km_s = (mach * 343.0) / 1000.0;
        double dt  = UPDATE_MS / 1000.0;
        double t_s = tick * dt;
        double hdg_r = hdg * DEG2RAD;

        lat += (vel_km_s * cos(hdg_r) * dt / EARTH_R) * (180.0 / 3.14159);
        lon += (vel_km_s * sin(hdg_r) * dt /
                (EARTH_R * cos(lat * DEG2RAD))) * (180.0 / 3.14159);
        alt += (climb * dt) / 1000.0;
        if (alt < 10.0) { climb = 0.0; alt = 10.0; }

        write_radar_return(lat, lon, alt, mach, hdg, climb);

        CHECK_INTERCEPT(shm, tick);

        if (first) { send_detection_pulse(QNET_HMDS_NODE); first = 0; }

        if (tick % 10 == 0)
            printf("[RADAR SIM] T+%3.0fs | Primary: "
                   "Lat=%.4f Lon=%.4f Alt=%.1fkm\n",
                   t_s, lat, lon, alt);

        /* Cluster separation event */
        if (tick == 450) {
            printf("\n╔══════════════════════════════════════════╗\n");
            printf("║  *** CLUSTER SEPARATION EVENT ***        ║\n");
            printf("║  Multiple submunitions detected.         ║\n");
            printf("║  Multi-track intercept architecture      ║\n");
            printf("║  would spawn additional task instances.  ║\n");
            printf("╚══════════════════════════════════════════╝\n\n");
        }

        struct timespec ts = {0, UPDATE_MS * 1000000L};
        nanosleep(&ts, NULL);
    }

    if (shm) munmap(shm, sizeof(HMDSSharedMem));
}

/* ════════════════════════════════════════════════════════
   MAIN — runs all 3 scenarios sequentially unless a
   specific one is requested via argv[1].
   ════════════════════════════════════════════════════════ */
int main(int argc, char *argv[]) {
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    printf("╔══════════════════════════════════════════╗\n");
    printf("║    HMDS RADAR SIMULATOR                  ║\n");
    printf("║    Team Trident | Q-eHACK 2026           ║\n");
    printf("╚══════════════════════════════════════════╝\n");
    printf("║ Qnet Config:                             ║\n");
    printf("║   Target HMDS Node: %d                    ║\n", QNET_HMDS_NODE);
    if (QNET_HMDS_NODE == 0) {
        printf("║   Mode: Single-Node Simulation           ║\n");
    } else {
        printf("║   Mode: Multi-Node Distributed           ║\n");
    }
    printf("╚══════════════════════════════════════════╝\n\n");
    fflush(stdout);

    int scenario = (argc > 1) ? atoi(argv[1]) : 0;

    if (scenario == 1) {
        scenario_straight();
    } else if (scenario == 2) {
        scenario_manoeuvring();
    } else if (scenario == 3) {
        scenario_cluster();
    } else {
        /* Run all 3 scenarios sequentially */
        printf("Running all 3 scenarios sequentially.\n");
        printf("(Pass 1, 2, or 3 as argument to run a specific one)\n\n");

        printf("═══ SCENARIO 1 of 3 ═══\n\n");
        scenario_straight();
        if (!running) goto done;

        /* Reset intercept state between scenarios */
        {
            int fd = shm_open(SHM_NAME, O_RDWR, 0);
            if (fd != -1) {
                HMDSSharedMem *shm = mmap(NULL, sizeof(HMDSSharedMem),
                                          PROT_READ | PROT_WRITE,
                                          MAP_SHARED, fd, 0);
                close(fd);
                if (shm != MAP_FAILED) {
                    shm->intercept_confirmed = false;
                    shm->threat_active       = false;
                    shm->launch_executed     = false;
                    munmap(shm, sizeof(HMDSSharedMem));
                }
            }
        }

        printf("\n\n═══ SCENARIO 2 of 3 ═══\n\n");
        sleep(3); /* brief pause between scenarios */
        scenario_manoeuvring();
        if (!running) goto done;

        /* Reset again */
        {
            int fd = shm_open(SHM_NAME, O_RDWR, 0);
            if (fd != -1) {
                HMDSSharedMem *shm = mmap(NULL, sizeof(HMDSSharedMem),
                                          PROT_READ | PROT_WRITE,
                                          MAP_SHARED, fd, 0);
                close(fd);
                if (shm != MAP_FAILED) {
                    shm->intercept_confirmed = false;
                    shm->threat_active       = false;
                    shm->launch_executed     = false;
                    munmap(shm, sizeof(HMDSSharedMem));
                }
            }
        }

        printf("\n\n═══ SCENARIO 3 of 3 ═══\n\n");
        sleep(3);
        scenario_cluster();
    }

done:
    printf("\n[RADAR SIM] All scenarios complete.\n");
    return 0;
}

