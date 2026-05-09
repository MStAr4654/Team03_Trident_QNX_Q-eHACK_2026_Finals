#ifndef HMDS_COMMON_H
#define HMDS_COMMON_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __QNX__
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/iofunc.h>
#else
#include "qnx_compat.h"
#endif

/* ─────────────────────────────────────────────────────────────────
   PROJECT : Hypersonic Missile Defense System (HMDS)
   PLATFORM: QNX Neutrino RTOS — x86_64 (Intel/AMD PC / server)
             (previously aarch64 / Raspberry Pi 4)
   TEAM    : Trident | Q-eHACK 2026
   ─────────────────────────────────────────────────────────────────
   QNX Core Concepts used in this system (see Section 20):
   ┌──────────────────────┬──────────────────────────────────────┐
   │ QNX Concept          │ Role in HMDS                         │
   ├──────────────────────┼──────────────────────────────────────┤
   │ Microkernel          │ Reliability — tasks isolated/restart │
   │ Message Passing      │ Deterministic IPC between tasks      │
   │ Real-Time Scheduling │ Priority-based preemptive scheduling │
   │ Interrupt Handling   │ Sensor responsiveness via pulses     │
   │ Resource Managers    │ Device abstraction (GPIO, radar)     │
   │ Qnet                 │ Distributed operation (ND_LOCAL_NODE)│
   │ APS                  │ CPU guarantee for critical tasks     │
   │ Memory Protection    │ Safety — isolated address spaces     │
   │ Multithreading       │ Parallel task pipeline               │
   │ Timers               │ 100ms deterministic control loop     │
   │ Networking           │ Secure radar feed communication      │
   │ Fault Tolerance      │ Watchdog restart on task failure     │
   └──────────────────────┴──────────────────────────────────────┘
   ───────────────────────────────────────────────────────────────── */

/* ── Pulse codes (QNX: Message Passing) ── */
#define PULSE_MISSILE_DETECTED      _PULSE_CODE_MINAVAIL
#define PULSE_TRACK_UPDATE          (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_INTERCEPT_SOLUTION    (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_LAUNCH_COMMAND        (_PULSE_CODE_MINAVAIL + 3)
#define PULSE_HEARTBEAT             (_PULSE_CODE_MINAVAIL + 4)
#define PULSE_TIMER_TICK            (_PULSE_CODE_MINAVAIL + 5)

/* ── Task priorities (QNX: Real-Time Scheduling + APS) ── */
#define PRI_WATCHDOG                10   /* Fault Tolerance monitor    */
#define PRI_LOGGER                  12   /* Background logging         */
#define PRI_LAUNCH_COMMAND          30   /* Launch authority           */
#define PRI_DETECTION               45   /* Sensor fusion              */
#define PRI_TRACKING                45   /* Kalman track loop          */
#define PRI_INTERCEPT_CALC          50   /* Highest — intercept solver */

/* ── Control loop (QNX: Timers — Accurate tracking) ── */
#define CONTROL_LOOP_MS             100
#define CONTROL_LOOP_NS             (CONTROL_LOOP_MS * 1000000LL)

/* ── Watchdog (QNX: Fault Tolerance — High availability) ── */
#define WATCHDOG_INTERVAL_MS        500
#define WATCHDOG_MISS_LIMIT         2

/* ── Qnet Configuration (QNX: Distributed Computing) ── */
/* SINGLE-NODE MODE: Both processes run on same node (node 0)
 * For two-node demo: Set QNET_RADAR_NODE=1, QNET_HMDS_NODE=0 */
#define QNET_RADAR_NODE             1    /* Radar node (0=local, 1=remote) */
#define QNET_HMDS_NODE              0    /* Local HMDS node (ND_LOCAL_NODE) */
#define QNET_RADAR_CHANNEL_NAME     "/dev/hmds/radar_feed"

/* ── Shared memory name (QNX: Memory Protection) ── */
#define SHM_NAME                    "/hmds_shared"

/* ── GPIO pin map (QNX: Resource Managers — Device abstraction) ── */
#define GPIO_LAUNCH_LED             17
#define GPIO_INTERCEPT_LED          27
#define GPIO_ALERT_LED              22

/* ── Physics constants ── */
#define EARTH_RADIUS_KM             6371.0
#define GRAVITY_MS2                 9.81
#define MACH_1_MS                   343.0
#define MIN_INTERCEPT_ALTITUDE_KM   10.0
#define MAX_THREAT_RANGE_KM         2000.0

/* ── Missile State (updated by Detection + Tracking tasks) ── */
typedef struct {
    double   lat;
    double   lon;
    double   altitude_km;
    double   velocity_mach;
    double   heading_deg;
    double   climb_rate_ms;
    uint64_t timestamp_ns;
    bool     is_manoeuvring;
    double   accel_lateral_g;
} MissileState;

/* ── Intercept Solution (computed by InterceptCalc task) ── */
typedef struct {
    double   intercept_lat;
    double   intercept_lon;
    double   intercept_altitude_km;
    double   time_to_intercept_s;
    double   counter_missile_heading;
    double   counter_missile_elevation;
    double   confidence;
    bool     solution_valid;
    uint64_t computed_at_ns;
} InterceptSolution;

/*
 * Shared Memory Block — QNX Concept: Memory Protection
 * All volatile fields are written by exactly one task and
 * read by others; true atomic protection requires QNX
 * pulse-based handshake in production (omitted in sim).
 */
typedef struct {
    volatile bool       threat_active;
    volatile uint64_t   detection_time_ns;
    MissileState        missile;
    InterceptSolution   solution;
    volatile bool       launch_executed;
    volatile uint64_t   launch_time_ns;
    volatile bool       intercept_confirmed;
    /* ── Heartbeat timestamps (QNX: Fault Tolerance) ── */
    volatile uint64_t   hb_detection;
    volatile uint64_t   hb_tracking;
    volatile uint64_t   hb_intercept;
    volatile uint64_t   hb_launch;
    volatile bool       system_armed;
    volatile int        loop_count;
    /* ── Qnet network status ── */
    volatile int        radar_node_id;       /* Remote radar node (Qnet) */
    volatile bool       qnet_active;         /* Qnet link status */
    volatile uint64_t   last_radar_pulse_ns; /* Last radar data received */
} HMDSSharedMem;

/* ── Message Types (QNX: Message Passing) ── */
typedef enum {
    MSG_MISSILE_PARAMS = 1,
    MSG_TRACK_DATA,
    MSG_INTERCEPT_READY,
    MSG_LAUNCH_CONFIRM,
    MSG_HEARTBEAT,
    MSG_SYSTEM_STATUS,
} MsgType;

typedef struct {
    uint16_t type;
    uint16_t seq;
    uint32_t flags;
    union {
        MissileState      missile;
        InterceptSolution solution;
        uint64_t          timestamp_ns;
        char              text[128];
    } payload;
} HMDSMessage;

/* ── Logging severity ── */
typedef enum {
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARN,
    LOG_ALERT,
    LOG_CRITICAL,
} LogLevel;

void        hmds_log(LogLevel level, const char *task,
                     const char *fmt, ...);
uint64_t    hmds_now_ns(void);
const char *level_str(LogLevel level);

/* ── Qnet Helper Functions ── */
int         qnet_connect_to_node(int node_id, int chid);
int         qnet_send_missile_data(int coid, const MissileState *missile);
int         qnet_get_remote_node_id(const char *node_name);
bool        qnet_is_node_alive(int node_id);

/* ── QNX Concepts summary (runtime display) ── */
void        hmds_print_qnx_concepts(void);

#endif /* HMDS_COMMON_H */
