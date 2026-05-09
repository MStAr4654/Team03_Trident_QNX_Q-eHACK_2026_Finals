/*
 * qnx_concepts.c — Section 20: QNX Core Concepts Summary
 *
 * This module implements the runtime display of all 12 QNX Neutrino
 * core concepts and their roles in the HMDS defence system.
 *
 * Each concept is also annotated throughout the codebase at the
 * point where it is actively used.
 */

#include <stdio.h>
#include "hmds_common.h"

/*
 * QNX Concept: Microkernel — Reliability
 *
 * The QNX microkernel runs only the essential scheduler and IPC in
 * kernel space. All drivers and services (radar resource manager,
 * GPIO manager) run as user-space processes. If a driver crashes,
 * only that process dies; the kernel and other tasks survive.
 * In HMDS this means a faulty radar driver cannot bring down the
 * tracking or intercept tasks.
 */
static void concept_microkernel(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[1] Microkernel       → Reliability: drivers in user space; "
        "kernel crash-isolated from sensor/tracking tasks");
}

/*
 * QNX Concept: Message Passing — Deterministic communication
 *
 * QNX uses synchronous MsgSend/MsgReceive for IPC. Timing is
 * bounded and priority-aware — the sender blocks until the receiver
 * replies, giving hard real-time guarantees. HMDS uses pulse-based
 * message passing (MsgSendPulse) between Detection→Tracking→Intercept.
 */
static void concept_message_passing(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[2] Message Passing   → Deterministic IPC: "
        "MsgSendPulse used for Detection→Tracking→Intercept pipeline");
}

/*
 * QNX Concept: Real-Time Scheduling — Fast response
 *
 * QNX uses fixed-priority preemptive scheduling. A higher-priority
 * thread immediately preempts a lower one. HMDS assigns:
 *   PRI 50  InterceptCalc  (highest — launch decision)
 *   PRI 45  Detection / Tracking
 *   PRI 30  LaunchCommand
 *   PRI 10  Watchdog (lowest — background health monitor)
 */
static void concept_realtime_scheduling(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[3] Real-Time Sched   → Fast response: "
        "Intercept(50) > Detect/Track(45) > Launch(30) > Watchdog(10)");
}

/*
 * QNX Concept: Interrupt Handling — Sensor responsiveness
 *
 * Hardware interrupts attach to ISR threads via InterruptAttach().
 * In HMDS, radar sensor events arrive as QNX pulses (async signal
 * to the Detection task channel), ensuring sub-millisecond latency
 * from hardware interrupt to software handler with no polling.
 */
static void concept_interrupt_handling(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[4] Interrupt Handling → Sensor responsiveness: "
        "Radar IRQ → QNX pulse → Detection task (no polling)");
}

/*
 * QNX Concept: Resource Managers — Device abstraction
 *
 * QNX resource managers expose hardware as file-like paths in the
 * namespace (e.g. /dev/radar, /dev/gpio). Clients use open/read/write.
 * HMDS uses this for GPIO LED control (/sys/class/gpio) and the
 * radar data feed, abstracting hardware differences between
 * x86_64 PC and embedded targets.
 */
static void concept_resource_managers(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[5] Resource Managers → Device abstraction: "
        "GPIO LEDs and radar feed accessed as /dev/* paths");
}

/*
 * QNX Concept: Qnet — Distributed operation
 *
 * Qnet is QNX's transparent distributed networking layer. A process
 * on node A can MsgSend to a thread on node B using the same API
 * as local IPC. In HMDS, the radar simulator runs on a separate node
 * (QNET_RADAR_NODE=1) and sends missile detection pulses to the main
 * HMDS node (QNET_HMDS_NODE=0) via ConnectAttach(node_id, ...).
 * This demonstrates distributed sensor fusion across Qnet cluster.
 */
static void concept_qnet(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[6] Qnet              → Distributed operation: "
        "Radar on node %d sends pulses to HMDS on node %d via Qnet IPC",
        QNET_RADAR_NODE, QNET_HMDS_NODE);
}

/*
 * QNX Concept: APS (Adaptive Partitioning Scheduler) — CPU guarantee
 *
 * APS partitions CPU budget across groups, guaranteeing each partition
 * a minimum CPU share even under overload. HMDS would assign:
 *   PARTITION "intercept"  → 40% CPU guaranteed
 *   PARTITION "tracking"   → 30% CPU guaranteed
 *   PARTITION "housekeep"  → 30% CPU (watchdog, logging)
 * This prevents a runaway detection loop from starving the intercept solver.
 */
static void concept_aps(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[7] APS               → CPU guarantee: "
        "Intercept=40%% / Tracking=30%% / Housekeeping=30%% partition budget");
}

/*
 * QNX Concept: Memory Protection — Safety
 *
 * QNX enforces per-process virtual address spaces (MMU-based).
 * A buffer overflow in the radar driver cannot corrupt the Kalman
 * filter state or the launch command logic. Shared memory (SHM_NAME)
 * is the only intentional shared region, explicitly mapped with mmap().
 */
static void concept_memory_protection(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[8] Memory Protection → Safety: "
        "Each task isolated; shared state only via /hmds_shared mmap region");
}

/*
 * QNX Concept: Multithreading — Parallel processing
 *
 * HMDS runs five concurrent tasks as pthreads:
 *   Detection  → polls radar sim file, publishes threat
 *   Tracking   → Kalman filter update loop (100ms)
 *   Intercept  → binary-search intercept solver
 *   Launch     → monitors solution, fires GPIO
 *   Watchdog   → heartbeat monitor, restarts dead tasks
 * All run truly in parallel on x86_64 multi-core hardware.
 */
static void concept_multithreading(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[9] Multithreading    → Parallel processing: "
        "5 tasks: Detection|Tracking|Intercept|Launch|Watchdog");
}

/*
 * QNX Concept: Timers — Accurate tracking
 *
 * QNX provides POSIX timers with nanosecond resolution backed by
 * hardware timers. HMDS uses a 100ms nanosleep-based control loop
 * (CONTROL_LOOP_NS) in Tracking and Intercept tasks. On real QNX,
 * timer_create() + MsgReceivePulse() gives hard-deadline wakeups.
 */
static void concept_timers(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[10] Timers            → Accurate tracking: "
        "100ms deterministic control loop (CONTROL_LOOP_NS = 100000000)");
}

/*
 * QNX Concept: Networking — Secure communication
 *
 * QNX io-pkt provides a full TCP/IP stack with optional encryption.
 * In a fielded HMDS, radar returns would arrive over a hardened
 * encrypted network link (TLS/DTLS). The sim uses a local file
 * (/tmp/hmds_radar.sim) in place of the network feed but the
 * detection_task interface is network-transport-agnostic.
 */
static void concept_networking(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[11] Networking        → Secure communication: "
        "Radar feed over encrypted io-pkt link (sim: /tmp/hmds_radar.sim)");
}

/*
 * QNX Concept: Fault Tolerance — High availability
 *
 * The Watchdog task monitors heartbeat timestamps written by each task
 * into shared memory. If a task misses WATCHDOG_MISS_LIMIT (2) consecutive
 * heartbeat windows, the watchdog restarts it via pthread_create().
 * This maps directly to QNX's process manager restart policy and
 * High Availability Manager (HAM) in production.
 */
static void concept_fault_tolerance(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "[12] Fault Tolerance   → High availability: "
        "Watchdog restarts any task missing %d heartbeats (interval=%dms)",
        WATCHDOG_MISS_LIMIT, WATCHDOG_INTERVAL_MS);
}

/* ══════════════════════════════════════════════════════
   PUBLIC: print all 12 QNX concepts at system startup
   ══════════════════════════════════════════════════════ */
void hmds_print_qnx_concepts(void) {
    hmds_log(LOG_INFO, "CONCEPTS",
        "┌──────────────────────────────────────────────────────────────────┐");
    hmds_log(LOG_INFO, "CONCEPTS",
        "│          Section 20 — QNX Core Concepts in HMDS                 │");
    hmds_log(LOG_INFO, "CONCEPTS",
        "├──────────────────────────────────────────────────────────────────┤");

    concept_microkernel();
    concept_message_passing();
    concept_realtime_scheduling();
    concept_interrupt_handling();
    concept_resource_managers();
    concept_qnet();
    concept_aps();
    concept_memory_protection();
    concept_multithreading();
    concept_timers();
    concept_networking();
    concept_fault_tolerance();

    hmds_log(LOG_INFO, "CONCEPTS",
        "└──────────────────────────────────────────────────────────────────┘");
}
