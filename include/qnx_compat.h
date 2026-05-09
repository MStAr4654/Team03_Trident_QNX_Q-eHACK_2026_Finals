#ifndef QNX_COMPAT_H
#define QNX_COMPAT_H

/*
 * QNX Neutrino Compatibility Shim — x86_64 Linux Simulation Layer
 *
 * Maps QNX IPC primitives to POSIX equivalents so the HMDS source
 * compiles and runs natively on any x86_64 Linux host.
 *
 * QNX Concepts implemented here:
 *   - Message Passing  : MsgSendPulse / MsgReceivePulse via POSIX pipe
 *   - Interrupt Handling: SIGEV_PULSE_INIT mapped to SIGALRM
 *   - Resource Managers: ChannelCreate/ConnectAttach abstracted
 *   - Qnet            : ND_LOCAL_NODE = 0 (local node only in sim)
 *   - Timers          : TimedReceivePulse with select() timeout
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>

/* ── QNX Pulse constants ── */
#define _PULSE_CODE_MINAVAIL    0
#define ND_LOCAL_NODE           0
#define _NTO_SIDE_CHANNEL       0

/* ── Pulse message type (mirrors QNX _pulse struct) ── */
typedef struct { int code; int value; } _pulse_sim;

/*
 * ChannelCreate — QNX Concept: Message Passing / Resource Managers
 * Simulated via a POSIX pipe pair. The read-end is encoded in the
 * low 16 bits of chid, write-end in the high 16 bits.
 */
static inline int ChannelCreate(int flags) {
    (void)flags;
    int fds[2];
    if (pipe(fds) == -1) return -1;
    return (fds[0] & 0xFFFF) | ((fds[1] & 0xFFFF) << 16);
}

static inline int ChannelDestroy(int chid) {
    close(chid & 0xFFFF);
    close((chid >> 16) & 0xFFFF);
    return 0;
}

/*
 * ConnectAttach — QNX Concept: Message Passing / Qnet
 * Returns the write-end fd of the pipe created by ChannelCreate.
 */
static inline int ConnectAttach(int nd, pid_t pid, int chid,
                                 int idx, int flags) {
    (void)nd; (void)pid; (void)idx; (void)flags;
    return (chid >> 16) & 0xFFFF;
}

static inline int ConnectDetach(int coid) { (void)coid; return 0; }

/*
 * MsgSendPulse — QNX Concept: Message Passing (Deterministic communication)
 * Sends a 1-byte pulse code over the pipe.
 */
static inline int MsgSendPulse(int coid, int priority,
                                int code, int value) {
    (void)priority; (void)value;
    uint8_t c = (uint8_t)code;
    return (int)write(coid, &c, 1);
}

/*
 * MsgReceivePulse — QNX Concept: Message Passing (blocking receive)
 */
static inline int MsgReceivePulse(int chid, void *buf,
                                   int bufsize, void *info) {
    (void)info; (void)bufsize;
    _pulse_sim *p = (_pulse_sim *)buf;
    int rd = chid & 0xFFFF;
    uint8_t c;
    if (read(rd, &c, 1) == 1) { p->code = c; p->value = 0; return 0; }
    return -1;
}

/*
 * TimedReceivePulse — QNX Concept: Timers (Accurate tracking)
 * Blocks up to timeout_ns nanoseconds using POSIX select().
 */
static inline int TimedReceivePulse(int chid, void *buf,
                                     int bufsize, void *info,
                                     uint64_t timeout_ns) {
    (void)info; (void)bufsize;
    int rd = chid & 0xFFFF;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(rd, &rfds);
    struct timeval tv;
    tv.tv_sec  = (long)(timeout_ns / 1000000000ULL);
    tv.tv_usec = (long)((timeout_ns % 1000000000ULL) / 1000ULL);
    int rc = select(rd + 1, &rfds, NULL, NULL, &tv);
    if (rc == 0) { errno = ETIMEDOUT; return -1; }
    if (rc < 0)  return -1;
    _pulse_sim *p = (_pulse_sim *)buf;
    uint8_t c;
    if (read(rd, &c, 1) == 1) { p->code = c; p->value = 0; return 0; }
    return -1;
}

/*
 * SIGEV_PULSE_INIT — QNX Concept: Interrupt Handling / Timers
 * Maps the QNX async event to SIGALRM in simulation mode.
 */
#define SIGEV_PULSE_INIT(sev, coid, pri, code, val)  \
    memset((sev), 0, sizeof(*(sev)));                \
    (sev)->sigev_notify = SIGEV_SIGNAL;              \
    (sev)->sigev_signo  = SIGALRM;

#endif /* QNX_COMPAT_H */
