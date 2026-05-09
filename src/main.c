/*******************************************************************************
 * @file    main.c
 * @brief   Main entry point and process launcher for HMDS
 * @author  Team Trident
 * @date    Q-eHACK 2026
 * @version 2.1
 *
 * PURPOSE:
 *   This file contains the main() function which serves as the entry point for
 *   the Hypersonic Missile Defense System. It is responsible for:
 *   - Initializing shared memory for inter-process communication
 *   - Launching all five defense tasks as separate processes
 *   - Monitoring task health and restarting failed components
 *
 * ARCHITECTURE:
 *   The HMDS uses a multi-process architecture where each defense function
 *   runs as an independent process. This provides:
 *   - Fault isolation (one crash doesn't bring down the whole system)
 *   - Memory protection (MMU keeps each process separate)
 *   - Real-time scheduling (QNX can prioritize critical tasks)
 *
 * PROCESSES LAUNCHED:
 *   1. Detection Task   - Monitors radar for incoming threats
 *   2. Tracking Task    - Predicts missile trajectory using Kalman filter
 *   3. Intercept Task   - Calculates optimal intercept point
 *   4. Launch Task      - Fires counter-missile when ready
 *   5. Watchdog Task    - Monitors all tasks and restarts failures
 *
 * QNX CONCEPTS USED:
 *   - Multi-process architecture (each task is separate process)
 *   - Shared memory (all processes read/write common state)
 *   - Process spawning (posix_spawn creates child processes)
 *   - Signal handling (SIGTERM/SIGINT for graceful shutdown)
 *   - Memory protection (each process has isolated address space)
 *
 ******************************************************************************/

/*-----------------------------------------------------------------------------
 * SYSTEM HEADER FILES
 * These are standard C and POSIX headers required for the program to function
 *---------------------------------------------------------------------------*/
#include <stdio.h>       // Standard input/output: printf, fprintf, perror
#include <stdlib.h>      // Standard library: EXIT_SUCCESS, EXIT_FAILURE
#include <string.h>      // String operations: strcmp, memset, strerror
#include <unistd.h>      // POSIX API: sleep, close, readlink
#include <errno.h>       // Error number definitions for system calls
#include <spawn.h>       // Process spawning: posix_spawn and related functions
#include <sys/wait.h>    // Process waiting: waitpid, wait, WNOHANG
#include <sys/mman.h>    // Memory management: mmap, munmap, shm_open
#include <sys/stat.h>    // File status: permission bits like S_IRUSR
#include <fcntl.h>       // File control: O_CREAT, O_RDWR, O_RDONLY flags
#include <signal.h>      // Signal handling: signal(), kill(), SIGTERM

/*-----------------------------------------------------------------------------
 * PROJECT HEADER FILES
 * Our custom headers defining system-wide types and constants
 *---------------------------------------------------------------------------*/
#include "hmds_common.h"  // All HMDS type definitions, constants, and prototypes

/*=============================================================================
 * GLOBAL VARIABLES
 *===========================================================================*/

/**
 * @brief Pointer to shared memory region used by all processes
 *
 * This global variable points to a region of memory that is mapped into the
 * address space of EVERY process in the system. It contains:
 * - Current missile state (position, velocity, heading)
 * - Intercept solution (where and when to fire)
 * - Launch status (has counter-missile been fired?)
 * - Heartbeat timestamps (used by watchdog to detect crashes)
 *
 * WHY IT'S GLOBAL:
 * Making this global allows all tasks to easily access shared state without
 * passing pointers through every function call. This is a common pattern in
 * embedded systems where simplicity and performance matter.
 *
 * THREAD SAFETY:
 * Multiple processes can read/write this simultaneously. We use 'volatile'
 * keyword on fields that change frequently to prevent compiler optimizations
 * that could cache stale values.
 */
HMDSSharedMem *g_shm = NULL;  // NULL until shm_init() succeeds

/**
 * @brief Environment variables passed to child processes
 *
 * When we spawn child processes using posix_spawn(), they need access to
 * environment variables (like PATH, HOME, etc.). This external variable
 * is provided by the C runtime and contains all environment settings.
 *
 * USAGE:
 * We pass this to posix_spawn() so child processes inherit our environment.
 * This is standard practice in UNIX/QNX process creation.
 */
extern char **environ;  // Defined by C runtime, contains environment vars

/*=============================================================================
 * FORWARD DECLARATIONS
 *
 * These are function prototypes for the task entry points. Each task is
 * implemented in its own .c file, so we declare them here to tell the
 * compiler they exist.
 *===========================================================================*/

/**
 * @brief Entry point for the Detection Task
 * @param arg Unused parameter (required by pthread/process signature)
 * @return NULL when task exits
 *
 * This task continuously monitors the radar simulator for new threats.
 * When a missile is detected, it updates the shared memory with the
 * initial position and velocity.
 */
void *detection_task(void *arg);

/**
 * @brief Entry point for the Tracking Task
 * @param arg Unused parameter
 * @return NULL when task exits
 *
 * This task runs a Kalman filter to smooth noisy radar measurements and
 * predict where the missile will be in the future. Runs every 100ms.
 */
void *tracking_task(void *arg);

/**
 * @brief Entry point for the Intercept Calculation Task
 * @param arg Unused parameter
 * @return NULL when task exits
 *
 * This task solves the intercept geometry problem: given missile trajectory
 * and counter-missile speed, where should we aim to hit it?
 */
void *intercept_calc_task(void *arg);

/**
 * @brief Entry point for the Launch Command Task
 * @param arg Unused parameter
 * @return NULL when task exits
 *
 * This task monitors the intercept solution and fires the counter-missile
 * when conditions are right (high confidence, valid solution, stable track).
 */
void *launch_command_task(void *arg);

/**
 * @brief Entry point for the Watchdog Task
 * @param arg Unused parameter
 * @return NULL when task exits
 *
 * This task monitors all other tasks' heartbeats. If any task stops
 * updating its heartbeat, the watchdog automatically restarts it.
 */
void *watchdog_task(void *arg);

/*=============================================================================
 * HELPER FUNCTIONS
 *===========================================================================*/

/*-----------------------------------------------------------------------------
 * FUNCTION: run_as_child_process
 *
 * PURPOSE:
 *   When this program is launched with a task name argument, it runs as
 *   a child process executing that specific task instead of being the
 *   main launcher.
 *
 * HOW IT WORKS:
 *   1. Opens the already-created shared memory
 *   2. Maps it into this process's address space
 *   3. Calls the appropriate task function based on the name
 *   4. Cleans up and exits when task finishes
 *
 * PARAMETERS:
 *   task_name - String identifying which task to run:
 *               "detection", "tracking", "intercept", "launch", or "watchdog"
 *
 * RETURNS:
 *   EXIT_SUCCESS (0) if task completed normally
 *   EXIT_FAILURE (1) if shared memory couldn't be opened
 *
 *---------------------------------------------------------------------------*/
static int run_as_child_process(const char *task_name) {
    
    /* Step 1: Open the shared memory object created by parent process
     * --------------------------------------------------------------
     * The parent process (main launcher) has already created this shared
     * memory region. We just need to open it for reading and writing.
     *
     * shm_open() is like open() for files, but for shared memory objects.
     * It returns a file descriptor that we can use with mmap().
     */
    int fd = shm_open(SHM_NAME,    // Name of shared memory ("/hmds_shared")
                      O_RDWR,       // Open for reading and writing
                      0);           // No creation flags (parent already created it)
    
    // Check if opening failed (parent might not have created it yet)
    if (fd == -1) {
        // shm_open failed - shared memory doesn't exist or permissions wrong
        return EXIT_FAILURE;  // Exit with error code 1
    }
    
    /* Step 2: Map shared memory into our process's address space
     * ---------------------------------------------------------
     * mmap() creates a mapping from the shared memory into our virtual
     * address space. After this call, we can access the shared memory
     * through the g_shm pointer just like a regular struct.
     */
    g_shm = mmap(
        NULL,                       // Let kernel choose address in our space
        sizeof(HMDSSharedMem),     // Size of the region to map
        PROT_READ | PROT_WRITE,    // We need both read and write access
        MAP_SHARED,                 // Changes are visible to other processes
        fd,                         // File descriptor from shm_open
        0                           // Offset 0 (map from beginning)
    );
    
    close(fd);  // Don't need the file descriptor anymore after mapping
    
    // Check if mapping failed
    if (g_shm == MAP_FAILED) {
        // mmap failed - probably out of memory or permissions issue
        return EXIT_FAILURE;  // Exit with error code 1
    }
    
    /* Step 3: Run the appropriate task based on the task name
     * -------------------------------------------------------
     * The task name was passed as a command-line argument. We compare
     * it against known task names and call the corresponding function.
     *
     * Each task function runs in an infinite loop until system_armed
     * becomes false, then returns.
     */
    if (strcmp(task_name, "detection") == 0) {
        // This is the detection task - monitor radar for threats
        detection_task(NULL);  // NULL argument (not used by task)
        
    } else if (strcmp(task_name, "tracking") == 0) {
        // This is the tracking task - run Kalman filter on detections
        tracking_task(NULL);
        
    } else if (strcmp(task_name, "intercept") == 0) {
        // This is the intercept calculator - solve firing solution
        intercept_calc_task(NULL);
        
    } else if (strcmp(task_name, "launch") == 0) {
        // This is the launch command task - fire counter-missile
        launch_command_task(NULL);
        
    } else if (strcmp(task_name, "watchdog") == 0) {
        // This is the watchdog task - monitor other tasks' health
        watchdog_task(NULL);
    }
    // If task_name doesn't match any of the above, we just fall through
    // and exit (this shouldn't happen in normal operation)
    
    /* Step 4: Clean up before exiting
     * -------------------------------
     * When the task function returns (because system is shutting down),
     * we need to unmap the shared memory from our address space.
     */
    munmap(g_shm,                   // Pointer to mapped region
           sizeof(HMDSSharedMem));  // Size of the region
    
    return EXIT_SUCCESS;  // Exit successfully (code 0)
}

/*-----------------------------------------------------------------------------
 * FUNCTION: shm_init
 *
 * PURPOSE:
 *   Create and initialize the shared memory region that all tasks will use
 *   to communicate. This is only called by the parent launcher process.
 *
 * HOW IT WORKS:
 *   1. Create a new shared memory object (or open existing one)
 *   2. Set its size to fit our HMDSSharedMem structure
 *   3. Map it into our address space
 *   4. Zero out all fields
 *   5. Set initial values (system_armed = true, etc.)
 *
 * RETURNS:
 *   0 on success
 *   -1 on failure (with error message printed)
 *
 * QNX CONCEPT - SHARED MEMORY:
 *   Shared memory is the fastest way for processes to exchange data in QNX.
 *   Unlike message passing (which copies data), shared memory lets multiple
 *   processes access the SAME physical memory, eliminating copy overhead.
 *---------------------------------------------------------------------------*/
static int shm_init(void) {
    
    /* Step 1: Create the shared memory object
     * ---------------------------------------
     * shm_open() creates a POSIX shared memory object with the given name.
     * Think of it like creating a file, but in RAM instead of on disk.
     */
    int fd = shm_open(
        SHM_NAME,              // Name: "/hmds_shared" (defined in hmds_common.h)
        O_CREAT | O_RDWR,     // Create if doesn't exist, open for read/write
        S_IRUSR | S_IWUSR     // Permissions: owner can read and write
    );
    
    // Check if creation failed
    if (fd == -1) {
        perror("shm_open");  // Print error message with reason
        return -1;            // Return failure code
    }
    
    /* Step 2: Set the size of the shared memory region
     * ------------------------------------------------
     * When first created, shared memory has size 0. We need to set it
     * to the size of our structure using ftruncate().
     *
     * Why ftruncate():
     * Shared memory objects act like files, so we use file operations
     * like ftruncate() to resize them.
     */
    if (ftruncate(fd, sizeof(HMDSSharedMem)) == -1) {
        perror("ftruncate");  // Print error message
        close(fd);             // Clean up the file descriptor
        return -1;             // Return failure code
    }
    
    /* Step 3: Map the shared memory into our process's address space
     * -------------------------------------------------------------
     * After this call, g_shm will point to the shared memory region,
     * and we can access it like a normal struct pointer.
     */
    g_shm = mmap(
        NULL,                       // Let kernel choose the address
        sizeof(HMDSSharedMem),     // Map the entire structure
        PROT_READ | PROT_WRITE,    // Allow reading and writing
        MAP_SHARED,                 // Share with other processes (not private)
        fd,                         // File descriptor from shm_open
        0                           // Start from offset 0
    );
    
    close(fd);  // Don't need fd anymore after mapping
    
    // Check if mapping failed
    if (g_shm == MAP_FAILED) {
        perror("mmap");   // Print error message
        return -1;         // Return failure code
    }
    
    /* Step 4: Zero out all fields in the shared memory
     * ------------------------------------------------
     * memset() fills the entire structure with zeros. This ensures:
     * - All flags start as false
     * - All counters start at 0
     * - No garbage data from previous runs
     */
    memset(g_shm,                   // Pointer to shared memory
           0,                        // Value to fill with (0 = zero)
           sizeof(HMDSSharedMem));  // Number of bytes to zero
    
    /* Step 5: Set initial values for key fields
     * -----------------------------------------
     * Some fields need non-zero starting values.
     */
    g_shm->system_armed = true;  // Enable the system (tasks will run)
    
    /* Step 6: Initialize Qnet (distributed networking) status
     * -------------------------------------------------------
     * Qnet allows processes on different QNX machines to communicate.
     * We check if the radar node is reachable.
     */
    g_shm->radar_node_id = QNET_RADAR_NODE;  // Store which node has the radar
    
    // Check if the radar node is alive (reachable over network)
    g_shm->qnet_active = qnet_is_node_alive(QNET_RADAR_NODE);
    
    /* Step 7: Log successful initialization
     * -------------------------------------
     * Print confirmation message showing shared memory is ready.
     */
    hmds_log(LOG_INFO,        // Severity level: informational
             "MAIN",           // Task name for log
             "Shared memory initialised (%zu bytes)",  // Message format
             sizeof(HMDSSharedMem));  // Actual size in bytes
    
    // Log Qnet status (is the radar node reachable?)
    hmds_log(LOG_INFO, "MAIN", "Qnet: Radar node %d status = %s",
             QNET_RADAR_NODE,  // Node ID number
             g_shm->qnet_active ? "ACTIVE" : "INACTIVE");  // Status string
    
    return 0;
}

/*-----------------------------------------------------------------------------
 * FUNCTION: spawn_task_process
 *
 * PURPOSE:
 *   Launch one of the defense tasks as a new process
 *
 * HOW IT WORKS:
 *   Uses posix_spawn() to create a new process running the same executable
 *   but with a different command-line argument specifying which task to run.
 *
 * PARAMETERS:
 *   task_name    - Name of task: "detection", "tracking", etc.
 *   binary_path  - Full path to our executable (usually /proc/self/exe)
 *
 * RETURNS:
 *   Process ID (PID) of the new process on success
 *   -1 on failure
 *
 * EXAMPLE:
 *   pid_t tracking_pid = spawn_task_process("tracking", "/path/to/hmds");
 *   // Now a new process is running the tracking task
 *
 * QNX CONCEPT - PROCESS SPAWNING:
 *   QNX uses processes (not threads) for isolation. If one process crashes,
 *   others keep running. posix_spawn() is the modern way to create processes.
 *---------------------------------------------------------------------------*/
static pid_t spawn_task_process(const char *task_name, 
                                 const char *binary_path) {
    
    pid_t pid;  // Will hold the process ID of the new process
    
    /* Step 1: Build the argument array for the new process
     * ----------------------------------------------------
     * When we spawn a process, we need to tell it:
     * - argv[0]: Program name (by convention, path to executable)
     * - argv[1]: Task name (which task to run)
     * - argv[2]: NULL (marks end of arguments)
     *
     * Example: ./hmds tracking
     *   argv[0] = "./hmds"
     *   argv[1] = "tracking"
     *   argv[2] = NULL
     */
    char *argv[] = {
        (char *)binary_path,   // argv[0]: Path to executable
        (char *)task_name,     // argv[1]: Task name
        NULL                    // argv[2]: End marker
    };
    
    /* Step 2: Spawn the new process
     * -----------------------------
     * posix_spawn() creates a new process and starts it running.
     * It's similar to fork() + exec() but more efficient and portable.
     */
    int rc = posix_spawn(
        &pid,          // Output: PID of new process stored here
        binary_path,   // Path to executable to run
        NULL,          // File actions (none needed)
        NULL,          // Spawn attributes (use defaults)
        argv,          // Command-line arguments
        environ        // Environment variables to inherit
    );
    
    /* Step 3: Check if spawning succeeded
     * -----------------------------------
     * posix_spawn() returns 0 on success, error code on failure.
     */
    if (rc != 0) {
        // Spawning failed - log critical error
        hmds_log(LOG_CRITICAL,     // This is a serious problem
                 "MAIN",            // From main launcher
                 "posix_spawn %s failed: %s",  // Error message format
                 task_name,          // Which task failed to spawn
                 strerror(rc));      // Human-readable error description
        return -1;  // Return failure code
    }
    
    /* Step 4: Log successful process creation
     * ---------------------------------------
     * Print confirmation with the new process's ID number.
     */
    hmds_log(LOG_INFO, "MAIN", 
             "Spawned %s process (PID=%d)", 
             task_name,      // Name of the task
             (int)pid);      // Process ID number
    
    return pid;  // Return PID so caller can track this process
}

/*-----------------------------------------------------------------------------
 * FUNCTION: cleanup
 *
 * PURPOSE:
 *   Signal handler called when user presses Ctrl+C or system sends SIGTERM.
 *   Performs graceful shutdown of the entire system.
 *
 * HOW IT WORKS:
 *   1. Set system_armed flag to false (tells all tasks to exit)
 *   2. Unmap shared memory
 *   3. Delete shared memory object
 *
 * PARAMETERS:
 *   sig - Signal number that triggered this handler (SIGTERM or SIGINT)
 *         We don't use it, but the signature requires it.
 *
 * QNX CONCEPT - SIGNAL HANDLING:
 *   Signals are async notifications sent to processes. SIGINT comes from
 *   Ctrl+C, SIGTERM from kill command. Handling them gracefully prevents
 *   data corruption and allows proper cleanup.
 *---------------------------------------------------------------------------*/
static void cleanup(int sig) {
    
    (void)sig;  // Suppress "unused parameter" warning
    
    /* Step 1: Check if shared memory is mapped
     * ----------------------------------------
     * If g_shm is NULL, we never successfully initialized, so nothing to clean.
     */
    if (g_shm) {
        
        /* Tell all tasks to stop running
         * -------------------------------
         * Every task's main loop checks this flag. When false, they exit.
         */
        g_shm->system_armed = false;
        
        /* Unmap the shared memory from our address space
         * ----------------------------------------------
         * This doesn't delete the memory, just removes it from our process.
         */
        munmap(g_shm, sizeof(HMDSSharedMem));
    }
    
    /* Step 2: Delete the shared memory object
     * ---------------------------------------
     * shm_unlink() removes the shared memory from the system. After this,
     * no process can open it anymore. Any processes that already have it
     * mapped can still use it until they unmap.
     */
    shm_unlink(SHM_NAME);  // Delete "/hmds_shared"
}

/*=============================================================================
 * MAIN FUNCTION
 *
 * This is the entry point when the program starts. It can run in two modes:
 *
 * MODE 1: LAUNCHER (no arguments)
 *   Initializes the system and spawns all task processes
 *
 * MODE 2: CHILD TASK (one argument)
 *   Runs as a specific task (detection, tracking, etc.)
 *===========================================================================*/
int main(int argc, char *argv[]) {
    
    /*-------------------------------------------------------------------------
     * CHECK WHICH MODE TO RUN IN
     *-----------------------------------------------------------------------*/
    
    /* Mode 2: Child process mode
     * --------------------------
     * If launched with one argument (the task name), run that task and exit.
     *
     * argc (argument count) tells us how many command-line arguments:
     * - argc == 1 means just program name: ./hmds
     * - argc == 2 means program name + task: ./hmds tracking
     */
    if (argc == 2) {
        // We were launched with a task name argument
        // argv[1] contains the task name string
        return run_as_child_process(argv[1]);  // Run task and exit
    }
    
    /*-------------------------------------------------------------------------
     * MODE 1: PARENT LAUNCHER MODE
     * From here on, we're the main launcher process
     *-----------------------------------------------------------------------*/
    
    /* Step 1: Print startup banner
     * ----------------------------
     * Display system information to console.
     */
    hmds_log(LOG_INFO, "MAIN", "═══════════════════════════════════════");
    hmds_log(LOG_INFO, "MAIN", " HMDS MULTI-PROCESS ARCHITECTURE");
    hmds_log(LOG_INFO, "MAIN", " 5 Processes | MMU Isolation | QNX IPC");
    hmds_log(LOG_INFO, "MAIN", "═══════════════════════════════════════");
    
    /* Step 2: Setup signal handlers for graceful shutdown
     * ---------------------------------------------------
     * Register our cleanup function to handle:
     * - SIGTERM: sent by 'kill' command
     * - SIGINT: sent by Ctrl+C
     *
     * When these signals arrive, cleanup() function will be called.
     */
    signal(SIGTERM, cleanup);  // Handle kill command
    signal(SIGINT, cleanup);   // Handle Ctrl+C
    
    /* Step 3: Initialize shared memory
     * --------------------------------
     * Create and setup the memory region all processes will share.
     */
    if (shm_init() != 0) {
        // Shared memory initialization failed - can't continue
        return EXIT_FAILURE;  // Exit with error code
    }
    
    /* Step 4: Display QNX concepts information
     * ----------------------------------------
     * Print a table showing the 12 QNX concepts we're using.
     * This is educational - helps understand the architecture.
     */
    hmds_print_qnx_concepts();
    
    /* Step 5: Display Qnet distributed architecture banner
     * ----------------------------------------------------
     * Show configuration for multi-node operation (radar on one node,
     * HMDS processing on another).
     */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║              QNET DISTRIBUTED ARCHITECTURE               ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  Local HMDS Node:    %d (ND_LOCAL_NODE)                  ║\n", 
           QNET_HMDS_NODE);
    printf("║  Remote Radar Node:  %d (QNET_RADAR_NODE)                ║\n", 
           QNET_RADAR_NODE);
    printf("║  Qnet Status:        %-35s║\n", 
           g_shm->qnet_active ? "ACTIVE ✓" : "INACTIVE ✗");
    

    if (QNET_RADAR_NODE == 0) {
        printf("║  Mode:               Single-Node Simulation              ║\n");
    } else {
        printf("║  Mode:               Multi-Node Distributed              ║\n");
    }
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\n");
    fflush(stdout);  // Force output to appear immediately
    
    /* Step 6: Get the path to our own executable
     * ------------------------------------------
     * We need this to spawn child processes. On Linux, /proc/self/exe
     * is a symbolic link to the currently running executable.
     */
    char binary_path[256];  // Buffer to hold the path string
    
    // readlink() reads where the symlink points to
    ssize_t len = readlink("/proc/self/exe",      // Symlink to read
                          binary_path,            // Buffer to store result
                          sizeof(binary_path) - 1); // Max bytes to read
    
    if (len == -1) {
        // readlink() failed - use command-line argument as fallback
        strcpy(binary_path, argv[0]);  // Use how we were invoked
    } else {
        // readlink() succeeded - null-terminate the string
        binary_path[len] = '\0';  // Add string terminator
    }
    
    /* Step 7: Spawn all five task processes
     * -------------------------------------
     * Launch each task as a separate process. Store their PIDs so we can
     * monitor them later.
     */
    pid_t pids[5];  // Array to hold process IDs
    
    // Spawn task 0: Detection (monitors radar for threats)
    pids[0] = spawn_task_process("detection", binary_path);
    
    // Spawn task 1: Tracking (Kalman filter for trajectory prediction)
    pids[1] = spawn_task_process("tracking", binary_path);
    
    // Spawn task 2: Intercept Calculator (computes firing solution)
    pids[2] = spawn_task_process("intercept", binary_path);
    
    // Spawn task 3: Launch Command (fires counter-missile)
    pids[3] = spawn_task_process("launch", binary_path);
    
    // Wait 1 second before spawning watchdog (give tasks time to start)
    sleep(1);
    
    // Spawn task 4: Watchdog (monitors and restarts other tasks)
    pids[4] = spawn_task_process("watchdog", binary_path);
    
    // Log that all processes are running
    hmds_log(LOG_INFO, "MAIN", "All processes spawned. Monitoring active.");
    
    /*-------------------------------------------------------------------------
     * MAIN MONITORING LOOP
     *
     * The parent process now enters a monitoring loop that:
     * 1. Checks if the watchdog died (and restarts it if needed)
     * 2. Logs when other tasks die (watchdog will restart them)
     * 3. Runs until user presses Ctrl+C or system_armed becomes false
     *-----------------------------------------------------------------------*/
    
    while (g_shm->system_armed) {
        
        int status;  // Will hold exit status of dead processes
        
        /*---------------------------------------------------------------------
         * CHECK IF WATCHDOG DIED
         * The watchdog monitors other tasks, but who watches the watchdog?
         * We do! If watchdog dies, we restart it ourselves.
         *-------------------------------------------------------------------*/
        
        // Check if watchdog process has exited (non-blocking check)
        pid_t watchdog_check = waitpid(
            pids[4],     // Watchdog's PID
            &status,     // Store exit status here
            WNOHANG      // Don't block (return immediately if still running)
        );
        
        if (watchdog_check == pids[4]) {
            // Watchdog process has died - this is serious!
            
            // Print bordered alert message
            hmds_log(LOG_WARN, "MAIN",
                     "╔══════════════════════════════════════════════════════════╗");
            hmds_log(LOG_WARN, "MAIN",
                     "║  WATCHDOG FAILURE DETECTED (PID %d)                     ║", 
                     (int)watchdog_check);
            hmds_log(LOG_WARN, "MAIN",
                     "║  Restarting watchdog — system remains operational       ║");
            hmds_log(LOG_WARN, "MAIN",
                     "╚══════════════════════════════════════════════════════════╝");
            
            // Wait a second for system to stabilize
            sleep(1);
            
            // Restart the watchdog
            pids[4] = spawn_task_process("watchdog", binary_path);
            
            /* Update the PID file that watchdog uses
             * ----------------------------------------
             * Watchdog reads this file to know which processes to monitor.
             * Since we just restarted it, it needs the current PIDs.
             */
            FILE *pid_fp = fopen("/tmp/hmds_task_pids", "w");
            if (pid_fp) {
                // Write PIDs of the four non-watchdog tasks
                fprintf(pid_fp, "%d\n%d\n%d\n%d\n",
                        (int)pids[0],  // Detection PID
                        (int)pids[1],  // Tracking PID
                        (int)pids[2],  // Intercept PID
                        (int)pids[3]); // Launch PID
                fclose(pid_fp);  // Close the file
            }
            
            // Log successful restart
            hmds_log(LOG_INFO, "MAIN",
                     "✓ Watchdog restarted successfully (PID=%d)", (int)pids[4]);
        }
        
        /*---------------------------------------------------------------------
         * CHECK IF ANY OTHER TASK DIED
         * If a task dies, log it but DON'T restart it here. The watchdog
         * will detect the missing heartbeat and restart it automatically.
         *-------------------------------------------------------------------*/
        
        // Loop through the four non-watchdog tasks
        for (int i = 0; i < 4; i++) {
            
            // Check if this task has exited (non-blocking)
            pid_t check = waitpid(
                pids[i],      // This task's PID
                &status,      // Store exit status
                WNOHANG       // Don't block
            );
            
            if (check > 0) {
                // This task has died
                
                // Map task index to human-readable name
                const char *task_name = 
                    (i == 0) ? "Detection" :
                    (i == 1) ? "Tracking" :
                    (i == 2) ? "Intercept" : "Launch";
                
                // Log the death (watchdog will handle restart)
                hmds_log(LOG_WARN, "MAIN",
                         "Task %s (PID %d) died — watchdog will handle restart",
                         task_name,  // Which task
                         (int)check); // Its PID
                
                // NOTE: We DON'T break or exit here - let watchdog handle it
            }
        }
        
        // Sleep for 1 second before next check
        // This prevents busy-waiting and consuming too much CPU
        sleep(1);
    }
    
    /*-------------------------------------------------------------------------
     * SHUTDOWN SEQUENCE
     * We only reach here if:
     * - User pressed Ctrl+C (cleanup() set system_armed = false)
     * - Some code set system_armed = false explicitly
     *-----------------------------------------------------------------------*/
    
    hmds_log(LOG_INFO, "MAIN", "System shutdown requested. Cleaning up...");
    
    /* Step 1: Set system_armed to false
     * ---------------------------------
     * In case we got here some other way, make sure flag is cleared.
     */
    g_shm->system_armed = false;
    
    /* Step 2: Send SIGTERM to all child processes
     * -------------------------------------------
     * Politely ask each process to shut down.
     */
    for (int i = 0; i < 5; i++) {
        if (pids[i] > 0) {  // Only if process was spawned successfully
            kill(pids[i], SIGTERM);  // Send termination signal
        }
    }
    
    /* Step 3: Wait for all children to exit
     * -------------------------------------
     * wait() blocks until a child process exits. We call it repeatedly
     * until no more children remain.
     */
    while (wait(NULL) > 0) {
        // Loop continues until wait() returns -1 (no more children)
    }
    
    /* Step 4: Final cleanup
     * --------------------
     * Unmap shared memory and delete it.
     */
    cleanup(0);
    
    return EXIT_SUCCESS;  // Exit successfully
}
