/*
 * qnet_utils.c — QNet Distributed Computing Support
 * 
 * Implements Qnet functionality for distributed radar/HMDS architecture:
 * - Remote node connection
 * - Cross-node message passing
 * - Node health monitoring
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "hmds_common.h"

#ifdef __QNX__
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif

/*
 * qnet_connect_to_node() — Connect to channel on remote QNX node
 * 
 * node_id: Target node (0=local, 1+=remote)
 * chid: Channel ID on target node
 * 
 * Returns: Connection ID (coid) on success, -1 on failure
 */
int qnet_connect_to_node(int node_id, int chid) {
#ifdef __QNX__
    int coid = ConnectAttach(node_id, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        hmds_log(LOG_WARN, "QNET", "ConnectAttach to node %d chid %d failed: %s",
                 node_id, chid, strerror(errno));
        return -1;
    }
    hmds_log(LOG_INFO, "QNET", "Connected to node %d chid %d → coid %d",
             node_id, chid, coid);
    return coid;
#else
    (void)node_id;
    (void)chid;
    hmds_log(LOG_WARN, "QNET", "Qnet not available in simulation mode");
    return -1;
#endif
}

/*
 * qnet_send_missile_data() — Send missile state to remote node
 */
int qnet_send_missile_data(int coid, const MissileState *missile) {
#ifdef __QNX__
    HMDSMessage msg;
    msg.type = MSG_MISSILE_PARAMS;
    msg.seq = 0;
    msg.flags = 0;
    memcpy(&msg.payload.missile, missile, sizeof(MissileState));
    
    // Use MsgSend for synchronous delivery (blocks until reply)
    int status = 0;
    if (MsgSend(coid, &msg, sizeof(msg), &status, sizeof(status)) == -1) {
        hmds_log(LOG_WARN, "QNET", "MsgSend failed: %s", strerror(errno));
        return -1;
    }
    
    return 0;
#else
    (void)coid;
    (void)missile;
    return 0; // Simulation mode — no-op
#endif
}

/*
 * qnet_get_remote_node_id() — Resolve node name to node ID
 */
int qnet_get_remote_node_id(const char *node_name) {
#ifdef __QNX__
    /* In production: Use netmgr_strtond() to resolve hostname to node ID
     * For demo: Return hardcoded QNET_RADAR_NODE */
    (void)node_name;
    return QNET_RADAR_NODE;
#else
    (void)node_name;
    return QNET_RADAR_NODE;
#endif
}

/*
 * qnet_is_node_alive() — Check if remote node is reachable
 */
bool qnet_is_node_alive(int node_id) {
#ifdef __QNX__
    // Attempt to get node descriptor
    int nd = netmgr_ndtostr(0, node_id, NULL, 0);
    return (nd != -1);
#else
    (void)node_id;
    return true; // Simulation mode — assume alive
#endif
}
