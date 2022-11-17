/**
 * @file network_defines.hpp
 * @author Shaqeeb Momen (shaqeebmomen@yahoo.ca)
 * @brief definitions for packet structures common to both the ESP-01 and the RPI PICO to be passed over the network and through UART
 * Also defines inline functions for loading data between packet structures & byte buffers (and vice versa)
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <stdlib.h>
#include <memory.h>
#include <stdint.h>

/**
 * @brief packet codes, usually sent ahead of the packet itself
 * 
 */
typedef enum
{
    WHO_AM_I,
    NODE_STATE,
    PATH
} packet_code;

/**
 * @brief states for node operation
 * 
 */
typedef enum
{
    NODE_IDLE,
    NODE_PATH_LEAVE,
    NODE_PATH_RETURN
} node_state;


/**
 * @brief Node state structure
 * 
 */
typedef struct
{
    double x;         // x position
    double y;         // y position
    double v;         // Linear velocity
    double theta;     // Heading
    uint64_t ts_ms;   // Timestamp in ms
    node_state state; // Current executing state of the node

} packet_node_state;

/**
 * @brief path point packet structure
 * 
 */
typedef struct
{
    double x;       // x position
    double y;       // y position
    double v;       // Linear velocity
    double theta;   // Heading
    uint64_t ts_ms; // Timestamp in ms
} packet_path_point;


/**
 * @brief loads data into a packet structure (should be allocated already) from a byte buffer
 * 
 * @param packet packet to load data into
 * @param buff buffer to load bytes from
 * @param size size of packet structure, a sizeof() call is expected here
 */
static inline void packet_from_buff(void *packet, char *buff, size_t size)
{
    memcpy(packet, buff, size);
}

/**
 * @brief loads data into a buffer (buffer should be allocated already), from a packet structure
 * 
 * @param buff buffer to load bytes into
 * @param packet packet to get bytes from 
 * @param size size of packet structure, a sizeof() call is expected here
 */
static inline void buff_from_packet(char *buff, void *packet, size_t size)
{
    memcpy(buff, packet, size);
}