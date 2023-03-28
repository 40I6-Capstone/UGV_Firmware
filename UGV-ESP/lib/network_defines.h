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
#pragma once

#include <stdlib.h>
#include <memory.h>
#include <stdint.h>


#define PACKET_MAX_SIZE 256 

/**
 * @brief packet codes, usually sent ahead of the packet itself
 * 
 */
typedef enum
{
    PACKET_WHO_AM_I,
    PACKET_NODE_STATE,
    PACKET_PATH,
    PACKET_TYPES_COUNT // Dont add after this, always before
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
} packet_path_point; // 40 bytes


/**
 * @brief Copies data into a packet structure (should be allocated already) from a byte buffer
 * 
 * @param packet packet to load data into
 * @param buff buffer to load bytes from
 * @param size size of packet structure, a sizeof() call is expected here
 */
static inline void packet_from_buff(void *packet, uint8_t *buff, size_t size)
{
    memcpy(packet, buff, size);
}

/**
 * @brief Copies data into a buffer (buffer should be allocated already), from a packet structure
 * 
 * @param buff buffer to load bytes into
 * @param packet packet to get bytes from 
 * @param size size of packet structure, a sizeof() call is expected here
 */
static inline void buff_from_packet(uint8_t *buff, void *packet, size_t size)
{
    memcpy(buff, packet, size);
}