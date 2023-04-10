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

#define PACKET_MAX_SIZE 40

/**
 * @brief packet codes, usually sent ahead of the packet itself
 * 
 */
typedef enum
{
    PACKET_WHO_AM_I,
    PACKET_NODE_STATE,
    PACKET_PATH,
    PACKET_STOP,
    PACKET_GO,
    PACKET_DIAG_STATE,
    // PACKET_ESP_STATUS,
    PACKET_REBASE,
    PACKET_TYPES_COUNT
} packet_code;

/**
 * @brief states for node operation
 * 
 */
typedef enum
{
    NODE_IDLE,
    NODE_PATH_LEAVE,
    NODE_PATH_RETURN,
    NODE_STOPPED
} node_state;


/**
 * @brief Node state structure
 * 
 */
typedef struct
{
    char code = PACKET_NODE_STATE;
    double x;         // x position
    double y;         // y position
    double v;         // Linear velocity
    double theta;     // Heading
    node_state state; // Current executing state of the node
    double x_exp;     // expected x position relative to the start based on path
    double y_exp;     // expected y position relative to the start based on path
    double velocity_exp; // expected current velocity based on path
    double heading_exp; // expected current heading based on path
} packet_node_state;

/**
 * @brief path point packet structure
 * 
 */
typedef struct
{
    char code = PACKET_PATH;
    double x;       // x position
    double y;       // y position
} packet_path_point;


typedef struct 
{
    char code = PACKET_DIAG_STATE;
    double v_right; // current velocity of the right motor
    double d_right; // cumulative distance the right motor has traveled since time 0
    double v_left; // current velocity of the left motor
    double d_left; // cumulative distance the left motor has traveled since time 0
} packet_diag_node_state;


typedef struct
{
    char code = PACKET_REBASE;
    double x;
    double y;
    double heading;
} packet_rebase;


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
static inline void buff_from_packet(void *buff, void *packet, size_t size)
{
    memcpy(buff, packet, size);
}
