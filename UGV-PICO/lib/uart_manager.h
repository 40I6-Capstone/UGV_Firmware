/**
 * @file uart_manager.h
 * @author Shaqeeb Momen
 * @brief Functions for using the UART hw on the PICO to load data intro structures, defined by the network defines header
 * @version 0.1
 * @date 2022-11-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <stdint.h>
#include <pico/stdlib.h>
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "../../common_lib/network_defines.h"

#define UART uart0
#define UART_IRQ UART0_IRQ

static uint8_t buff[PACKET_MAX_SIZE];

volatile bool uart_parsing_done = false; // Flag for when a struct is parsed done being parsed over

/**
 * @brief Handler for UART RX interrupts
 * State machine to parse bytes read into a buffer and raise a flag when complete
 *
 */
void uart_handler()
{
    enum
    {
        RESET,   // Reset communications
        IDLE,    // Waiting for first byte in packet
        PARSING, // Parsing bytes untill enough for packet struct
    } uart_state;

    static packet_code packet;   // Code of the current packet being parsed from UART
    static uint8_t packet_size;  // Size of the packet to be saved
    static uint8_t rx_count = 0; // Count of the bytes read

    // Run the state machine for however many bytes are available
    while (uart_is_readable(UART))
    {
        switch (uart_state)
        {
        // Waiting for a packet code byte
        case IDLE:
            if (!uart_parsing_done)
            {
                packet = (packet_code)uart_getc(UART); // First byte, set the code to determine how the rest are parsed
                uart_state = PARSING;                  // Set state to parsing
                rx_count = 0;                          // Reset rx count
            }
            break;

        // Parsing bytes as they come in until the relevant struct is full
        case PARSING:
            // Get size of the packet to load
            switch (packet)
            {
            case PACKET_PATH:
                packet_size = sizeof(packet_path_point);
                break;
            }
            // Update buffer
            buff[rx_count++] = uart_getc(UART);
            // If enough is read to save to a struct, reset state and raise flag
            if (rx_count > packet_size)
            {
                uart_state = IDLE;
                uart_parsing_done = true;
            }
            break;
        }
    }
}

/**
 * @brief Initialize the uart manager
 *
 * @param tx tx pin
 * @param rx rx pin
 * @param baud baud rate
 */
void uart_manager_init(uint tx, uint rx, uint baud)
{
    uart_init(UART, baud); // Make sure this is consistent with the baud setup on the ESP-01s

    // Setting up GPIO pins
    gpio_set_function(tx, GPIO_FUNC_UART);
    gpio_set_function(rx, GPIO_FUNC_UART);

    // Enabling interrupt

    irq_set_exclusive_handler(UART_IRQ, uart_handler);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART, true, false);
}


/**
 * @brief Handles the uart manager looping code, subscribed methods, 
 * 
 */
void uart_manager_loop(){

}

/**
 * @brief Load data from buffer into a struct
 *
 * @param dst pointer to destination struct
 * @param size size of struct in bytes, a sizeof() call is expected here
 */
void uart_manager_load(void *dst, size_t size)
{
    packet_from_buff(dst, (char *)buff, size);
}



/**
 * @brief Send struct over uart
 * 
 * @param src pointer to struct to send
 * @param size size of struct, a sizeof() call is expected here
 */
void uart_manager_send(void *src, size_t size)
{
    uint8_t *tx_buff = (uint8_t *)malloc(size); // Buffer of bytes to send through uart
    uart_write_blocking(UART, tx_buff, size);   // Write to UART
}

