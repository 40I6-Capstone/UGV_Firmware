
/**
 * @file UARTManager.hpp
 * @author Shaqeeb Momen
 * @brief UART controller to manage incoming data and parse into appropriate structs,
 * Also will run any subscribed functions upon new data parsing
 * @version 0.1
 * @date 2022-11-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <vector>
#include <stdint.h>
#include "network_defines.h"

class UARTManager
{
private:
    uint8_t buff[PACKET_MAX_SIZE];
    volatile bool parsed;
    volatile packet_code parsed_packet_code;
    std::vector<void (*)()> subscribers[PACKET_TYPES_COUNT];
    void publish();

public:
    UARTManager();
    void loop();
    void load(void *dst, size_t size);
    void send(void *src, size_t size);
    void subscribe(void (*func)(), packet_code code);
    void int_handler();
};

/**
 * @brief Construct a new UARTManager::UARTManager object
 */
UARTManager::UARTManager()
{
    // Initial member states
    this->parsed = true;
}

void UARTManager::int_handler()
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
    while (Serial.available() > 0)
    {
        switch (uart_state)
        {
        // Waiting for a packet code byte
        case IDLE:
            if (!this->parsed)
            {
                packet = (packet_code)Serial.read(); // First byte, set the code to determine how the rest are parsed
                uart_state = PARSING;                // Set state to parsing
                rx_count = 0;                        // Reset rx count
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
            this->buff[rx_count++] = Serial.read();
            // If enough is read to save to a struct, reset state and raise flag
            if (rx_count > packet_size)
            {
                uart_state = IDLE;
                this->parsed = true;
                this->parsed_packet_code = packet;
            }
            break;
        }
    }
}

/**
 * @brief looper function, should be called by the main loop to handle buffer after it has been loaded and fire any subscribed functions
 *
 */
void UARTManager::loop()
{
    if (this->parsed)
    {
        for (size_t i = 0; i < subscribers[parsed_packet_code].size(); i++)
        {
            subscribers[parsed_packet_code][i]();
        }
        this->parsed = false;
    }
}

/**
 * @brief Add a new function to be run on updates
 *
 * @param func pointer to function to be run
 * @param code the type of packet, when recieved to subscribe to
 */
void UARTManager::subscribe(void (*func)(), packet_code code)
{
    this->subscribers[code].push_back(func);
}

/**
 * @brief Load data from uart buffer into a struct
 *
 * @param dst pointer to destination struct
 * @param size size of struct in bytes, a sizeof() call is expected here
 */
void UARTManager::load(void *dst, size_t size)
{
    packet_from_buff(dst, this->buff, size);
}

/**
 * @brief Send struct over uart, blocking
 *
 * @param src pointer to struct to send
 * @param size size of struct, a sizeof() call is expected here
 */
void UARTManager::send(void *src, size_t size) // TODO send packet code
{
    uint8_t *tx_buff = (uint8_t *)malloc(size); // Buffer of bytes to send through uart
    buff_from_packet(tx_buff, src, size);
    Serial.write((uint8_t *)tx_buff, size); // Write to UART
    free(tx_buff);
}
