
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
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
// #include "../../common_lib/network_defines.hpp"
#include "../../UGV-ESP/UGV-ESP/network_defines.hpp"

#define UART uart0
#define UART_IRQ UART0_IRQ



class UARTManager
{
public:
    // UARTManager(uint tx, uint rx, uint baud, void(*flushFunc)());
    UARTManager(uint tx, uint rx, uint baud);
    char buff[256];
    void loop();
    void load(void *dst, size_t size);
    void send(void *src, size_t size);
    typedef void (*subscriber)(void* data, size_t length,packet_code code);
    void subscribe(subscriber subFunc, packet_code code);
    void subscribe(subscriber subFunc);
    void handleData();
    int flushCount;

private:
    bool parsed;
    packet_code parsed_packet_code;
    size_t parsed_packet_size;
    std::vector<subscriber> subscribers[PACKET_TYPES_COUNT];
    void (*flushFunc)();
    bool checkCodeValid(packet_code code);
    int getExpectedPacketSize(packet_code code);
};


/**
 * @brief Construct a new UARTManager::UARTManager object, will also configure UART hardware
 *
 * @param tx tx pin
 * @param rx rx pin
 * @param baud baud rate
 * @param irq Pointer to function that will run when uart interrupt is triggered,
 * create a function of type irq_handler_t and call uartmanager->int-handler()
 */
// UARTManager::UARTManager(uint tx, uint rx, uint baud, void(*flushFunc)())
UARTManager::UARTManager(uint tx, uint rx, uint baud)
{
    // Initial member states
    this->parsed = false;

    // Initialize hardware
    uart_init(UART, baud); // Make sure this is consistent with the baud setup on the ESP-01s

    // Setting up GPIO pins
    gpio_set_function(tx, GPIO_FUNC_UART);
    gpio_set_function(rx, GPIO_FUNC_UART);

    // Enabling interrupt
    // irq_set_exclusive_handler(UART_IRQ, irq);
    // irq_set_enabled(UART_IRQ, true);
    // uart_set_irq_enables(UART, true, false);

    // this->flushCount = 0;
    // this->flushFunc = flushFunc;

}


bool UARTManager::checkCodeValid(packet_code code){
    switch (code)
    {
        case PACKET_NODE_STATE:
        case PACKET_PATH:
        case PACKET_GO:
        case PACKET_STOP:
        case PACKET_DIAG_STATE:
            return true;
    
        default:
            return false;
    }
}

int UARTManager::getExpectedPacketSize(packet_code code){
    switch (code)
    {
        case PACKET_REBASE:
            return sizeof(packet_rebase);
        case PACKET_PATH:
            return sizeof(packet_path_point);
        case PACKET_GO:
        case PACKET_STOP:
        case PACKET_DIAG_STATE:
        case PACKET_NODE_STATE:
            return 1;
            
        default: // Flush
            return -1;
    }

}



void UARTManager::handleData()
{
    typedef enum
    {
        IDLE,// Waiting for first byte in packet
        // PREP_PARSE,   // Enabling Parsing
        PARSING, // Parsing bytes untill enough for packet struct
    } uart_state;
    static uart_state state = IDLE;

    static packet_code packet;   // Code of the current packet being parsed from UART
    static size_t packet_size;  // Size of the packet to be saved
    static uint8_t rx_count = 0; // Count of the bytes read

    // Run the state machine for however many bytes are available
    if (uart_is_readable(UART))
    {
        switch (state)
        {
        // Waiting for a packet code byte
        case IDLE:
            packet = (packet_code)uart_getc(UART); // First byte, set the code to determine how the rest are parsed
            if(checkCodeValid(packet)){
                if(getExpectedPacketSize(packet) == 1){
                    state = IDLE;
                    this->parsed = true;
                    this->parsed_packet_size = packet_size;
                    this->parsed_packet_code = packet;
                }else{
                    state = PARSING;                       // Set state to parsing
                    rx_count = 1;                          // Reset rx count
                    buff[0] = packet;
                }
            } else { // Flush
            std::cout << "flushing..." << int(packet) << std::endl;
            //     flushCount = 1;
            //     this->buff[0] = start;    
            //     this->flushFunc();
            //     flushCount = 0;
            }
        break;

        // Parsing bytes as they come in until the relevant struct is full
        case PARSING:
            // Get size of the packet to load
            packet_size = getExpectedPacketSize(packet);

            if(packet_size > 0){
                this->buff[rx_count++] = uart_getc(UART);

                // If enough is read to save to a struct, reset state and raise flag
                if (rx_count >= packet_size)
                {
                    state = IDLE;
                    this->parsed = true;
                    this->parsed_packet_size = packet_size;
                    this->parsed_packet_code = packet;
                }
            } else {
                state = IDLE;
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
                void* data = malloc(this->parsed_packet_size);
                load(data,this->parsed_packet_size);
                subscribers[parsed_packet_code][i](data, this->parsed_packet_size,this->parsed_packet_code);
            }
        this->parsed = false;
    }else {
        handleData();
    }
}

/**
 * @brief Add a new function to be run on updates
 *
 * @param func pointer to function to be run
 * @param code the type of packet, when recieved to subscribe to
 */
void UARTManager::subscribe(subscriber subFunc, packet_code code)
{
    this->subscribers[code].push_back(subFunc);
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
    uart_write_blocking(UART, (uint8_t *)tx_buff, size); // Write to UART
}
