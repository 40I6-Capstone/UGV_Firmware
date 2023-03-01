/**
 * @file UGV-ESP.ino
 * @author Shaqeeb Momen (shaqeebmomen@yahoo.ca)
 * @brief Firmware for the ESP-01s module running on the UGV
 * @version 0.1
 * @date 2022-11-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// TODO when sending paths over the UART, send 1 point at a time with repeat packet codes
// TODO dont bother sending WHO_AM_I through the UART pipe, just handle it on the ESP
#include "./lib/ESP_UARTManager.hpp"
#include "./lib/network_defines.h"

#define TEST_UART

#ifdef TEST_UART
#include "lib/uart_test_helpers.hpp"
#endif

UARTManager uart;

void serialEvent()
{
    uart.int_handler();
}

void setup()
{
    // UART Hardware
    Serial.begin(115200);
// Attach subscribers to the uart
// uart.subscribe(/*Function*/,/*packet code*/);
#ifdef TEST_UART
    uart.subscribe([]() {
        // TODO ideally the uartmanager is written in a manner that allows for the loaded stuct to be passed automatically
        packet_node_state pack; // Declare structure 
        uart.load(&pack,sizeof(pack)); // Load data from manager buffer into structure
        print_node_state(pack); // Print data
    }, PACKET_NODE_STATE); // Respond to a node state packet
#endif
}


void loop()
{
    uart.loop(); // Fires any subscribed functions if the relevant packet was loaded
    #ifdef TEST_UART
    static uint32_t test_ts = 0;
    // Fire a packet every 1.5s
    if (millis() - test_ts > 1500)
    {
        // Send the test path point packet over uart
        test_path_point.ts_ms = millis(); // Change the timestamp to validate new values going through
        uart.send(&test_path_point,sizeof(test_path_point));
        test_ts = millis();
    }
    #endif
}