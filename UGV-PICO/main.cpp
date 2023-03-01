/**
 * @file main.cpp
 * @author Shaqeeb Momen (shaqeebmomen@yahoo.ca)
 * @brief Firmware running on the RPI-PICO mcu on the UGV
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>

#include <math.h>
#include <iostream>

#include "lib/PICO_UARTManager.hpp"
#include "lib/hw_defines.h"
#include "../common_lib/network_defines.h"

#define TEST_UART

UARTManager *uart_man;

// Main function to execute on core 1
void core1_main()
{

    while (true)
    {
        tight_loop_contents();
    }
}

// Main function to execute on core 0
void core0_main()
{

    // Initalize uart manager

    uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200, []()
                               { uart_man->int_handler(); });
    while (true)
    {

        uart_man->loop();

#ifdef TEST_UART
        // Subscribe a printing function
        uart_man->subscribe([]()
                            {
                                packet_path_point pack;
                                uart_man->load(&pack, sizeof(pack));
                                std::cout << "x: " << pack.x << std::endl;
                                std::cout << "y: " << pack.y << std::endl;
                                std::cout << "v: " << pack.v << std::endl;
                                std::cout << "theta: " << pack.theta << std::endl;
                                std::cout << "ts_ms: " << pack.ts_ms << std::endl; },
                            PACKET_PATH);

        static uint32_t uart_test_ts = 0;
        static packet_node_state pack = {
            .x = 10.0,
            .y = -3.0,
            .v = -100.0,
            .theta = M_PI,
            .ts_ms = 0,
            .state = NODE_IDLE};

        pack.ts_ms = to_ms_since_boot(get_absolute_time()); // Update timestamp
        // Every 2 seconds, send the packet over
        if (to_ms_since_boot(get_absolute_time()) - uart_test_ts > 2000)
        {
            uart_man->send(&pack, sizeof(pack));
            uart_test_ts = to_ms_since_boot(get_absolute_time());
        }
#endif
    }
}

int main()
{

    // Initialize USB bus
    stdio_usb_init(); // Seems that this needs to happen before starting core1, even if that's where the printing happens

#ifdef TEST_UART
    while (!stdio_usb_connected())
        tight_loop_contents();
#endif

    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}
