#include "../lib/PICO_UARTManager.hpp"
#include "../../common_lib/network_defines.h"


void uart_test_setup()
{
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
}


void uart_test_(){
    while (!stdio_usb_connected())
        tight_loop_contents();
}