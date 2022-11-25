#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/stdio_usb.h>

#include "lib/UARTManager.hpp"
#include "lib/hw_defines.h"
#include "../common_lib/network_defines.h"

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
    while (true)
    {
        tight_loop_contents();
    }
}

int main()
{

    // Initialize USB bus
    stdio_usb_init(); // Seems that this needs to happen before starting core1, even if that's where the printing happens

    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}
