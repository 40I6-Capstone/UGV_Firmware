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

#include "lib/MotorControl.hpp"

// #define TEST_UART

UARTManager *uart_man;

// Main function to execute on core 1
void core1_main()
{

    while (true)
    {
        // tight_loop_contents();
        uart_man->loop();
    }
}

// Main function to execute on core 0
void core0_main()
{
    std::cout << "SETTING UP MOTORS" << std::endl;
    MotorControl *motor_left = new MotorControl(PIN_MOTOR_LA, PIN_MOTOR_LB);
    MotorControl *motor_right = new MotorControl(PIN_MOTOR_RA, PIN_MOTOR_RB);

    motor_left->setReverse(false);
    motor_right->setReverse(false);
    while (true)
    {
        motor_left->run(1.0);
        motor_right->run(1.0);
    }
}

int main()
{
    // Initialize USB bus
    stdio_usb_init(); // Seems that this needs to happen before starting core1, even if that's where the printing happens

    // Wait until USB is connected before doing anything else
    while (!stdio_usb_connected())
        tight_loop_contents();

    std::cout << "USB CONNECTED" << std::endl;

    /******* INITIALIZE HARDWARE COMMON TO EACH CORE *******/

    // Initalize uart manager
    uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200, []()
                               { uart_man->int_handler(); });

    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}
