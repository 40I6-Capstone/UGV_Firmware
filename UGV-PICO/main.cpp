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
MotorControl *motor_right;
MotorControl *motor_left;
// Main function to execute on core 1 (Mainly used for telemetry)
void core1_main()
{

    // Initalize uart manager
    // uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200, []()
    //                            { uart_man->int_handler(); });

    // uart_man->subscribe([]()
    //                     {
    //                             packet_path_point pack;
    //                             uart_man->load(&pack, sizeof(pack));
    //                             std::cout << "x: " << pack.x << std::endl;
    //                             std::cout << "y: " << pack.y << std::endl;
    //                             std::cout << "v: " << pack.v << std::endl;
    //                             std::cout << "theta: " << pack.theta << std::endl;
    //                             std::cout << "ts_ms: " << pack.ts_ms << std::endl; },
    //                     PACKET_PATH);

    while (1)
    {
        // printf("Core1 Ping\n");
        // tight_loop_contents();
        // sleep_ms(100);
        // uart_man->loop();
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }
}

// Main function to execute on core 0 (primary core, interfaces with hardware)
void core0_main()
{
    motor_left = new MotorControl(PIN_MOTOR_LA, PIN_MOTOR_LB);
    motor_right = new MotorControl(PIN_MOTOR_RA, PIN_MOTOR_RB);
    motor_left->setReverse(false);
    motor_right->setReverse(false);

    while (1)
    {
        // printf("Core0 Ping\n");
        int left;
        int right;
        std::cin >> left >> right;

        double outputL = (double)left / 100.;
        double outputR = (double)right / 100.;
        std::cout << "Left: " << outputL << " | Right: " << outputR << std::endl;
        motor_left->run(outputL);
        motor_right->run(outputR);
        // motor_left->run(0.5);
        // motor_right->run(-0.5);
    }
}

int main()
{
    // Initialize USB bus
    // stdio_usb_init(); // Seems that this needs to happen before starting core1, even if that's where the printing happens
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Wait until USB is connected before doing anything else
    while (!stdio_usb_connected())
        tight_loop_contents();
    printf("USB CONNECTED");

    /******* INITIALIZE HARDWARE COMMON TO EACH CORE *******/

    sleep_ms(10);
    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}
