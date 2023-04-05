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

#include <iostream>
#include <cmath>

// #include "lib/hw_defines.h"
// include "lib/number_constants.h"
#include "../common_lib/network_defines.h"

#include "lib/PICO_UARTManager.hpp"
#include "lib/DiffDriveBase/DifferentialDrive.hpp"

// #include "lib/MotorControl.hpp"
// #include "lib/QuadEncoder.hpp"
// #include "lib/PICO_BMX160/PICO_DFRobot_BMX160.h"
// #include "lib/PICO_BMX160/PICO_IMU.hpp"
// #include "lib/DiffDriveOdom/DifferentialDriveOdometry.hpp"

#define LOOP_TIME_US 20*1E3




// #define TEST_UART

UARTManager *uart_man;

DifferentialDrive *drive;

void gpio_isr(uint gpio, uint32_t events)
{
    if (gpio == PIN_ENC_RA || gpio == PIN_ENC_RB)
    {
        drive->updateTicksRight();
    }
    if (gpio == PIN_ENC_LA || gpio == PIN_ENC_LB)
    {
        drive->updateTicksLeft();
    }
}


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



    drive = new DifferentialDrive();
    gpio_set_irq_enabled_with_callback(0, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_isr);


    uint64_t lastLoopTs = time_us_64();
    while (1)
    {
        // Main loop
        if(time_us_64()-lastLoopTs > LOOP_TIME_US){

       
            drive->update();
            // Reset timer
            lastLoopTs = time_us_64();
        }
        else{
            tight_loop_contents();
        }
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
    printf("USB CONNECTED\n");

    /******* INITIALIZE HARDWARE COMMON TO EACH CORE *******/

    sleep_ms(10);
    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}
