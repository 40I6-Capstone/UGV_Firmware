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
#include <stdlib.h>
#include <string>
#include <stdlib.h>
#include <string>

#include <iostream>
#include <cmath>


// #include "../common_lib/network_defines.hpp"
// #include "../UGV-ESP/UGV-ESP/network_defines.hpp"

#include "lib/PICO_UARTManager.hpp"
// #include "lib/UART_Subscribers.hpp"
#include "lib/DiffDriveBase/DifferentialDrive.hpp"
#include "lib/PathLoader.hpp"

// #include "lib/MotorControl.hpp"
// #include "lib/QuadEncoder.hpp"
#include "lib/DiffDriveBase/DifferentialDrive.hpp"


#define LOOP_TIME_US 20 * 1E3

// #define TEST_UART

UARTManager *uart_man;

DifferentialDrive *drive;
DifferentialDrive *drive;

PathLoader *pathLoader;

void gpio_isr(uint gpio, uint32_t events)
{
    if (gpio == PIN_ENC_RA || gpio == PIN_ENC_RB)
        drive->updateTicksRight();
    if (gpio == PIN_ENC_LA || gpio == PIN_ENC_LB)
        drive->updateTicksLeft();
}

double getSysTime()
{
    return double(time_us_64()) / 1E6;
}




void pathSubscriber(void *data, size_t length, packet_code code){
    packet_path_point *pathPoint = (packet_path_point*)data;

    pathLoader->load(*pathPoint);

    // for(int i = 0; i< PATH_MAX_POINTS; i++){
    //     packet_path_point tp = pathLoader->getActivePath()[i];
    //     std::cout << "code: " << (int)(tp.code) << std::endl;
    //     std::cout << "x: " << tp.x << std::endl;
    //     std::cout << "y: " << tp.y << std::endl;
    // }
    //     std::cout << "--------" << std::endl;

}





// Main function to execute on core 1 (Mainly used for telemetry)
void core1_main()
{

    // Initalize uart manager
    // uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200,
    //                             []()
    //                             {
    //                                 printf("Flushing %d bytes ", uart_man->flushCount);
    //                                 for(int i = 0; i < uart_man->flushCount; i++){
    //                                     printf(" %d ", uart_man->buff[i]);
    //                                 }
    //                                 printf("\n");
    //                             });

    pathLoader = new PathLoader();
    uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200);

    uart_man->subscribe(pathSubscriber,PACKET_PATH);


    static uint64_t ledTs = time_us_64();
    static bool pinState = false;
    while (1)
    {
        uart_man->loop();
        if (time_us_64() - ledTs > 500 * 1E3)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, pinState);
            pinState = !pinState;
            ledTs = time_us_64();
        }
    }
}

// Main function to execute on core 0 (primary core, interfaces with hardware)
void core0_main()
{

    drive = new DifferentialDrive(getSysTime);
    gpio_set_irq_enabled_with_callback(PIN_ENC_LA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_isr);

    uint64_t lastLoopTs = time_us_64();
    while (1)
    {
        // Main loop
        if (time_us_64() - lastLoopTs > LOOP_TIME_US)
        {

            drive->update();

            // Reset timer
            lastLoopTs = time_us_64();
        }
        else
        {
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
