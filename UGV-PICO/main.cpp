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



#define LOOP_TIME_US 20 * 1E3

// #define TEST_UART

UARTManager *uart_man;
DifferentialDrive *drive;
PathLoader *pathLoader;

void gpio_isr(uint gpio, uint32_t events)
{
    if (gpio == PIN_ENC_RA || gpio == PIN_ENC_RB)
        drive->updateTicksRight();
    if (gpio == PIN_ENC_LA || gpio == PIN_ENC_LB)
        drive->updateTicksLeft();
}

double getSysTime(){return double(time_us_64()) / 1E6;}



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

void goSubscriber(void *data, size_t length, packet_code code){}   
void stopSubscriber(void *data, size_t length, packet_code code){}   
void diagSubscriber(void *data, size_t length, packet_code code){}   
void stateSubscriber(void *data, size_t length, packet_code code){}   

void setupUARTSubscribers()
{
    uart_man->subscribe(pathSubscriber,PACKET_PATH);
    uart_man->subscribe(goSubscriber,PACKET_GO);
    uart_man->subscribe(stopSubscriber,PACKET_STOP);
    uart_man->subscribe(diagSubscriber,PACKET_DIAG_STATE);
    uart_man->subscribe(stateSubscriber,PACKET_NODE_STATE);
}




// Main function to execute on core 1 (Mainly used for telemetry)
void core1_main()
{
    
    pathLoader = new PathLoader([](){
        std::cout << "Buffer Swapped" << std::endl;
     });
    uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200);
    setupUARTSubscribers();

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

    sleep_ms(2000);
    uint64_t lastLoopTs = time_us_64();
    uint64_t motorLoop = time_us_64();
    double leftPwr = 0.3;
    double rightPwr = 0.3;
    // drive->setLeftV(leftPwr);
    // drive->setRightV(rightPwr);
    while (1)
    {
        uint64_t currentTs = time_us_64();
        // Main loop
        if (currentTs - lastLoopTs > LOOP_TIME_US)
        {
            drive->setDriveState(0,0.3);
            drive->update();
            std::cout << "Theta: " << drive->getAngle()
            << " RightV: " << drive->getVRight() 
            << " LeftV: " << drive->getVLeft()
            << std::endl;

            // Reset timer
            lastLoopTs = currentTs;
        }
        // if(currentTs - motorLoop > 3000 *1E3){
        //     drive->setLeftV(leftPwr);
        //     drive->setRightV(rightPwr);

        //     leftPwr = 0.0;
        //     rightPwr = 0.0;
        //     motorLoop = currentTs;
        // }
        // else
        // {
            tight_loop_contents();
        // }
    }
}

int main()
{
    // Initialize USB bus
    // stdio_usb_init(); // Seems that this needs to happen before starting core1, even if that's where the printing happens
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    // Wait until USB is connected before doing anything else
    while (!stdio_usb_connected())
        tight_loop_contents();
    printf("USB CONNECTED\n");

    /******* INITIALIZE HARDWARE COMMON TO EACH CORE *******/

    sleep_ms(10);
    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}
