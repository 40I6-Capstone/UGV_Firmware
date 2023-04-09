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

#include <pico/mutex.h>


// #include "../common_lib/network_defines.hpp"
// #include "../UGV-ESP/UGV-ESP/network_defines.hpp"

#include "lib/PICO_UARTManager.hpp"
// #include "lib/UART_Subscribers.hpp"
#include "lib/DiffDriveBase/DifferentialDrive.hpp"
#include "lib/PathLoader.hpp"
#include "lib/ServoControl.hpp"
// #include "lib/GeometryDefines.hpp"
#include "lib/PurePursuit.hpp"
#include "lib/TestPath.hpp"



// #define LOOP_TIME_US 20 * 1E3
#define LOOP_TIME_US 1500 * 1E3

// #define TEST_UART

// UARTManager *uart_man;
// DifferentialDrive *drive;
// PathLoader *pathLoader;
PurePursuit *purep;
// ServoControl *arm;



// MUTEXES
auto_init_mutex(pwrMtx);
auto_init_mutex(poseMtx);

// void gpio_isr(uint gpio, uint32_t events)
// {
//     if (gpio == PIN_ENC_RA || gpio == PIN_ENC_RB)
//         drive->updateTicksRight();
//     if (gpio == PIN_ENC_LA || gpio == PIN_ENC_LB)
//         drive->updateTicksLeft();
// }



// UART SUBSCRIBERS
// void pathSubscriber(void *data, size_t length, packet_code code){
//     packet_path_point *pathPoint = (packet_path_point*)data;
//     pathLoader->load(*pathPoint);

//     // for(int i = 0; i< PATH_MAX_POINTS; i++){
//     //     packet_path_point tp = pathLoader->getActivePath()[i];
//     //     std::cout << "code: " << (int)(tp.code) << std::endl;
//     //     std::cout << "x: " << tp.x << std::endl;
//     //     std::cout << "y: " << tp.y << std::endl;
//     // }
//     //     std::cout << "--------" << std::endl;

// }

// void goSubscriber(void *data, size_t length, packet_code code){}   
// void stopSubscriber(void *data, size_t length, packet_code code){}   
// void diagSubscriber(void *data, size_t length, packet_code code){}   
// void stateSubscriber(void *data, size_t length, packet_code code){}   

// void rebaseSubscriber(void *data, size_t length, packet_code code){
//     packet_rebase *rebasePoint = (packet_rebase*)data;
//     Pose newPose = {.x = rebasePoint->x, .y = rebasePoint->y, .theta = rebasePoint->heading}; 
//     mutex_enter_blocking(&poseMtx);
//     // drive->setPose(newPose);
//     mutex_exit(&poseMtx);
// }   

// void setupUARTSubscribers()
// {
//     uart_man->subscribe(pathSubscriber,PACKET_PATH);
//     uart_man->subscribe(goSubscriber,PACKET_GO);
//     uart_man->subscribe(stopSubscriber,PACKET_STOP);
//     uart_man->subscribe(diagSubscriber,PACKET_DIAG_STATE);
//     uart_man->subscribe(stateSubscriber,PACKET_NODE_STATE);
//     uart_man->subscribe(rebaseSubscriber,PACKET_REBASE);
// }



double driveSpeed = 0.2;

// Main function to execute on core 1 (Mainly used for telemetry)
void core1_main()
{
    std::cout << "Core 1 begin" << std::endl;
    
    // pathLoader = new PathLoader([](){
    //     std::cout << "Buffer Swapped" << std::endl;
    //      load path into pure pursuit controller
    //      use path mutex
    //  });
    // uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200);
    // setupUARTSubscribers();

    static uint64_t ledTs = time_us_64();
    static bool pinState = false;
    while (1)
    {
        // double left, right, spd;
        // std::cin >> right;
        // std::cin >> left;
        // std::cin >> spd;
        // std::cout << " right: " << right << " left: " << left  << " speed: " << spd <<  std::endl;
        // mutex_enter_blocking(&pwrMtx);
        // drive->setLeftGains(left, 0, 0);
        // drive->setRightGains(right, 0, 0);
        // driveSpeed = spd;
        // drive->resetControllers();
        // mutex_exit(&pwrMtx);



        // uart_man->loop();
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

    // drive = new DifferentialDrive([]()->double{
    //     return double(time_us_64()) / 1E6;
    // });
    // gpio_set_irq_enabled_with_callback(PIN_ENC_LA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_isr);

    // purep = new PurePursuit(0.1,drive->getPose(),false);
    bool reverse = false;
    purep = new PurePursuit(0.02,{.x = 0.0, .y = 0.0, .theta = 0.0},reverse);
    // arm = new ServoControl(PIN_SERVO);


    sleep_ms(2000);
    std::cout << "Core 0 begin" << std::endl;
    uint64_t lastLoopTs = time_us_64();
    uint64_t motorLoop = time_us_64();
    uint64_t odomTs = time_us_64();

    static Pose testPose = {.x = 0, .y =0, .theta = 0};
    // static int index = 0;
    purep->setPath(testPath1, 5);
    while (1)
    {
        uint64_t currentTs = time_us_64();
        // if(currentTs - odomTs >10*1E3)
        // {
        //     mutex_enter_blocking(&poseMtx);
        //     // drive->update();
        //     mutex_exit(&poseMtx);
        //     odomTs = currentTs;
        // }
        if (currentTs - lastLoopTs > LOOP_TIME_US)
        {
            // drive->setDriveState(10,driveSpeed);


            // mutex_enter_timeout_ms(&pwrMtx,10);
            // drive->setLeftV(driveSpeed);
            // drive->setRightV(driveSpeed);
            // std::cout << "right v: " << drive->getVRight() << " left v: " << drive->getVLeft() << " setpoint: "<< driveSpeed << std::endl;
            // mutex_exit(&pwrMtx);

            // std::cout << "Theta: " << RAD_TO_DEG(drive->getPose().theta)
            // << " X:     " << drive->getPose().x 
            // << " Y:     " << drive->getPose().y
            // << std::endl;

            std::cout << "PoseX:   " << testPose.x << " PoseY:   " << testPose.y << std::endl;
            Pose dest = purep->getLookAheadPose(testPose);
            // Pose dest = purep->poseFromPacket(testPath1[index]);
            double heading = purep->getLookAheadHeading(testPose);
            std::cout << "TargetX: " << dest.x <<     " TargetY: " << dest.y << " Heading: " << heading  << "\n" << std::endl; 
            // index = (index+1) % 5;
            // Reset timer
            testPose.x += 0.03*std::cos(DEG_TO_RAD(heading));
            testPose.y += 0.03*std::sin(DEG_TO_RAD(heading));
            if(distToPoint(testPose, purep->getLastPose()) < 0.04){
                std::cout << "PATH DONE" <<std::endl;
                reverse = !reverse;
                purep->setReversed(reverse);
            }
            lastLoopTs = currentTs;
        }
        // if(currentTs - motorLoop > 3500 *1E3){
        //     // driveSpeed = 0;
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



void robotFSMLoop(){
    typedef enum {
        IDLE, // Claw open, waiting to move
        STOPPED, // Claw closed, waiting to move
        LEAVING, // Claw closed, moving 
        RETURNING // Claw closed, moving
    } robState;

    static robState currentState = IDLE;

    switch(currentState){
        case IDLE:
            // Set claw open
            // arm->write(OPEN_POSITION);
            // Stop Drive
            // drive->stop();

        break;

        case STOPPED:
            // Set claw closed
            // arm->write(CLOSED_POSITION);
            // Stop Drive
            // drive->stop();
        break;

        case LEAVING:
            // Set claw closed
            // arm->write(CLOSED_POSITION);
            

            // Check path mutex
            // Check if heading to last point
            // Compute lookahead
            // Release path mutex
            // Check if heading is grossly different than target and set V to 0 if necessary first
            // Compute heading and V
            // Command drive

        break;

        case RETURNING:
            // Set claw closed
            // arm->write(CLOSED_POSITION);
            // Check path mutex
            // Check if heading to last point
            // Compute lookahead
            // Release path mutex
            // Check if heading is grossly different than target and set V to 0 if necessary first
            // Compute heading and V
            // Command drive

        break;
    }

}