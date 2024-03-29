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
#define EN_PRINTS

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

// #include <pico///mutex.h>


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
#include "lib/GeometryUtils/GeometryUtils.hpp"


#define MAX_SPEED 0.15 //0.13 // Max m/s to set motors to
#define MIN_SPEED 0.10 // Min m/s to set motors to before sudden stop
#define RAMP_DOWN_DIST 0.05 // m away from path end to ramp down
#define LOOP_TIME_US 20 * 1E3
#define POSE_TOLERANCE 0.03
#define PUREP_LOOKAHEAD 0.03
// #define LOOP_TIME_US 1500 * 1E3

// #define TEST_UART
// #define EN_TIMEOUT
#define TIMEOUT_US 12*1E6
// #define PWR_TEST
#define PRNT_POSE_CSV
#define PRNT_TARGET
// #define SERVO_TEST
// #define STOP_START_TEST

UARTManager *uart_man;
DifferentialDrive *drive;
PathLoader *pathLoader;
PurePursuit *purep;
ServoControl *arm;

// static //mutex_t poseMtx; 
// static //mutex_t pathMtx; 


double getSysTime(){
    return double(time_us_64()) / 1E6;
}

// State Data
node_state currentState = NODE_IDLE;
bool goFlag = false;
bool stopFlag = false;
bool isFwdFinished = false;
double setV = 0;
double setHeading = 0;




void gpio_isr(uint gpio, uint32_t events)
{
    if (gpio == PIN_ENC_RA || gpio == PIN_ENC_RB)
        drive->updateTicksRight();
    if (gpio == PIN_ENC_LA || gpio == PIN_ENC_LB)
        drive->updateTicksLeft();
}

// UART SUBSCRIBERS
void pathSubscriber(void *data, size_t length, packet_code code){
    std::cout << "Subscriber [Path] fired" << std::endl;
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
void goSubscriber(void *data, size_t length, packet_code code){
    std::cout << "Subscriber [Go] fired" << std::endl;
    goFlag = true;
}   
void stopSubscriber(void *data, size_t length, packet_code code){
    std::cout << "Subscriber [Stop] fired" << std::endl;
    stopFlag = true;
}   
void diagSubscriber(void *data, size_t length, packet_code code){
    std::cout << "Subscriber [Diagnostic] fired" << std::endl;
    // Access pose mtx
    //mutex_enter_blocking(&poseMtx);
    // Get pose
    GeometryUtils::Pose current = drive->getPose();
    // Exit pose mtx
    //mutex_exit(&poseMtx);
    current.theta = GeometryUtils::radToDeg(current.theta);
    // Construct diag state packet with pose and other data
    packet_diag_node_state packet = {
        .v_right = drive->getVRight(),
        .d_right = drive->getDistRight(),
        .v_left = drive->getVLeft(),
        .d_left = drive->getDistLeft(),
    };
    // Write diagnostic state down UART
    uart_man->send(&packet,sizeof(packet_diag_node_state));
}   
void stateSubscriber(void *data, size_t length, packet_code code){
    std::cout << "Subscriber [State] fired" << std::endl;
    // Access pose mtx
    //mutex_enter_blocking(&poseMtx);
    // Get pose
    GeometryUtils::Pose current = drive->getPose();
    // Exit pose mtx
    ////mutex_exit(&poseMtx);
    std::cout << "Got Pose:" << current.x << "," << current.y << std::endl;
    // Construct state packet
    current.theta = GeometryUtils::radToDeg(current.theta);
    GeometryUtils::Pose lookAhead =  purep->getLookAheadPose();
    packet_node_state packet = {
        .x = current.x,
        .y = current.y,
        .v = (drive->getVLeft() + drive->getVRight())/2.,
        .theta = current.theta,
        .state = currentState,
        .x_exp = lookAhead.x,
        .y_exp = lookAhead.y,
        .velocity_exp = setV,
        .heading_exp = setHeading
    };
    // packet_node_state packet = {
    //     .code = 1,
    //     .x = -0.9,
    //     .y = -0.9,
    //     .v = -0.9,
    //     .theta = -0.9,
    //     .state = (node_state)1,
    //     .x_exp =  -0.9,
    //     .y_exp = -0.9,
    //     .velocity_exp = -0.9,
    //     .heading_exp = -0.00001
    // };
    // Write pose down UART
    uart_man->send(&packet,sizeof(packet_node_state));
}   
void rebaseSubscriber(void *data, size_t length, packet_code code){
    #ifdef EN_PRINTS
    std::cout << "Subscriber [Rebase] fired" << std::endl;
    #endif
    packet_rebase *rebasePoint = (packet_rebase*)data;
    GeometryUtils::Pose newPose = {.x = rebasePoint->x, .y = rebasePoint->y, .theta = rebasePoint->heading}; 
    ////mutex_enter_blocking(&poseMtx);
    drive->setPose(newPose);
    ////mutex_exit(&poseMtx);
}   
void setupUARTSubscribers()
{
    uart_man->subscribe(pathSubscriber,PACKET_PATH);
    uart_man->subscribe(goSubscriber,PACKET_GO);
    uart_man->subscribe(stopSubscriber,PACKET_STOP);
    uart_man->subscribe(diagSubscriber,PACKET_DIAG_STATE);
    uart_man->subscribe(stateSubscriber,PACKET_NODE_STATE);
    uart_man->subscribe(rebaseSubscriber,PACKET_REBASE);
}


#ifdef PWR_TEST
double driveSpeed = 0;
auto_init_//mutex(pwrMtx);
#endif

#ifdef SERVO_TEST
uint16_t servoSetpoint = 0;
#endif

// Main function to execute on core 1 (Mainly used for telemetry)
void core1_main()
{

    // //mutexES
    ////mutex_init(&poseMtx);
    ////mutex_init(&pathMtx);
    // std::cout << "Core 1 begin" << std::endl;
    
    pathLoader = new PathLoader([](){
        #ifdef EN_PRINTS
        std::cout << "Buffer Swapped" << std::endl;
        #endif
        //  use path //mutex
        ////mutex_enter_blocking(&pathMtx);
        //  load path into pure pursuit controller
        purep->setPath(pathLoader->getActivePath(),pathLoader->size);
        ////mutex_exit(&pathMtx);

        for(int i = 0; i< PATH_MAX_POINTS; i++){
        packet_path_point tp = pathLoader->getActivePath()[i];
            std::cout << "x: " << tp.x << " y: " << tp.y << std::endl;
        }
        std::cout << "--------" << std::endl;
     });
    uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200);
    setupUARTSubscribers();

    static uint64_t ledTs = time_us_64();
    static bool pinState = false;
    while (1)
    {
        #ifdef PWR_TEST
        double left, right, spd;
        std::cin >> right;
        std::cin >> left;
        std::cin >> spd;
        std::cout << " right: " << right << " left: " << left  << " speed: " << spd <<  std::endl;
        ////mutex_enter_blocking(&pwrMtx);
        drive->setLeftGains(0, left, 0);
        drive->setRightGains(0, right, 0);
        driveSpeed = spd;
        drive->resetControllers();
        ////mutex_exit(&pwrMtx);
        #endif

<<<<<<< HEAD
        // double input;
        // std::cin >> input;
        // if(input > 0){

        //     if((currentState == NODE_IDLE) || (currentState == NODE_STOPPED) ){
        //         std::cout << "SENDING GO" << std::endl;
        //         goFlag = true;
        //     }
        //     if((currentState == NODE_PATH_LEAVE) || (currentState == NODE_PATH_RETURN)){
        //         std::cout << "SENDING STOP" << std::endl;
        //         stopFlag = true;
        //     }
        // }
=======
        #ifdef STOP_START_TEST
        double input;
        std::cin >> input;
        if(input > 0){

            if((currentState == NODE_IDLE) || (currentState == NODE_STOPPED) ){
                std::cout << "SENDING GO" << std::endl;
                goFlag = true;
            }
            if((currentState == NODE_PATH_LEAVE) || (currentState == NODE_PATH_RETURN)){
                std::cout << "SENDING STOP" << std::endl;
                stopFlag = true;
            }
        }
        #endif

        #ifdef SERVO_TEST
        std::cin >> servoSetpoint;
        std::cout <<"servo: " << servoSetpoint << std::endl;
        #endif
>>>>>>> 125ac3f (pre test)

        uart_man->loop();
        if (time_us_64() - ledTs > 500 * 1E3)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, pinState);
            pinState = !pinState;
            ledTs = time_us_64();
        }
    }
}




void followPath(bool isReversed, GeometryUtils::Pose current){
    purep->setReversed(isReversed);
    // Check path //mutex
    ////mutex_enter_blocking(&pathMtx);
    // Compute lookahead
    GeometryUtils::Pose dest = purep->getLookAheadPose(current);

    ////mutex_exit(&pathMtx);
    // Release path //mutex
    // Check if heading is grossly different than target and set V to 0 if necessary first ?
    // Compute heading and V
    double gain;
    double distToEnd = GeometryUtils::distToPoint(current,purep->getLastPose());
    if(distToEnd < RAMP_DOWN_DIST){
        gain = (MAX_SPEED-MIN_SPEED)/RAMP_DOWN_DIST * distToEnd + MIN_SPEED;
    }else {
        gain = MAX_SPEED;
    }

    double heading = purep->getLookAheadHeading(current);

    // bool isOvershoot =  (180. - abs(heading)) < 45.;

    double v = gain * (isReversed? -1. : 1.);

    // if(isReversed || isOvershoot){
    if(isReversed){
        heading = GeometryUtils::flipAngle(heading);
    }
    setV = v;
    setHeading = heading;

    #if defined PRNT_POSE_CSV
    std::cout<< current.x << "," << current.y << "," << GeometryUtils::radToDeg(current.theta)
    #endif 

    #if defined PRNT_TARGET && defined PRNT_POSE_CSV
    << "," << dest.x <<"," << dest.y << "," << heading << "," << v << "," << distToEnd
    #endif

    #if defined PRNT_POSE_CSV
    << std::endl;
    #endif

    // Command drive
    drive->setDriveState(heading,v);
}



void robotFSMLoop(){
    //mutex_enter_blocking(&poseMtx);
    drive->update();
    GeometryUtils::Pose current = drive->getPose();
    //mutex_exit(&poseMtx);
    // std::cout << "Got Pose:" << current.x << "," << current.y << std::endl;

    switch(currentState){
        case NODE_IDLE:
            // Set claw open
            // arm->write(OPEN_POSITION);
            // Stop Drive
            drive->stop();

            if(goFlag){
                currentState = isFwdFinished? NODE_PATH_RETURN : NODE_PATH_LEAVE;
                #ifdef EN_PRINTS
                // std::cout << "State: " << currentState <<std::endl;
                #endif
            }
        break;

        case NODE_STOPPED:
            // Set claw closed
            // arm->write(CLOSED_POSITION);
            // Stop Drive
            drive->stop();
            if(goFlag){
                currentState = isFwdFinished? NODE_PATH_RETURN : NODE_PATH_LEAVE;
                #ifdef EN_PRINTS
                // std::cout << "State: " << currentState <<std::endl;
                #endif
            }
        break;

        case NODE_PATH_LEAVE:
            // Set claw closed
            // arm->write(CLOSED_POSITION);

            if(stopFlag){
                currentState = NODE_STOPPED;
                #ifdef EN_PRINTS
                // std::cout << "State: " << currentState <<std::endl;
                #endif
            }
            // Check if at last point
            else if(GeometryUtils::distToPoint(current, purep->getLastPose()) < POSE_TOLERANCE){
                isFwdFinished = true;
                purep->setReversed(true);
                currentState = NODE_IDLE;
                #ifdef EN_PRINTS
                std::cout << "LEAVE DONE" <<std::endl;
                // std::cout << "State: " << currentState <<std::endl;
                #endif
            } else {
                followPath(false,current);
            }
        break;

        case NODE_PATH_RETURN:
            // Set claw closed
            // arm->write(CLOSED_POSITION);
            if(stopFlag){
                currentState = NODE_STOPPED;
                #ifdef EN_PRINTS
                // std::cout << "State: " << currentState <<std::endl;
                #endif
            }
            else if(GeometryUtils::distToPoint(current, purep->getLastPose()) < POSE_TOLERANCE){
                
                isFwdFinished = false;
                purep->setReversed(false);
                currentState = NODE_IDLE;
                #ifdef EN_PRINTS
                std::cout << "RETURN DONE" <<std::endl;
                // std::cout << "State: " << currentState <<std::endl;
                #endif
            } else {
               followPath(true,current);
            }
        break;
    }
    // Reset flags
    goFlag = false;
    stopFlag = false;
}


// Main function to execute on core 0 (primary core, interfaces with hardware)
void core0_main()
{


    // //mutexES
    //mutex_init(&poseMtx);
    //mutex_init(&pathMtx);

    drive = new DifferentialDrive(getSysTime);
    gpio_set_irq_enabled_with_callback(PIN_ENC_LA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_isr);
    // std::cout << "Drive Setup" << std::endl;
    purep = new PurePursuit(PUREP_LOOKAHEAD,drive->getPose(),false);
    bool reverse = false;
    // purep = new PurePursuit(0.01,{.x = 0.0, .y = 0.0, .theta = 0.0},reverse);
    // std::cout << "PurePursuit Setup" << std::endl;
    // arm = new ServoControl(PIN_SERVO);



    // sleep_ms(2000);
    // std::cout << "Core 0 begin" << std::endl;
    uint64_t lastLoopTs = time_us_64();
    #ifdef EN_TIMEOUT
    uint64_t timeoutTs = time_us_64();
    #endif
    // static Pose testPose = {.x = 0, .y =0, .theta = 0};
    // static int index = 0;
    // purep->setPath(testPath1, 5);
    // purep->setPath(testPath2, 56);
    // purep->setPath(testPath2, 56);
    currentState = NODE_IDLE;
    while (1)
    {
        uint64_t currentTs = time_us_64();
        if (currentTs - lastLoopTs > LOOP_TIME_US)
        {
            // drive->setDriveState(10,driveSpeed);
            #ifdef PWR_TEST
            //mutex_enter_timeout_ms(&pwrMtx,10);
            drive->setLeftV(driveSpeed);
            drive->setRightV(driveSpeed);
            // drive->setLeft(driveSpeed);
            // drive->setRight(driveSpeed);
            std::cout << drive->getVRight() << "," << drive->getVLeft() << ","<< driveSpeed << std::endl;
            //mutex_exit(&pwrMtx);
            #endif

            // std::cout << "Theta: " << RAD_TO_DEG(drive->getPose().theta)
            // << " X:     " << drive->getPose().x 
            // << " Y:     " << drive->getPose().y
            // << std::endl;
            
            // index = (index+1) % 5;
            // Reset timer
            // testPose.x += 0.03*std::cos(DEG_TO_RAD(heading));
            // testPose.y += 0.03*std::sin(DEG_TO_RAD(heading));
                // reverse = !reverse;
                // purep->setReversed(reverse);

            robotFSMLoop();

            // //mutex_enter_blocking(&poseMtx);
            // drive->update();
            // GeometryUtils::Pose current = drive->getPose();
            // //mutex_exit(&poseMtx);

            lastLoopTs = currentTs;
        }
        #ifdef EN_TIMEOUT
        if(currentTs - timeoutTs > TIMEOUT_US){
            currentState = NODE_IDLE;
            std::cout << "State: " << currentState <<std::endl;
            drive->stop();
            while (1)
                tight_loop_contents();
        }
        #endif
        tight_loop_contents();
    }
}

int main()
{
    // Initialize USB bus
    // stdio_usb_init(); // Seems that this needs to happen before starting core1, even if that's where the printing happens
    #ifdef EN_PRINTS
    stdio_init_all();
    #endif

    

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    // Wait until USB is connected before doing anything else
    #ifdef EN_PRINTS
    // while (!stdio_usb_connected())
        // tight_loop_contents();
    printf("USB CONNECTED\n");
    #endif

    /******* INITIALIZE HARDWARE COMMON TO EACH CORE *******/

    sleep_ms(100);
    multicore_launch_core1(core1_main); // Start up core1
    core0_main();                       // core0 main function
}







