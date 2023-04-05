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

#include "lib/hw_defines.h"
#include "lib/number_constants.h"
// #include "../common_lib/network_defines.hpp"

#include "lib/PICO_UARTManager.hpp"
#include "lib/MotorControl.hpp"
#include "lib/QuadEncoder.hpp"
// #include "lib/PICO_BMX160/PICO_DFRobot_BMX160.h"
#include "lib/PICO_BMX160/PICO_IMU.hpp"
#include "lib/DiffDriveOdom/DifferentialDriveOdometry.hpp"

#define LOOP_TIME_US 1000*1E3 //20*1E3

#define DEG_TO_RAD(deg) (deg*M_PI/180.)
#define RAD_TO_DEG(rad) (rad*180./M_PI)


// #define TEST_UART

UARTManager *uart_man;
MotorControl *motor_right;
MotorControl *motor_left;

QuadEncoder *enc_right;
QuadEncoder *enc_left;

PICO_IMU *imu;

DifferentialDriveOdometry *odom;


void gpio_isr(uint gpio, uint32_t events)
{
    if (gpio == PIN_ENC_RA || gpio == PIN_ENC_RB)
    {
        enc_right->updateTicks();
    }
    if (gpio == PIN_ENC_LA || gpio == PIN_ENC_LB)
    {
        enc_left->updateTicks();
    }
}

// Main function to execute on core 1 (Mainly used for telemetry)
void core1_main()
{

    // Initalize uart manager
    uart_man = new UARTManager(PIN_UART0_TX, PIN_UART0_RX, 115200,
                                []()
                                {
                                    printf("Flush Data:\n");
                                    char buff[uart_man->flushIndex];
                                    uart_man->load(buff,uart_man->flushIndex);
                                    puts(buff);
                                    printf("\n");
                                });

    uart_man->subscribe([]()
                        {
                                packet_path_point pack;
                                uart_man->load(&pack, sizeof(pack));
                                std::cout << "x: " << pack.x << std::endl;
                                std::cout << "y: " << pack.y << std::endl;
                        },
                        PACKET_PATH);


        

    static uint64_t ts = time_us_64();
    while (1)
    {
        // tight_loop_contents();
        uart_man->loop();
        // sleep_ms(50);
        if(time_us_64()-ts > 500*1E3){
            printf("Core1 Ping\n");
            gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get_out_level(PICO_DEFAULT_LED_PIN));
            ts = time_us_64();
        }
        // sleep_ms(500);
        // gpio_put(PICO_DEFAULT_LED_PIN, 0);
        // sleep_ms(500);
    }
}

// Main function to execute on core 0 (primary core, interfaces with hardware)
void core0_main()
{
    motor_right = new MotorControl(PIN_MOTOR_RA, PIN_MOTOR_RB);
    motor_left = new MotorControl(PIN_MOTOR_LA, PIN_MOTOR_LB);
    motor_right->setReverse(false);
    motor_left->setReverse(false);

    enc_right = new QuadEncoder(PIN_ENC_RA, PIN_ENC_RB);
    enc_left = new QuadEncoder(PIN_ENC_LA, PIN_ENC_LB);
    enc_right->setInverted(false);
    enc_left->setInverted(false);
    enc_right->setConversionFactor(M_PER_REV);
    enc_left->setConversionFactor(M_PER_REV);


    gpio_set_irq_enabled_with_callback(PIN_ENC_RA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_isr);

    imu = new PICO_IMU(I2C_INST, PIN_SDA, PIN_SCL);
    bool imuSetupSuccess;
    imuSetupSuccess = imu->begin();
    std::cout << (imuSetupSuccess ? "Setup Success" : "Setup Fail") << std::endl;
    sleep_ms(100);
    imu->update();

    odom = new DifferentialDriveOdometry(DEG_TO_RAD(imu->getAngle()));


    uint64_t lastLoopTs = time_us_64();
    while (1)
    {
        if(time_us_64()-lastLoopTs > LOOP_TIME_US){

        odom->update(enc_right->getPosition(),enc_left->getPosition(),DEG_TO_RAD(imu->getAngle()));
        // std::cout << "Gyro: " << imu->getAngle() << std::endl;
        // std::cout << "EncR: " << enc_right->getPosition() << " EncL: " << enc_left->getPosition() << std::endl;
        DifferentialDriveOdometry::Pose currentPose = odom->getCurrentPose();
        // std::cout << "Pose: " << currentPose.x << " " << currentPose.y << " " << RAD_TO_DEG(currentPose.theta) << " " << std::endl; 

            // Reset timer
            lastLoopTs = time_us_64();
        }
        else{
            tight_loop_contents();
        }
        // printf("Core0 Ping\n");
        // int left;
        // int right;
        // std::cin >> left >> right;

        // double outputL = (double)left / 100.;
        // double outputR = (double)right / 100.;
        // std::cout << "Left: " << outputL << " | Right: " << outputR << std::endl;
        // motor_left->set(outputL);
        // motor_right->set(outputR);
        // std::cout << "Left:"
        //           << enc_left->getPosition()
        //           << ",Right:"
        //           << enc_right->getPosition()
        //           << ",LeftV:"
        //           << enc_left->getVelocity()
        //           << ",RightV:"
        //           << enc_right->getVelocity() << std::endl;

        // sBmx160SensorData_t Omagn, Ogyro, Oaccel;

        // imu->getAllData(&Omagn, &Ogyro, &Oaccel);
        // std::cout << "M |"
        //           << " X: " << Omagn.x
        //           << " Y: " << Omagn.y
        //           << " Z: " << Omagn.z
        //           << " uT" << std::endl;
        // std::cout << "G |"
        //           << " X: " << Ogyro.x
        //           << " Y: " << Ogyro.y
        //           << " Z: " << Ogyro.z
        //           << " dps" << std::endl;
        // std::cout << "A |"
        //           << " X: " << Oaccel.x
        //           << " Y: " << Oaccel.y
        //           << " Z: " << Oaccel.z
        //           << " m/s^2"
        //           << "\n"
        //           << std::endl;
        // std::cout << std::sqrt(Omagn.x * Omagn.x + Omagn.y * Omagn.y + Omagn.z * Omagn.z) << std::endl;
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
