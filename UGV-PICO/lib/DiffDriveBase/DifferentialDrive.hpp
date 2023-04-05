#include "PIDController/PIDController.hpp"
#include "PICO_BMX160/PICO_IMU.hpp"
#include "MotorControl.hpp"
#include "QuadEncoder.hpp"
#include "DiffDriveOdom/DifferentialDriveOdometry.hpp"
#include <pico/stdlib.h>
#include "number_constants.h"
#include <iostream>
#include "hw_defines.h"

#define DEG_TO_RAD(deg) (deg*M_PI/180.)
#define RAD_TO_DEG(rad) (rad*180./M_PI)

class DifferentialDrive
{
private:
    MotorControl *motor_right;
    MotorControl *motor_left;

    QuadEncoder *enc_right;
    QuadEncoder *enc_left;

    PICO_IMU *imu;

    DifferentialDriveOdometry *odom;

public:
    DifferentialDrive(/* args */);
    void update();
    void updateTicksRight();
    void updateTicksLeft();
};

DifferentialDrive::DifferentialDrive(/* args */)
{
    this->motor_right = new MotorControl(PIN_MOTOR_RA, PIN_MOTOR_RB);
    this->motor_left = new MotorControl(PIN_MOTOR_LA, PIN_MOTOR_LB);
    motor_right->setReverse(false);
    motor_left->setReverse(false);

    this->enc_right = new QuadEncoder(PIN_ENC_RA, PIN_ENC_RB);
    this->enc_left = new QuadEncoder(PIN_ENC_LA, PIN_ENC_LB);
    enc_right->setInverted(false);
    enc_left->setInverted(false);
    enc_right->setConversionFactor(M_PER_REV);
    enc_left->setConversionFactor(M_PER_REV);

    this->imu = new PICO_IMU(I2C_INST, PIN_SDA, PIN_SCL);
    bool imuSetupSuccess;
    imuSetupSuccess = imu->begin();
    std::cout << (imuSetupSuccess ? "Setup Success" : "Setup Fail") << std::endl;
    sleep_ms(100);
    imu->update();

    odom = new DifferentialDriveOdometry(DEG_TO_RAD(imu->getAngle()));
}



void DifferentialDrive::update()
{
    odom->update(enc_right->getPosition(), enc_left->getPosition(), DEG_TO_RAD(imu->getAngle()));
    std::cout << "Gyro: " << imu->getAngle() << std::endl;
    std::cout << "EncR: " << enc_right->getPosition() << " EncL: " << enc_left->getPosition() << std::endl;
    DifferentialDriveOdometry::Pose currentPose = odom->getCurrentPose();
    std::cout << "Pose: " << currentPose.x << " " << currentPose.y << " " << RAD_TO_DEG(currentPose.theta) << " " << std::endl;
}

void DifferentialDrive::updateTicksRight(){
    this->enc_right->updateTicks();
}

void DifferentialDrive::updateTicksLeft(){
    this->enc_left->updateTicks();
}