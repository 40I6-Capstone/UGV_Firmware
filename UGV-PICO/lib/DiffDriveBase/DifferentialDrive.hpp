#include "PIDController/PIDController.hpp"
#include "PICO_BMX160/PICO_IMU.hpp"
#include "MotorControl.hpp"
#include "QuadEncoder.hpp"
#include "DiffDriveOdom/DifferentialDriveOdometry.hpp"
#include <pico/stdlib.h>
#include "DriveConstants.hpp"
#include <iostream>
#include "DriveHWConstants.hpp"
#include <cmath>


class DifferentialDrive
{
private:
    MotorControl *motor_right;
    MotorControl *motor_left;

    QuadEncoder *enc_right;
    QuadEncoder *enc_left;

    PICO_IMU *imu;

    DifferentialDriveOdometry *odom;

    PIDController *anglePID;
    PIDController *leftPID;
    PIDController *rightPID;

    double getFF(double setpoint);

public:
    DifferentialDrive(double(*getSysTime)());
    void update();
    void updateTicksRight();
    void updateTicksLeft();
    void setLeft(double output);
    void setLeftV(double setpoint);
    void setRightV(double setpoint);
    void setRight(double output);
    double getDistLeft();
    double getDistRight();
    double getVLeft();
    double getVRight();
    double getAngle();
    void setDriveState(double theta, double v);
    void setLeftGains(double kP, double kI, double kD);
    void setRightGains(double kP, double kI, double kD);
    void resetControllers();
    void stop();
    double distanceToPoint(Pose);

    
    Pose getPose();
    void setPose(Pose newPose);
};

