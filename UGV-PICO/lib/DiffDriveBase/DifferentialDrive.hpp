#include "PIDController/PIDController.hpp"
#include "PICO_BMX160/PICO_IMU.hpp"
#include "MotorControl/MotorControl.hpp"
#include "QuadEncoder/QuadEncoder.hpp"
#include "DiffDriveOdom/DifferentialDriveOdometry.hpp"
#include <pico/stdlib.h>
#include "DriveConstants.hpp"
#include <iostream>
#include "DriveHWConstants.hpp"
#include <cmath>
#include "../GeometryUtils/GeometryUtils.hpp"
#include "../SlewRateLimiter/SlewRateLimiter.hpp"

#define LEFT_FF(x) (-0.8458*std::pow(x,3) + 4.6697*std::pow(x,2) - 0.2236*x + 0.0517)
#define RIGHT_FF(x) (2.1134*std::pow(x,3) + 1.9547*std::pow(x,2) + 0.2369*x + 0.0479)

#define MAX_ANGLE_SLEW 0.1

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
    SlewRateLimiter *angleSlewLimiter;
    PIDController *leftPID;
    PIDController *rightPID;

    double getRightFF(double setpoint);
    double getLeftFF(double setpoint);

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
    void setTurnGains(double kP, double kI, double kD);
    void resetControllers();
    void stop();
    double distanceToPoint(GeometryUtils::Pose);
    
    GeometryUtils::Pose getPose();
    void setPose(GeometryUtils::Pose newPose);
};

