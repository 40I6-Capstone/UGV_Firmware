#include "DifferentialDriveOdometry.hpp"

DifferentialDriveOdometry::DifferentialDriveOdometry(double trackWidth, double initialAngle)
    : DifferentialDriveOdometry(trackWidth, initialAngle, {.x = 0, .y = 0, .theta = 0}) {}

DifferentialDriveOdometry::DifferentialDriveOdometry(double trackWidth, double initialAngle, DifferentialDriveOdometry::Pose initialPose)
    : trackWidth{trackWidth}, initialAngle{initialAngle}, currentPose{initialPose} {}

DifferentialDriveOdometry::Pose DifferentialDriveOdometry::getCurrentPose()
{
    return this->currentPose;
}

DifferentialDriveOdometry::Pose DifferentialDriveOdometry::update(double distL, double distR, double angle)
{
    double dTheta = angle - this->currentPose.theta;
    
}