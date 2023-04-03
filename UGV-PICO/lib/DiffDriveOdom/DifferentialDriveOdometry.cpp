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

    // Using Pose exponential
    // double termSinR = sin(dTheta)/dTheta;
    // double termCosR_A = (cos(dTheta)-1)/dTheta;
    // double termCosR_B = (1-cos(dTheta))/dTheta;

    // double dx = 

    return this->currentPose;
    
}

void DifferentialDriveOdometry::setPose(DifferentialDriveOdometry::Pose pose, double gyroAngle){
    this->initialAngle = gyroAngle;
    this->currentPose = pose;
    updateGlobalTerms();
}

void DifferentialDriveOdometry::updateGlobalTerms(){
    // this->termSinG = sin(this->initialAngle);
    // this->termCosG = cos(this->initialAngle);
}