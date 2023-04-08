#include "DifferentialDriveOdometry.hpp"

DifferentialDriveOdometry::DifferentialDriveOdometry(double initialAngle)
    : DifferentialDriveOdometry(initialAngle, {.x = 0, .y = 0, .theta = 0}) {}

DifferentialDriveOdometry::DifferentialDriveOdometry(double initialAngle, DifferentialDriveOdometry::Pose initialPose)
    : initialAngle{initialAngle}, currentPose{initialPose}
{
    this->distL = 0;
    this->distR = 0;
}

DifferentialDriveOdometry::Pose DifferentialDriveOdometry::getCurrentPose()
{
    return this->currentPose;
}

/**
 * @param distR distance traveled by right encoder
 * @param distL distance traveled by left encoder
 * @param angleRad chassis angle in radians
 */
DifferentialDriveOdometry::Pose DifferentialDriveOdometry::update(double distR, double distL, double angleRad)
{
    // Change in robot heading
    double angle = angleRad - this->initialAngle;    // remove initial offset
    double dTheta = angle - this->currentPose.theta; // comupte change since last update

    // Computing translational vector in local co-ords
    double deltaRight = distR - this->distR;
    double deltaLeft = distL - this->distL;
    double distArc = (deltaLeft + deltaRight) / 2.;
    this->distR = distR;
    this->distL = distL;

    Eigen::VectorXd twistRobot(3);
    twistRobot << distArc, 0, dTheta;

    // Using Pose exponential
    double termSin, termCos;

    if (abs(dTheta) < 1E-9)
    {
        termSin = 1. - 1. / 6. * dTheta * dTheta;
        termCos = 0.5 * dTheta;
    }
    else
    {
        termSin = std::sin(dTheta) / dTheta;
        termCos = (1 - std::cos(dTheta)) / dTheta;
    }

    Eigen::MatrixXd transform(3, 3);
    transform << termSin, -termCos, 0,
                 termCos, termSin, 0,
                 0, 0, 1;

    Eigen::MatrixXd rotation(3, 3);
    rotation << std::cos(this->currentPose.theta), -std::sin(this->currentPose.theta), 0,
                std::sin(this->currentPose.theta), std::cos(this->currentPose.theta), 0,
                0, 0, 1;

    Eigen::VectorXd poseDelta(3);
    poseDelta = rotation * transform * twistRobot;

    this->currentPose.x += poseDelta(0);
    this->currentPose.y += poseDelta(1);
    this->currentPose.theta += poseDelta(2);

    return this->currentPose;
}

void DifferentialDriveOdometry::setPose(DifferentialDriveOdometry::Pose pose, double gyroAngle)
{
    this->initialAngle = gyroAngle;
    this->currentPose = pose;
}
