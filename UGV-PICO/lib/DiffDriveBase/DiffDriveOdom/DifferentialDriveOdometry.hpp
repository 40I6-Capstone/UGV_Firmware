#include <math.h>
// #include "../Eigen/Dense"
// #include "~/lib/eigen/Eigen/Dense"
#include <Eigen/Dense>
#include <cmath>
#include "../../GeometryUtils/GeometryUtils.hpp"


class DifferentialDriveOdometry
{
public:
    DifferentialDriveOdometry(double initialAngle, GeometryUtils::Pose initialPose);
    DifferentialDriveOdometry(double initialAngle);
    GeometryUtils::Pose getCurrentPose();
    GeometryUtils::Pose update(double distL, double distR, double angle);
    void setPose(GeometryUtils::Pose newPose, double gyroAngle);

private:
    GeometryUtils::Pose currentPose;
    double initialAngle; // Initial gyro angle, used to offset later
    double distR,distL; // Last distance report from each encoder
};
