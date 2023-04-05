#include <math.h>
// #include "../Eigen/Dense"
// #include "~/lib/eigen/Eigen/Dense"
#include <Eigen/Dense>
#include <cmath>


class DifferentialDriveOdometry
{
public:
    typedef struct
    {
        double x;
        double y;
        double theta;
    } Pose;

    DifferentialDriveOdometry(double initialAngle, Pose initialPose);
    DifferentialDriveOdometry(double initialAngle);
    Pose getCurrentPose();
    Pose update(double distL, double distR, double angle);
    void setPose(Pose newPose, double gyroAngle);

private:
    Pose currentPose;
    double initialAngle; // Initial gyro angle, used to offset later
    double distR,distL; // Last distance report from each encoder
};