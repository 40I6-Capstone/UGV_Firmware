#include <math.h>
// #include "../Eigen/Dense"
class DifferentialDriveOdometry
{

    typedef struct
    {
        double x;
        double y;
        double theta;
    } Pose;

private:
    double trackWidth;
    Pose currentPose;
    double initialAngle;
    double dTheta;
    double arcR;
    double d_r;
    double d_l;
    // Terms for constant global matrix
    double termCosG; 
    double termSinG;
    void updateGlobalTerms();
    

public:
    DifferentialDriveOdometry(double trackWidth,double initialAngle, Pose initialPose);
    DifferentialDriveOdometry(double trackWidth,double initialAngle);
    Pose getCurrentPose();
    Pose update(double distL, double distR, double angle);
    void setPose(Pose newPose, double gyroAngle);
};
