
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

public:
    DifferentialDriveOdometry(double trackWidth,double initialAngle, Pose initialPose);
    DifferentialDriveOdometry(double trackWidth,double initialAngle);
    Pose getCurrentPose();
    Pose update(double distL, double distR, double angle);
};
