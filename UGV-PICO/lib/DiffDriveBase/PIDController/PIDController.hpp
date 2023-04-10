#include <stdlib.h>
#include "../../GeometryUtils/GeometryUtils.hpp"

class PIDController
{
private:
    double err;
    double lastErr;
    double errSum;

    double kP, kI, kD,kF;
    double (*getSysTime)();
    double lastTs;

    double setpoint;
    double threshold;
    double output;
    bool isContinuous;
    double minInput;
    double maxInput;

public:
    PIDController(double (*getSysTime)());
    void setGains(double kP, double kI, double kD);
    double calculate(double measurement);
    double calculate(double setpoint, double measurement);
    void setSetpoint(double setpoint);
    void reset();
    bool atSetpoint();
    double getOutput();
    void setContinuous(double minInput, double maxInput);
    double getError();
};

