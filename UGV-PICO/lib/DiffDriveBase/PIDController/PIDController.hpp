#include <stdlib.h>

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

public:
    PIDController(double (*getSysTime)());
    void setGains(double kP, double kI, double kD);
    double calculate(double measurement);
    double calculate(double setpoint, double measurement);
    void setSetpoint(double setpoint);
    void reset();
    bool atSetpoint();
    double getOutput();
};

