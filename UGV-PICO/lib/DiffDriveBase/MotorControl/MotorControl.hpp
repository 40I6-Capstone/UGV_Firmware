#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>
// #include <cmath>

#define MOTOR_DEADBAND 0.05

class MotorControl
{
private:
    uint pinA, pinB;
    double reverse;
    uint16_t wrapLevel = 1000u;
    void writePin(uint pin, uint16_t level);

public:
    MotorControl(uint pinA, uint pinB);
    void set(double output);
    void setReverse(bool);
};
