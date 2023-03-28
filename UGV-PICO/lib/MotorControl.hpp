#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>

class MotorControl
{
private:
    uint pinA, pinB;
    double reverse;
    uint16_t wrapLevel = 1000;
    void writePin(uint pin, uint16_t level);

public:
    MotorControl(uint pinA, uint pinB);
    void run(double output);
    void setReverse(bool);
};

MotorControl::MotorControl(uint pinA, uint pinB)
{
    // Config private members
    this->pinA = pinA;
    this->pinB = pinB;
    this->reverse = false;

    // Config pins
    gpio_set_function(pinA, GPIO_FUNC_PWM);
    writePin(pinA, 0);

    gpio_set_function(pinB, GPIO_FUNC_PWM);
    writePin(pinB, 0);
}

/// @brief set the motor to be reversed
/// @param isReversed
void MotorControl::setReverse(bool isReversed)
{
    this->reverse = isReversed ? -1 : 1;
}

/// @brief set the motor at an output
/// @param output output number -1.0 to 1.0
void MotorControl::run(double output)
{
    // double out;
    // // clamp output
    // if (output > 1.0)
    //     out = 1.0;
    // else if (output < -1.0)
    //     out = -1.0;
    // else
    //     out = output;

    // uint16_t top = std::abs(out) * (double)wrapLevel;

    // bool reverseOut = this->reverse != (output < 0);

    // if (output == 0.0)
    // {
    //     writePin(pinA, 0);
    //     writePin(pinB, 0);
    // }
    // else if (!reverseOut)
    // {
    //     writePin(pinA, top);
    //     writePin(pinB, 0);
    // }
    // else
    // {
    //     writePin(pinA, 0);
    //     writePin(pinB, top);
    // }
    writePin(pinA,1000);
    writePin(pinB,1000);
}

void MotorControl::writePin(uint pin, uint16_t level)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan = pwm_gpio_to_channel(pin);
    pwm_set_wrap(slice, wrapLevel);
    pwm_set_chan_level(slice, chan, level);
    pwm_set_enabled(slice, true);
}