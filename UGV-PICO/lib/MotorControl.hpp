#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>

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

MotorControl::MotorControl(uint pinA, uint pinB)
{
    // Config private members
    this->pinA = pinA;
    this->pinB = pinB;
    this->reverse = 1.0;

    // Config pins
    gpio_set_function(this->pinA, GPIO_FUNC_PWM);
    writePin(this->pinA, 0);

    gpio_set_function(this->pinB, GPIO_FUNC_PWM);
    writePin(this->pinB, 0);
}

/**
 * @brief set the motor to be reversed
 *
 * @param isReversed
 */
void MotorControl::setReverse(bool isReversed)
{
    this->reverse = isReversed ? -1. : 1.;
}

/**
 * @brief set the motor at an output
 *
 * @param output output number -1.0 to 1.0
 */
void MotorControl::set(double output)
{
    double out;
    // clamp output
    if (output > 1.0)
        out = 1.0;
    else if (output < -1.0)
        out = -1.0;
    else
        out = output;

    uint16_t counterCompare = std::abs(out) * (double)wrapLevel;

    bool reverseOutput = (this->reverse * out) > 0;

    if (output == 0.0)
    {
        writePin(pinA, 0);
        writePin(pinB, 0);
    }
    else if (!reverseOutput)
    {
        writePin(pinA, counterCompare);
        writePin(pinB, 0);
    }
    else
    {
        writePin(pinA, 0);
        writePin(pinB, counterCompare);
    }
}

void MotorControl::writePin(uint pin, uint16_t level)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, this->wrapLevel);
    pwm_set_gpio_level(pin, level);
    pwm_set_enabled(slice, true);
}