#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>
// #include <cmath>

#define PIN_SERVO 22

#define CLOSED_POSITION 100
#define OPEN_POSITION 800


class ServoControl
{
private:
    uint pin;
    uint16_t wrapLevel = 1000u;

public:
    ServoControl(uint pin);
    void write(uint16_t level);
};

ServoControl::ServoControl(uint pin)
{
    // Config private members
    this->pin = pin;

    // Config pins
    gpio_set_function(this->pin, GPIO_FUNC_PWM);
    write(OPEN_POSITION);
}


void ServoControl::write(uint16_t level)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, this->wrapLevel);
    pwm_set_gpio_level(pin, level);
    pwm_set_enabled(slice, true);
}