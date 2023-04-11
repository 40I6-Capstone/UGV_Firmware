#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>
// #include <cmath>

// #define PIN_SERVO 22

#define CLOSED_POSITION 1100
#define OPEN_POSITION 650
#define TICKS_PER_DEG 5
#define MIN_TICKS 650
#define MAX_TICKS 2450

class ServoControl
{
private:
    uint pin;
    // Wrap level, constant indicates with single pulse ends
    uint16_t wrapLevel = 20000u;
    // Compare level
    pwm_config cfg;

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
    uint slice = pwm_gpio_to_slice_num(pin);
    this->cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&(this->cfg),125);
    pwm_init(slice,&cfg,true);
}



// From 0-1000
void ServoControl::write(uint16_t level)
{
    if(level < MIN_TICKS){level = MIN_TICKS;}   
    if(level > MAX_TICKS){level = MAX_TICKS;}   
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, this->wrapLevel);
    pwm_set_gpio_level(pin, level);
    pwm_set_enabled(slice, true);
}