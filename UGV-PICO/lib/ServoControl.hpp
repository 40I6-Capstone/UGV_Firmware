#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>
// #include <cmath>

// #define PIN_SERVO 22

#define CLOSED_POSITION 
#define OPEN_POSITION 
#define TICKS_PER_DEG 5
#define MIN_TICKS 1000

class ServoControl
{
private:
    uint pin;
    // Wrap level, constant indicates with single pulse ends
    uint16_t wrapLevel = 20000u;
    // Compare level
    // uint16_t compareLevel = 100; // Should be minimum pulse high width
    // 1-2 ms = 124378 to 248756 ticks
    // each degree is 690 ticks for a range of 0 to 180
    uint16_t convertDeg(uint deg);
    void write(uint16_t level);
    pwm_config cfg;

public:
    void writeDeg(uint deg);
    ServoControl(uint pin);
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
    
    writeDeg(0);
}

// take in degrees and map to ticks
uint16_t ServoControl::convertDeg(uint deg)
{
    // each degree is 690 ticks for a range of 0 to 180
    uint16_t ticks = TICKS_PER_DEG * deg + MIN_TICKS;
    return ticks;
}

void ServoControl::writeDeg(uint deg)
{
    write(convertDeg(deg));
}

// From 0-1000
void ServoControl::write(uint16_t level)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, this->wrapLevel);
    pwm_set_gpio_level(pin, level);
    pwm_set_enabled(slice, true);
}