#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <math.h>
// #include <cmath>

// #define PIN_SERVO 22

#define CLOSED_POSITION 124378
#define OPEN_POSITION 248756
#define TICKS_PER_DEG 690

class ServoControl
{
private:
    uint pin;
    // Wrap level, constant indicates with single pulse ends
    uint16_t wrapLevel = 2487562u;
    // Compare level
    // uint16_t compareLevel = 100; // Should be minimum pulse high width
    // 1-2 ms = 124378 to 248756 ticks
    // each degree is 690 ticks for a range of 0 to 180
    void convertDeg(uint deg);
    void write(uint16_t level);

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
    writeDeg(0);
}

// take in degrees and map to ticks
void ServoControl::convertDeg(uint deg)
{
    // each degree is 690 ticks for a range of 0 to 180
    uint16_t ticks = TICKS_PER_DEG * deg + CLOSED_POSITION;
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