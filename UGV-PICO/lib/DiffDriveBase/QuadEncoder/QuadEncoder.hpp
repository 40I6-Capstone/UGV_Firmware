
#include <stdint.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <pico/time.h>

/**
 * @brief Pico-Specific Quadrature encoder class
 * @author Shaqeeb Momen
 *
 *
 * @ref http://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
 */

class QuadEncoder
{
private:
    uint pinA, pinB;
    volatile double position;
    volatile double velocity;
    const long lookup[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    volatile long encoderVal;
    volatile long tickCount;
    long inverted = 1l;
    double conversionFactor = 1.;
    double oldPosition;
    uint64_t oldTimestamp;

    void tickInterrupt();
    // uint8_t (*getPinStates)();
    // double (*getSysTimeSeconds)();

    double convertTicks(double);

    uint8_t getPinStates();
    void setupPins();

    struct repeating_timer vTimer;
    void updateVelocity();

public:
    QuadEncoder(uint pinA, uint pinB);
    void setInverted(bool);
    void setConversionFactor(double);
    double getPosition();
    double getVelocity();
    void setPosition(double);
    void updateTicks();
};

