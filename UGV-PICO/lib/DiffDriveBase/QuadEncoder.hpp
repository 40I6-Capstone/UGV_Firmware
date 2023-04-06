
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

/**
 * @brief Construct a new Encoder:: Encoder object
 *
 * @param pinA
 * @param pinB
 * @param encoderISR pointer to function to be called on on interrupt. Make a function and call this->updateTicks inside it
 */
QuadEncoder::QuadEncoder(uint pinA, uint pinB)
{
    this->pinA = pinA;
    this->pinB = pinB;
    setupPins();
    // this->getPinStates = getPinStates;
    // this->getSysTimeSeconds = getSysTimeSeconds;
    this->tickCount = 0;
    this->encoderVal = 0;
    this->position = 0;
    this->oldPosition = 0;
    this->oldTimestamp = time_us_64();
    add_repeating_timer_ms(
        -100, [](struct repeating_timer *t) -> bool
        {
            QuadEncoder *enc = (QuadEncoder*)(t->user_data);
            enc->updateVelocity();
            return true; },
        this, &(this->vTimer));
}

/**
 * @brief set the encoder inverted
 *
 * @param isInverted
 */
void QuadEncoder::setInverted(bool isInverted)
{
    if (isInverted)
        inverted = -1L;
    else
        inverted = 1l;
}

/**
 * @brief set the internal factor to convert tick values to position or velocity
 *
 * @param factor conversion factor in units/tick
 */
void QuadEncoder::setConversionFactor(double factor)
{
    this->conversionFactor = factor;
}

/**
 * @brief set position of encoder
 *
 * @param pos
 */
void QuadEncoder::setPosition(double pos)
{
    this->position = pos;
}

/**
 * @brief Get position of encoder
 *
 * @return double position of encoder in converted units per second
 */
double QuadEncoder::getPosition()
{
    return this->convertTicks(this->position);
}

/**
 * @brief Get velocity of encoder
 *
 * @return double velocity of encoder in converted units per second
 */
double QuadEncoder::getVelocity()
{
    return this->convertTicks(this->velocity);
}

double QuadEncoder::convertTicks(double raw)
{
    return raw * this->conversionFactor;
}

void QuadEncoder::updateTicks()
{
    // Shift old tick state to the left
    this->encoderVal = this->encoderVal << 2;
    // Place new pin state to the right side
    this->encoderVal = this->encoderVal | (this->getPinStates());

    // Update tick count based on previous two encoder pin states
    this->tickCount += this->lookup[this->encoderVal & 0b1111] * this->inverted;

    // Upate local members
    double newPosition = (double)this->tickCount;

    this->position = newPosition;
}

void QuadEncoder::setupPins()
{
    gpio_init(this->pinA);
    gpio_init(this->pinB);
    gpio_set_dir(this->pinA, GPIO_IN);
    gpio_set_dir(this->pinB, GPIO_IN);
    // gpio_set_irq_enabled_with_callback(this->pinA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, this->encoderISR);
    gpio_set_irq_enabled(this->pinA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(this->pinB, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
}

uint8_t QuadEncoder::getPinStates()
{
    uint8_t pinStates = 0;
    pinStates = gpio_get(this->pinA) << 1 | gpio_get(this->pinB);
    return pinStates;
}

void QuadEncoder::updateVelocity()
{
    double dP = this->position -this->oldPosition; // Ticks
    double dt = double(time_us_64()-oldTimestamp) /1E6; // S

    this->velocity = dP/dt;
    this->oldPosition = this->position;
    this->oldTimestamp = time_us_64();
}