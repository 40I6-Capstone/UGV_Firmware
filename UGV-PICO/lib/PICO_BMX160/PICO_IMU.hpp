#include "PICO_DFRobot_BMX160.h"
#include "math.h"

class PICO_IMU
{
private:
    double staticDeadband;
    volatile double angle;
    DFRobot_BMX160 *imu;
    struct repeating_timer updateTimer;
    bool inverted;
    uint64_t lastTimeStamp;

public:
    PICO_IMU(i2c_inst_t *i2c, uint sda, uint scl);
    bool begin();
    double getAngle();
    void update();
    void setDeadband(double);
    void setInverted(bool);
};

PICO_IMU::PICO_IMU(i2c_inst_t *i2c, uint sda, uint scl)
{
    this->angle = 0;
    this->staticDeadband = 0.124;
    this->imu = new DFRobot_BMX160(i2c, sda, scl);
    this->inverted = false;
}

/**
 * @brief Starts the IMU
 *
 * @return true startup success
 * @return false startup failure
 */
bool PICO_IMU::begin()
{
    this->lastTimeStamp = time_us_64();
    imu->setGyroRange(eGyroRange_2000DPS);
    return this->imu->begin() && add_repeating_timer_ms(
                                     -10,
                                     [](struct repeating_timer *t) -> bool
                                     {
                                         PICO_IMU *imu = (PICO_IMU *)(t->user_data);
                                         imu->update();
                                         return true;
                                     },
                                     this,
                                     &(this->updateTimer));
    ;
}

double PICO_IMU::getAngle()
{
    return this->angle;
}

void PICO_IMU::setInverted(bool isInverted)
{
    this->inverted = isInverted;
}

/**
 * @brief update the minimum threshold that the gyro needs to report to increment angle
 *
 * @param deadband deg/s
 */
void PICO_IMU::setDeadband(double deadband)
{
    this->staticDeadband = deadband;
}

void PICO_IMU::update()
{
    sBmx160SensorData_t gyroData;
    this->imu->getAllData(NULL, &gyroData, NULL);
    // std::cout << gyroData.z << std::endl;

    if (std::abs(gyroData.z) > this->staticDeadband)
    {
        double dt = double(time_us_64() - this->lastTimeStamp) / 1E6;     // us to seconds
        double dTheta = ((gyroData.z-this->staticDeadband) * (this->inverted ? -1. : 1.)); // gradian/s to deg/s
        this->angle = (this->angle + dTheta * dt);
        this->lastTimeStamp = time_us_64();
    }
}
