#include "PICO_IMU.hpp"

PICO_IMU::PICO_IMU(i2c_inst_t *i2c, uint sda, uint scl)
{
    this->angle = 0;
    this->staticDeadband = 0.01;//0.0609756;
    this->driftOffset = 0.0609756;
    this->imu = new DFRobot_BMX160(i2c, sda, scl);
    this->inverted = false;
    this->filt = new MedianFilt<double, FILT_SIZE>();
    this->filtB = new FIRFilt<double, FILTB_SIZE>(this->coeffs);
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
    bool gyroStarted = this->imu->begin();
    this->imu->setGyroRange(eGyroRange_2000DPS);
    // bool timerStarted = add_repeating_timer_ms(
    //                                  -10,
    //                                  [](struct repeating_timer *t) -> bool
    //                                  {
    //                                      PICO_IMU *imu = (PICO_IMU *)(t->user_data);
    //                                      imu->update();
    //                                      return true;
    //                                  },
    //                                  this,
    //                                  &(this->updateTimer));
    // return gyroStarted && timerStarted;
    return gyroStarted;
}

double PICO_IMU::getAngle()
{
    double output;
    output = GeometryUtils::inputModulus(this->angle,-180,180);
    return output;
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
    // std::cout << "Gyro:  " << gyroData.z << std::endl;
    this->filt->update(gyroData.z - (gyroData.z > 0 ? this->driftOffset : 0));
    double filteredOmega = this->filt->getOutput();
    // std::cout << "Filt:  " << filteredOmega << std::endl;
    if (std::abs(filteredOmega) > this->staticDeadband)
    {
        double dt = double(time_us_64() - this->lastTimeStamp) / 1E6;                           // us to seconds
        double omega = ((filteredOmega) * (this->inverted ? -1. : 1.)); 
        this->filtB->update(omega * dt);
        double dTheta = filtB->getOutput();
        this->angle = (this->angle + dTheta);
    }
    this->lastTimeStamp = time_us_64();
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}