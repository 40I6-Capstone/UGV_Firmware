#include "PICO_DFRobot_BMX160.h"
#include <cmath>
#include "../MemoryFilter/MedianFilt/MedianFilt.hpp"
#include "../MemoryFilter/FIRFilt/FIRFilt.hpp"
#include "../../GeometryUtils/GeometryUtils.hpp"

#define FILT_SIZE 5
#define FILTB_SIZE 1

class PICO_IMU
{
private:
    double staticDeadband;
    double driftOffset;
    volatile double angle;
    DFRobot_BMX160 *imu;
    struct repeating_timer updateTimer;
    bool inverted;
    uint64_t lastTimeStamp;
    MedianFilt<double, FILT_SIZE> *filt;
    FIRFilt<double, FILTB_SIZE> *filtB;
    double coeffs[FILTB_SIZE] = { 1.0/(double)FILTB_SIZE};

public:
    PICO_IMU(i2c_inst_t *i2c, uint sda, uint scl);
    bool begin();
    double getAngle();
    void update();
    void setDeadband(double);
    void setInverted(bool);
};

