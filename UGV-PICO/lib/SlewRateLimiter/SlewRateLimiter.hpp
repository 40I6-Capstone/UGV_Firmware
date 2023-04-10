#pragma once
#include <stdlib.h>
#include <math.h>

class SlewRateLimiter{
    
    public:
        SlewRateLimiter(double, double (*getSysTime)());
        double calculate(double);
    private: 
        double prevVal;
        double prevTime;
        double rateLimit;
        double (*getSysTime)();
        double clamp(double, double, double);
};




