#include "SlewRateLimiter.hpp"

SlewRateLimiter::SlewRateLimiter(double rateLimit, double (*getSysTime)()){
    this->prevVal = 0;
    this->getSysTime = getSysTime;
    this->prevTime = this->getSysTime();
    this->rateLimit = rateLimit;
}


double SlewRateLimiter::calculate(double input){
    double currentTime = getSysTime();
    double elapsedTime = currentTime - prevTime;

    prevVal += this->clamp(input-prevVal,-this->rateLimit*elapsedTime,this->rateLimit*elapsedTime);
    prevTime = currentTime;
    return prevVal;
}


double SlewRateLimiter::clamp(double input, double min, double max){
    if(input > max){return max;}
    if(input < min) {return min;}
    return input;
}