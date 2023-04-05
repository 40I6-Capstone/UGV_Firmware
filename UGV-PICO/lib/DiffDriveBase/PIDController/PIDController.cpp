#include "PIDController.hpp"

PIDController::PIDController(double (*getSysTime)())
{
    this->err = 0;
    this->lastErr = 0;
    this->errSum = 0;
    this->kP = 0;
    this->kI = 0;
    this->kD = 0;
    this->threshold = 0;
    this->output = 0;
    this->setpoint = 0;
    this->getSysTime = getSysTime;
    this->lastTs = this->getSysTime();
    this->reset();
}

void PIDController::setGains(double kP, double kI, double kD)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

double PIDController::calculate(double measurement)
{
    return this->calculate(this->setpoint, measurement);
}

double PIDController::calculate(double setpoint, double measurement)
{
    this->err = setpoint - measurement;
    this->errSum += this->err;

    double P = this->kP * this->err;
    double I = this->kI * this->errSum;
    double dt = this->getSysTime() - this->lastTs;
    double D = this->kD * (this->err - this->lastErr) / dt;

    this->lastTs = this->getSysTime();
    this->lastErr = this->err;

    return P + I + D;
}

void PIDController::setSetpoint(double setpoint)
{
    this->setpoint = setpoint;
}

bool PIDController::atSetpoint()
{
    return std::abs(this->err) < this->threshold;
}

double PIDController::getOutput()
{
    return this->output;
}

void PIDController::reset()
{
    this->err = 0;
    this->errSum = 0;
    this->lastErr = 0;
    this->lastTs = getSysTime();
}