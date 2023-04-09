#include "DifferentialDrive.hpp"

DifferentialDrive::DifferentialDrive(double(*getSysTime)())
{
    this->motor_right = new MotorControl(PIN_MOTOR_RA, PIN_MOTOR_RB);
    this->motor_left = new MotorControl(PIN_MOTOR_LA, PIN_MOTOR_LB);
    motor_right->setReverse(false);
    motor_left->setReverse(false);

    this->enc_right = new QuadEncoder(PIN_ENC_RA, PIN_ENC_RB);
    this->enc_left = new QuadEncoder(PIN_ENC_LA, PIN_ENC_LB);
    enc_right->setInverted(false);
    enc_left->setInverted(false);
    enc_right->setConversionFactor(M_PER_REV);
    enc_left->setConversionFactor(M_PER_REV);


    anglePID = new PIDController(getSysTime);
    anglePID->setGains(TURN_KP,TURN_KI,TURN_KD);
    anglePID->setContinuous(-180.,180.);

    leftPID = new PIDController(getSysTime);
    leftPID->setGains(LEFT_KP,LEFT_KI,LEFT_KD);

    rightPID = new PIDController(getSysTime);
    rightPID->setGains(RIGHT_KP,RIGHT_KI,RIGHT_KD);


    this->imu = new PICO_IMU(I2C_INST, PIN_SDA, PIN_SCL);
    bool imuSetupSuccess;
    imuSetupSuccess = imu->begin();
    std::cout << (imuSetupSuccess ? "IMU Setup Success" : "IMU Setup Fail") << std::endl;
    sleep_ms(100);
    imu->update();
    std::cout << "IMU updated" << std::endl;
    // odom = new DifferentialDriveOdometry(DEG_TO_RAD(imu->getAngle()));
    odom = new DifferentialDriveOdometry(0);
    std::cout << "Odom Setup" << std::endl;

}



void DifferentialDrive::setLeft(double output){
    this->motor_left->set(output);
}


void DifferentialDrive::setRight(double output){
    this->motor_right->set(output);
}


void DifferentialDrive::update()
{
    odom->update(enc_right->getPosition(), enc_left->getPosition(), GeometryUtils::degToRad(imu->getAngle()));
    // std::cout << "Gyro: " << imu->getAngle() << std::endl;
    // std::cout << "EncR: " << enc_right->getPosition() << " EncL: " << enc_left->getPosition() << std::endl;
    // Pose currentPose = odom->getCurrentPose();
    // std::cout << "Pose: " << currentPose.x << " " << currentPose.y << " " << RAD_TO_DEG(currentPose.theta) << " " << std::endl;
}

GeometryUtils::Pose DifferentialDrive::getPose(){
    return this->odom->getCurrentPose();
}

// Returns angle in degs
double DifferentialDrive::getAngle(){
    return this->imu->getAngle();
}


double DifferentialDrive::getDistLeft(){
    return this->enc_left->getPosition();
}

double DifferentialDrive::getDistRight(){
    return this->enc_right->getPosition();
}

double DifferentialDrive::getVRight(){
    return this->enc_right->getVelocity();
}

double DifferentialDrive::getVLeft(){
    return this->enc_left->getVelocity();
}

void DifferentialDrive::setLeftV(double setpoint){
    double output = this->leftPID->calculate(setpoint, this->enc_left->getVelocity());
    setLeft(output+ getFF(setpoint));
}


void DifferentialDrive::setRightV(double setpoint){
    double output = this->rightPID->calculate(setpoint, this->enc_right->getVelocity());
    setRight(output + getFF(setpoint));
}


void DifferentialDrive::setLeftGains(double kP, double kI, double kD){
    this->leftPID->setGains(kP,kI,kD);
}

void DifferentialDrive::setRightGains(double kP, double kI, double kD){
    this->rightPID->setGains(kP,kI,kD);
}


void DifferentialDrive::setDriveState(double heading, double v){
    double bias = this->anglePID->calculate(heading,this->getAngle());

    setRightV(v + bias);
    setLeftV(v - bias);
}


double DifferentialDrive::getFF(double setpoint){
    // return exp(5.729*setpoint)*0.0353;
    return 3.6526*std::pow(setpoint,3) + 1.0577*std::pow(setpoint,2) - 0.1532*setpoint + 0.0548;
}


void DifferentialDrive::updateTicksRight(){
    this->enc_right->updateTicks();
}

void DifferentialDrive::updateTicksLeft(){
    this->enc_left->updateTicks();
}

void DifferentialDrive::resetControllers(){
    this->anglePID->reset();
    this->leftPID->reset();
    this->rightPID->reset();
}

void DifferentialDrive::stop(){
    this->setRight(0.);
    this->setLeft(0.);
}


double DifferentialDrive::distanceToPoint(GeometryUtils::Pose to){
    return GeometryUtils::distToPoint(this->getPose(),to);
}


void DifferentialDrive::setPose(GeometryUtils::Pose newPose){
    this->odom->setPose(newPose, imu->getAngle());
}