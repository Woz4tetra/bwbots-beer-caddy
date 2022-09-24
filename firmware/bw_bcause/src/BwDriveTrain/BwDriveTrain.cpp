#include <BwDriveTrain/BwDriveTrain.h>

BwDriveTrain::BwDriveTrain(
    Adafruit_PWMServoDriver* servos,
    MotorControllerMC33926** motors,
    Encoder** encoders,
    unsigned int num_motors,
    unsigned int motor_enable_pin)
{
    this->servos = servos;
    this->motors = motors;
    this->encoders = encoders;
    this->num_motors = num_motors;
    this->motor_enable_pin = motor_enable_pin;

    if (this->num_motors > BwDriveTrain::MAX_CHANNELS) {
        this->num_motors = BwDriveTrain::MAX_CHANNELS;
    }
}

unsigned int BwDriveTrain::get_num_motors() {
    return num_motors;
}

void BwDriveTrain::begin()
{
    for (signed int index = 0; index < get_num_motors(); index++) {
        motors[index]->begin();
        encoders->write(0);
    }
}


void BwDriveTrain::drive(float vx, float vy, float vt)
{
    
}
