#include <BwDriveTrain.h>

BwDriveTrain::BwDriveTrain(
    Adafruit_PWMServoDriver* servos,
    MotorControllerMC33926** motors,
    Encoder** encoders,
    unsigned int num_motors,
    unsigned int motor_enable_pin,
    double output_ratio)
{
    this->num_motors = num_motors;
    this->motor_enable_pin = motor_enable_pin;
    is_enabled = false;

    if (this->num_motors > BwDriveTrain::MAX_CHANNELS) {
        this->num_motors = BwDriveTrain::MAX_CHANNELS;
    }
    drive_modules = new BwDriveModule*[get_num_motors()];
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel] = new BwDriveModule(
            channel, output_ratio, servos, motors[channel], encoders[channel]
        );
    }
}

unsigned int BwDriveTrain::get_num_motors() {
    return num_motors;
}

void BwDriveTrain::begin()
{
    set_enable(false);
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->begin();
    }
}

void BwDriveTrain::set_enable(bool state) {
    is_enabled = state;
    digitalWrite(motor_enable_pin, state);
    if (is_enabled) {
        reset();
    }
}

void BwDriveTrain::reset()
{
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->reset();
    }
}


bool BwDriveTrain::get_enable() {
    return is_enabled;
}

void BwDriveTrain::set_limits(
    unsigned int channel,
    double servo_min_angle,
    double servo_max_angle,
    int servo_min_command,
    int servo_max_command,
    double servo_max_velocity)
{
    if (channel > get_num_motors()) {
        return;
    }
    drive_modules[channel]->set_limits(
        servo_min_angle,
        servo_max_angle,
        servo_min_command,
        servo_max_command,
        servo_max_velocity
    );
}


void BwDriveTrain::drive(float vx, float vy, float vt)
{
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        // drive_modules[channel]->set_direction()
        // drive_modules[channel]->set_velocity()
    }
}

double BwDriveTrain::get_velocity(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_velocity();
    }
}

double BwDriveTrain::get_position(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_position();
    }
}

double BwDriveTrain::get_angle(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_angle();
    }
}

SpeedPID* BwDriveTrain::get_pid(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return NULL;
    }
    else {
        return drive_modules[channel]->get_pid();
    }
}

SpeedFilter* BwDriveTrain::get_filter(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return NULL;
    }
    else {
        return drive_modules[channel]->get_filter();
    }
}
