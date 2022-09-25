#include <BwDriveTrain/BwDriveTrain.h>

BwDriveTrain::BwDriveTrain(
    Adafruit_PWMServoDriver* servos,
    MotorControllerMC33926** motors,
    Encoder** encoders,
    unsigned int num_motors,
    unsigned int motor_enable_pin,
    double output_ratio)
{
    this->servos = servos;
    this->motors = motors;
    this->encoders = encoders;
    this->num_motors = num_motors;
    this->motor_enable_pin = motor_enable_pin;
    this->output_ratio = output_ratio;
    is_enabled = false;

    if (this->num_motors > BwDriveTrain::MAX_CHANNELS) {
        this->num_motors = BwDriveTrain::MAX_CHANNELS;
    }
    speed_pids = new SpeedPID*[get_num_motors()];
    speed_filters = new SpeedFilter*[get_num_motors()];
    encoder_positions = new long[get_num_motors()];
    for (signed int index = 0; index < get_num_motors(); index++) {
        speed_pids[index] = new SpeedPID();
        speed_filters[index] = new SpeedFilter(1.0);
        encoder_positions[index] = 0;
    }
}

unsigned int BwDriveTrain::get_num_motors() {
    return num_motors;
}

void BwDriveTrain::begin()
{
    set_enable(false);
    for (signed int index = 0; index < get_num_motors(); index++) {
        motors[index]->begin();
        encoders[index]->write(0);
    }
}

void BwDriveTrain::set_enable(bool state) {
    is_enabled = state;
    digitalWrite(motor_enable_pin, state);
}


void BwDriveTrain::drive(float vx, float vy, float vt)
{
    // speed_pids[index]->set_target()

    for (signed int channel = 0; channel < get_num_motors(); channel++) {
        encoder_positions[channel] = encoders[channel]->read();
        double velocity = get_velocity(channel);
        int command = speed_pids[channel]->compute(velocity);
        motors[channel]->set(command);
    }
}

double BwDriveTrain::get_velocity(int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return speed_filters[channel]->compute(get_position(channel));
    }
}

double BwDriveTrain::get_position(int channel)
{
    return (double)(encoder_positions[channel]) * output_ratio;
}


SpeedPID* BwDriveTrain::get_pid(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return NULL;
    }
    else {
        return speed_pids[channel];
    }
}

SpeedFilter* BwDriveTrain::get_filter(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return NULL;
    }
    else {
        return speed_filters[channel];
    }
}
