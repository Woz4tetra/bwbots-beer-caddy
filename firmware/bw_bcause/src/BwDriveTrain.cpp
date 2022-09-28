#include <BwDriveTrain.h>

BwDriveTrain::BwDriveTrain(
    Adafruit_PWMServoDriver* servos,
    MotorControllerMC33926** motors,
    Encoder** encoders,
    unsigned int num_motors,
    unsigned int motor_enable_pin,
    double output_ratio,
    double width, double length,
    double armature_length)
{
    this->num_motors = num_motors;
    this->motor_enable_pin = motor_enable_pin;
    this->servos = servos;
    this->width = width;
    this->length = length;
    this->armature_length = armature_length;
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
    is_enabled = true;
    set_enable(false);
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->begin();
    }
}

void BwDriveTrain::set_enable(bool state) {
    if (is_enabled == state) {
        return;
    }
    is_enabled = state;
    digitalWrite(motor_enable_pin, state);
    if (is_enabled) {
        reset();
    }
    else {
        for (unsigned int channel = 0; channel < MAX_CHANNELS; channel++) {
            servos->setPWM(channel, 0, 4096);
        }
    }

    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_enable(is_enabled);
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
    double servo_angle_1,
    double servo_angle_2,
    int servo_command_1,
    int servo_command_2,
    double servo_max_velocity,
    bool flip_motor_commands)
{
    if (channel > get_num_motors()) {
        return;
    }
    drive_modules[channel]->set_limits(
        servo_min_angle,
        servo_max_angle,
        servo_angle_1,
        servo_angle_2,
        servo_command_1,
        servo_command_2,
        servo_max_velocity,
        flip_motor_commands
    );
}

void BwDriveTrain::set(unsigned int channel, double azimuth, double wheel_velocity)
{
    if (channel <= get_num_motors()) {
        drive_modules[channel]->set(azimuth, wheel_velocity);
    }
}


void BwDriveTrain::drive(double vx, double vy, double vt)
{
    // Assumes a wheel module at each corner!!
    // Every other part of this class doesn't make this assumption
    // Assumes channel 2 is front left (+X, +Y)
    // Assumes channel 0 is back left (-X, +Y)
    // Assumes channel 1 is back right (-X, -Y)
    // Assumes channel 3 is front right (+X, -Y)
    if (get_num_motors() != 4) {
        return;
    }
    double delta_time = dt();
    double azimuth, wheel_velocity;
    // left states
    compute_module_state(length / 2.0, width / 2.0, vx, vy, vt, delta_time, azimuth, wheel_velocity);
    set(2, azimuth, wheel_velocity);  // front left
    set(0, -azimuth, wheel_velocity);  // back left
    // right states
    compute_module_state(length / 2.0, -width / 2.0, vx, vy, vt, delta_time, azimuth, wheel_velocity);
    set(1, -azimuth, wheel_velocity);  // back right
    set(3, azimuth, wheel_velocity);  // front right
}

void BwDriveTrain::stop()
{
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_wheel_velocity(0.0);
    }
}


void BwDriveTrain::compute_module_state(double x, double y, double vx, double vy, double vt, double dt, double& azimuth, double& wheel_velocity)
{
    double theta_mag = vt * dt;
    double module_vx, module_vy;
    if (theta_mag == 0.0) {
        module_vx = vx;
        module_vy = vy;
    }
    else {
        double v_mag = sqrt(vx * vx + vy * vy);
        double d_mag = v_mag * dt;
        double radius_of_curvature = d_mag / tan(theta_mag);
        if (radius_of_curvature != radius_of_curvature || isinf(radius_of_curvature)) {
            module_vx = vx + vt * -y;
            module_vy = vy + vt * x;
        }
        else if (abs(radius_of_curvature) < 0.1) {
            module_vx = vt * -y;
            module_vy = vt * x;
        }
        else {
            v_mag = vx;
            double module_angle = atan2(x, radius_of_curvature + y);
            double module_radc = x / sin(module_angle) - armature_length;

            module_vx = v_mag * module_radc / radius_of_curvature * cos(module_angle);
            module_vy = v_mag * module_radc / radius_of_curvature * sin(module_angle);
        }
    }
    azimuth = atan2(module_vy, module_vx);
    wheel_velocity = sqrt(module_vx * module_vx + module_vy * module_vy);
}

double BwDriveTrain::dt()
{
    uint32_t current_time = micros();
    uint32_t delta_time = current_time - prev_time;
    prev_time = current_time;
    return (double)delta_time * 1E-6;
}


double BwDriveTrain::get_wheel_velocity(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_wheel_velocity();
    }
}

double BwDriveTrain::get_wheel_position(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_wheel_position();
    }
}

double BwDriveTrain::get_azimuth(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_azimuth();
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
