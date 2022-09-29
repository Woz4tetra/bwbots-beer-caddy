#include <BwDriveTrain.h>

BwDriveTrain::BwDriveTrain(
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926** motors,
        Encoder** encoders,
        unsigned int num_motors,
        unsigned int motor_enable_pin,
        double output_ratio,
        double armature_length,
        double min_radius_of_curvature,
        double* x_locations,
        double* y_locations
    )
{
    this->num_motors = num_motors;
    this->motor_enable_pin = motor_enable_pin;
    this->servos = servos;
    this->armature_length = armature_length;
    this->min_radius_of_curvature = min_radius_of_curvature;
    is_enabled = false;
    min_strafe_angle = -M_PI;
    max_strafe_angle = M_PI;
    reverse_min_strafe_angle = -M_PI;
    reverse_max_strafe_angle = M_PI;

    if (this->num_motors > BwDriveTrain::MAX_CHANNELS) {
        this->num_motors = BwDriveTrain::MAX_CHANNELS;
    }
    drive_modules = new BwDriveModule*[get_num_motors()];
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        double x_location = x_locations[channel];
        double y_location = y_locations[channel];
        drive_modules[channel] = new BwDriveModule(
            channel, output_ratio, x_location, y_location, min_radius_of_curvature, armature_length,
            servos, motors[channel], encoders[channel]
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
    double min_angle = min(abs(wrap_angle(servo_min_angle)), abs(wrap_angle(servo_max_angle)));
    if (min_angle < max_strafe_angle) {
        max_strafe_angle = min_angle;
        min_strafe_angle = -min_angle;
        reverse_max_strafe_angle = M_PI - max_strafe_angle;
        reverse_min_strafe_angle = -M_PI - min_strafe_angle;
    }
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_strafe_limits(min_strafe_angle, max_strafe_angle);
    }
}

void BwDriveTrain::set(unsigned int channel, double azimuth, double wheel_velocity)
{
    if (channel <= get_num_motors()) {
        drive_modules[channel]->set_azimuth(azimuth);
        drive_modules[channel]->set_wheel_velocity(wheel_velocity);
    }
}


void BwDriveTrain::drive(double vx, double vy, double vt)
{
    double delta_time = dt();
    double v_theta = atan2(vy, vx);

    if ((v_theta > max_strafe_angle && v_theta < reverse_max_strafe_angle) || (v_theta < min_strafe_angle && v_theta > reverse_min_strafe_angle)) {
        double v_mag = sqrt(vx * vx + vy * vy);
        if (0.0 <= v_theta && v_theta < M_PI / 2.0) {
            v_theta = max_strafe_angle;
        }
        else if (M_PI / 2.0 <= v_theta && v_theta <= M_PI) {
            v_theta = reverse_max_strafe_angle;
        }
        else if (-M_PI / 2.0 <= v_theta && v_theta < 0.0) {
            v_theta = min_strafe_angle;
        }
        else if (-M_PI <= v_theta && v_theta < -M_PI / 2.0) {
            v_theta = reverse_min_strafe_angle;
        }
        
        vx = v_mag * cos(v_theta);
        vy = v_mag * sin(v_theta);
    }
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set(vx, vy, vt, delta_time);
    }
}

void BwDriveTrain::stop()
{
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_wheel_velocity(0.0);
    }
}

double BwDriveTrain::wrap_angle(double angle)
{
    // wrap to -pi..pi
    angle = fmod(angle, 2.0 * M_PI);
    if (angle >= M_PI) {
        angle -= 2.0 * M_PI;
    }
    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
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
