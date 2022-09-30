#include <BwDriveModule.h>

BwDriveModule::BwDriveModule(
        int channel,
        double output_ratio,
        double x_location,
        double y_location,
        double min_radius_of_curvature,
        double armature_length,
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926* motor,
        Encoder* encoder
    )
{
    this->channel = channel;
    this->output_ratio = output_ratio;
    this->servos = servos;
    this->motor = motor;
    this->encoder = encoder;
    this->x_location = x_location;
    this->y_location = y_location;
    this->min_radius_of_curvature = min_radius_of_curvature;
    this->armature_length = armature_length;
    encoder_position = 0;
    speed_pid = new SpeedPID();
    speed_filter = new SpeedFilter(1.0);
    servo_min_angle = 0.0;
    servo_max_angle = 90.0;
    servo_angle_1 = 0.0;
    servo_angle_2 = 90.0;
    servo_command_1 = 450;   // semi neural default
    servo_command_2 = 300;   // semi neural default
    setpoint_angle = 0.0;
    predicted_angle = 0.0;
    is_enabled = false;
    flip_motor_commands = false;
    min_strafe_angle = 0.0;
    max_strafe_angle = 0.0;
}

void BwDriveModule::begin()
{
    motor->begin();
    encoder->write(0);
}


void BwDriveModule::reset()
{
    speed_pid->reset();
    speed_filter->reset();
    motor->set(0);
}

void BwDriveModule::set_azimuth(double setpoint)
{
    if (setpoint < servo_min_angle) {
        setpoint = servo_min_angle;
    }
    if (setpoint > servo_max_angle) {
        setpoint = servo_max_angle;
    }
    
    setpoint_angle = setpoint;
    int pulse = (int)(
        (servo_command_2 - servo_command_1) / 
        (servo_angle_2 - servo_angle_1) * 
        (setpoint - servo_angle_1) + 
        servo_command_1
    );

    if (!is_enabled) {
        return;
    }
    servos->setPWM(channel, 0, pulse);
    update_predicted_azimuth();
}

void BwDriveModule::set_enable(bool state) {
    is_enabled = state;
}


void BwDriveModule::set_limits(
    double servo_min_angle,
    double servo_max_angle,
    double servo_angle_1,
    double servo_angle_2,
    int servo_command_1,
    int servo_command_2,
    double servo_max_velocity,
    bool flip_motor_commands)
{
    this->servo_min_angle = servo_min_angle;
    this->servo_max_angle = servo_max_angle;
    this->servo_angle_1 = servo_angle_1;
    this->servo_angle_2 = servo_angle_2;
    this->servo_command_1 = servo_command_1;
    this->servo_command_2 = servo_command_2;
    this->servo_max_velocity = servo_max_velocity;
    this->flip_motor_commands = flip_motor_commands;
}

void BwDriveModule::set_strafe_limits(double min_strafe_angle, double max_strafe_angle)
{
    this->min_strafe_angle = min_strafe_angle;
    this->max_strafe_angle = max_strafe_angle;
}

double BwDriveModule::get_azimuth() {
    return predicted_angle;
}

double BwDriveModule::get_wheel_velocity() {
    return speed_filter->get_velocity();
}

double BwDriveModule::update_wheel_velocity() {
    return speed_filter->compute(get_wheel_position());
}

double BwDriveModule::get_wheel_position() {
    return (double)(encoder_position) * output_ratio;
}

void BwDriveModule::update_predicted_azimuth() {
    predicted_angle = setpoint_angle;
    // double error = setpoint_angle - predicted_angle;
    // if (abs(error) < 0.1) { // TODO: make this configurable
    //     predicted_angle = setpoint_angle;
    // }
    // double delta_time = dt();
    // double servo_delta = error * delta_time;  // TODO: add ramping constant
    // if (abs(servo_delta) > servo_max_velocity) {
    //     servo_delta = SpeedPID::sign(error) * servo_max_velocity * delta_time;
    // }
    // predicted_angle += servo_delta;
}

void BwDriveModule::set_wheel_velocity(double velocity)
{
    speed_pid->set_target(velocity);
    encoder_position = encoder->read();
    if (flip_motor_commands) {
        encoder_position = -encoder_position;
    }
    double measured_velocity = update_wheel_velocity();
    int command = speed_pid->compute(measured_velocity);
    if (!is_enabled) {
        return;
    }
    if (flip_motor_commands) {
        command = -command;
    }
    motor->set(command);
}

double BwDriveModule::wrap_angle(double angle)
{
    // wrap to -pi..pi for front modules, 0..2pi for back modules
    angle = fmod(angle, 2.0 * M_PI);
    if (servo_max_angle < M_PI) {
        if (angle >= M_PI) {
            angle -= 2.0 * M_PI;
        }
        if (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
    }
    else {
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
    }
    return angle;
}

void BwDriveModule::compute_state(double vx, double vy, double vt, double dt, double& azimuth, double& wheel_velocity)
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
            module_vx = vx + vt * -y_location;
            module_vy = vy + vt * x_location;
        }
        else if (abs(radius_of_curvature) < min_radius_of_curvature) {
            module_vx = vt * -y_location;
            module_vy = vt * x_location;
        }
        else {
            double module_angle = atan2(x_location, radius_of_curvature + y_location);
            double module_radc = x_location / sin(module_angle) - armature_length;

            module_vx = vx * module_radc / radius_of_curvature * cos(module_angle);
            module_vy = vy + vx * module_radc / radius_of_curvature * sin(module_angle);
        }
    }
    azimuth = atan2(module_vy, module_vx);
    wheel_velocity = sqrt(module_vx * module_vx + module_vy * module_vy);
}

void BwDriveModule::set(double vx, double vy, double vt, double dt)
{
    double azimuth, wheel_velocity;
    compute_state(vx, vy, vt, dt, azimuth, wheel_velocity);

    azimuth = wrap_angle(azimuth);
    if (azimuth < servo_min_angle || azimuth > servo_max_angle) {
        azimuth = wrap_angle(azimuth + M_PI);
        wheel_velocity = -wheel_velocity;
    }

    set_azimuth(azimuth);
    set_wheel_velocity(wheel_velocity);
}


double BwDriveModule::dt()
{
    uint32_t delta_time = micros() - prev_time;
    return (double)delta_time * 1E-6;
}
