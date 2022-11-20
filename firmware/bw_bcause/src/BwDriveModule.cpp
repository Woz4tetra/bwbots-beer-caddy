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
    wheel_radius = 1.0;
    servo_min_angle = 0.0;
    servo_max_angle = 90.0;
    servo_angle_1 = 0.0;
    servo_angle_2 = 90.0;
    servo_command_1 = 450;   // semi neural default
    servo_command_2 = 300;   // semi neural default
    setpoint_angle = 0.0;
    predicted_angle = 0.0;
    predicted_velocity = 0.0;
    is_enabled = false;
    flip_motor_commands = false;
    min_strafe_angle = 0.0;
    max_strafe_angle = 0.0;
    prev_time = 0;
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

void BwDriveModule::set_azimuth(double setpoint, double dt)
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
    update_predicted_azimuth(dt);
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
    double wheel_radius,
    bool flip_motor_commands)
{
    this->servo_min_angle = servo_min_angle;
    this->servo_max_angle = servo_max_angle;
    this->servo_angle_1 = servo_angle_1;
    this->servo_angle_2 = servo_angle_2;
    this->servo_command_1 = servo_command_1;
    this->servo_command_2 = servo_command_2;
    this->servo_max_velocity = servo_max_velocity;
    this->wheel_radius = wheel_radius;
    this->flip_motor_commands = flip_motor_commands;
    if (servo_min_angle < 0.0) {
        setpoint_angle = 0.0;
        predicted_angle = 0.0;
    }
    else {
        setpoint_angle = M_PI;
        predicted_angle = M_PI;
    }
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

void BwDriveModule::update_wheel_velocity() {
    speed_filter->compute(get_wheel_position());
}

void BwDriveModule::update_wheel_position() {
    encoder_position = encoder->read();
    if (flip_motor_commands) {
        encoder_position = -encoder_position;
    }
}

double BwDriveModule::get_wheel_position() {
    return (double)(encoder_position) * output_ratio;
}

void BwDriveModule::update_predicted_azimuth(double dt) {
    if (!is_enabled) {
        predicted_angle = setpoint_angle;
        predicted_velocity = 0.0;
        return;
    }
    double error = setpoint_angle - predicted_angle;
    if (abs(error) < 0.1) {
        predicted_angle = setpoint_angle;
    }

    predicted_velocity = 2.0 * error * servo_max_velocity;
    if (abs(predicted_velocity) > servo_max_velocity) {
        predicted_velocity = SpeedPID::sign(predicted_velocity) * servo_max_velocity;
    }
    predicted_angle += predicted_velocity * dt;
}

void BwDriveModule::set_wheel_velocity(double velocity)
{
    speed_pid->set_target(velocity);
    update_wheel_position();
    update_wheel_velocity();
    int command = speed_pid->compute(speed_filter->get_velocity());
    if (!is_enabled) {
        return;
    }
    if (flip_motor_commands) {
        command = -command;
    }
    command_wheel_pwm(command);
}

void BwDriveModule::command_wheel_pwm(int command) {
    motor->set(command);
}


double BwDriveModule::wrap_angle(double angle)
{
    // wrap to -pi..pi for front modules, 0..2pi for back modules
    angle = fmod(angle, 2.0 * M_PI);
    if (servo_min_angle < 0.0) {
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
    double v_mag = sqrt(vx * vx + vy * vy);
    double module_vx, module_vy;
    double radius_of_curvature;
    if (theta_mag == 0.0) {
        radius_of_curvature = 0.0;
    }
    else {
        radius_of_curvature = SpeedPID::sign(vx) * (v_mag * dt) / tan(theta_mag);
    }
    
    if (theta_mag == 0.0 || radius_of_curvature != radius_of_curvature || isinf(radius_of_curvature)) {
        // Strafe regime
        module_vx = vx;
        module_vy = vy;
        if (abs(v_mag) > 0.0) {
            // set azimuth if v_mag is > 0.0. Otherwise, use the previous angle
            azimuth = atan2(module_vy, module_vx);
        }
        wheel_velocity = v_mag;
    }
    else if (abs(radius_of_curvature) < min_radius_of_curvature) {
        // Rotate in place regime
        module_vx = vt * -y_location;
        module_vy = vt * x_location;
        azimuth = atan2(module_vy, module_vx);
        wheel_velocity = sqrt(module_vx * module_vx + module_vy * module_vy);
        if (servo_min_angle < 0.0) {
            azimuth += M_PI;
            wheel_velocity = -wheel_velocity;
        }
    }
    else {
        // Ackermann + strafe regime
        double module_angle = atan2(x_location, radius_of_curvature - y_location);
        double module_radc = x_location / sin(module_angle) - armature_length;

        module_vx = vx * module_radc / radius_of_curvature * cos(module_angle);
        module_vy = vy + vx * module_radc / radius_of_curvature * sin(module_angle);
        azimuth = atan2(module_vy, module_vx);
        wheel_velocity = sqrt(module_vx * module_vx + module_vy * module_vy);
        if (servo_min_angle < 0.0) {
            azimuth += M_PI;
            wheel_velocity = -wheel_velocity;
        }
    }
}

void BwDriveModule::set(double vx, double vy, double vt, double dt)
{
    double azimuth, wheel_velocity;
    azimuth = setpoint_angle;
    compute_state(vx, vy, vt, dt, azimuth, wheel_velocity);

    azimuth = wrap_angle(azimuth);
    if (azimuth < servo_min_angle || azimuth > servo_max_angle) {
        azimuth = wrap_angle(azimuth + M_PI);
        wheel_velocity = -wheel_velocity;
    }

    if (abs(predicted_velocity) > 0.001) {
        double delta = predicted_velocity * wheel_radius;
        if (flip_motor_commands) {
            delta *= -1.0;
        }
        wheel_velocity += delta;
    }

    set_azimuth(azimuth, dt);
    set_wheel_velocity(wheel_velocity);
}

void BwDriveModule::set_pwm_frequency(int frequency)
{
    motor->set_frequency(frequency);
}

double BwDriveModule::dt()
{
    uint32_t current_time = millis();
    uint32_t delta_time = current_time - prev_time;
    prev_time = current_time;
    return (double)delta_time * 1E-3;
}
