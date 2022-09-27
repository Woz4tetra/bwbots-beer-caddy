#include <BwDriveModule.h>

BwDriveModule::BwDriveModule(int channel, double output_ratio, Adafruit_PWMServoDriver* servos, MotorControllerMC33926* motor, Encoder* encoder)
{
    this->channel = channel;
    this->output_ratio = output_ratio;
    this->servos = servos;
    this->motor = motor;
    this->encoder = encoder;
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
    Serial.print(channel);
    Serial.print("\tsetpoint 1: ");
    Serial.print(setpoint);
    if (setpoint < servo_min_angle) {
        setpoint = servo_min_angle;
    }
    if (setpoint > servo_max_angle) {
        setpoint = servo_max_angle;
    }
    Serial.print("\tsetpoint 2: ");
    Serial.println(setpoint);
    
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
    double error = setpoint_angle - predicted_angle;
    if (abs(error) < 0.1) { // TODO: make this configurable
        predicted_angle = setpoint_angle;
    }
    double delta_time = dt();
    double servo_delta = error * delta_time;  // TODO: add ramping constant
    if (abs(servo_delta) > servo_max_velocity) {
        servo_delta = SpeedPID::sign(error) * servo_max_velocity * delta_time;
    }
    predicted_angle += servo_delta;
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


double BwDriveModule::dt()
{
    uint32_t delta_time = micros() - prev_time;
    return (double)delta_time * 1E-6;
}
