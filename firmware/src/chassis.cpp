#include "chassis.h"

Chassis::Chassis(
    int motor_stby,
    int motorl_dr1, int motorl_dr2, int motorl_pwm, int motorr_dr1, int motorr_dr2, int motorr_pwm,
    int motorl_enca, int motorl_encb, int motorr_enca, int motorr_encb)
{
    this->motor_stby = motor_stby;
    left_motor = new TB6612(motorl_pwm, motorl_dr1, motorl_dr2);
    right_motor = new TB6612(motorr_pwm, motorr_dr1, motorr_dr2);
    left_encoder = new Encoder(motorl_enca, motorl_encb);
    right_encoder = new Encoder(motorr_enca, motorr_encb);

    prev_enc_time = 0;
    motors_enabled = false;
    left_enc_pos = 0;
    right_enc_pos = 0;
    left_enc_speed = 0.0;
    right_enc_speed = 0.0;
    left_enc_speed_raw = 0.0;
    right_enc_speed_raw = 0.0;
    speed_smooth_left_k = 1.0;
    speed_smooth_right_k = 1.0;
    sample_rate = 33;
}

void Chassis::begin()
{
    pinMode(motor_stby, OUTPUT);
    left_motor->begin();
    right_motor->begin();
}

void Chassis::set_motor_enable(bool state)
{
    motors_enabled = state;
    digitalWrite(motor_stby, state);
}

bool Chassis::get_motor_enable() {
    return motors_enabled;
}

void Chassis::set_left_motor(int command)
{
    left_command = command;
    left_motor->setSpeed(command);
}

void Chassis::set_right_motor(int command)
{
    right_command = command;
    right_motor->setSpeed(command);
}

void Chassis::reset_encoders() {
    reset_encoders(0, 0);
}

void Chassis::reset_encoders(long left, long right)
{
    left_enc_pos = left;
    right_enc_pos = right;
    left_encoder->write(left);
    right_encoder->write(right);
}

bool Chassis::update()
{
    uint32_t current_time = millis();
    if (current_time - prev_enc_time < sample_rate) {
        return false;
    }
    if (current_time < prev_enc_time) {
        prev_enc_time = current_time;
        return false;
    }

    long new_left = left_encoder->read();
    long new_right = right_encoder->read();

    uint32_t dt = current_time - prev_enc_time;

    left_enc_speed_raw = (double)(new_left - left_enc_pos) / dt * 1000.0;
    right_enc_speed_raw = (double)(new_right - right_enc_pos) / dt * 1000.0;
    left_enc_speed += speed_smooth_left_k * (left_enc_speed_raw - left_enc_speed);
    right_enc_speed += speed_smooth_right_k * (right_enc_speed_raw - right_enc_speed);

    left_enc_pos = new_left;
    right_enc_pos = new_right;

    prev_enc_time = current_time;

    return true;
}
