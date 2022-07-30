#pragma once

#include <Arduino.h>
#include <TB6612.h>
#include <Encoder.h>

class Chassis
{
public:
    Chassis(
        int motor_stby,
        int motorl_dr1, int motorl_dr2, int motorl_pwm, int motorr_dr1, int motorr_dr2, int motorr_pwm,
        int motorl_enca, int motorl_encb, int motorr_enca, int motorr_encb
    );
    void begin();

    void set_motor_enable(bool state);
    bool get_motor_enable();

    void set_left_motor(int command);
    void set_right_motor(int command);

    void reset_encoders();
    void reset_encoders(long left, long right);
    
    bool update();

    long get_left_encoder()  { return left_enc_pos; }
    long get_right_encoder()  { return right_enc_pos; }

    double get_left_speed()  { return left_enc_speed; }
    double get_right_speed()  { return right_enc_speed; }

    int get_left_command()  { return left_command; }
    int get_right_command()  { return right_command; }

    double get_speed_smooth_left_k()  { return speed_smooth_left_k; }
    double get_speed_smooth_right_k()  { return speed_smooth_right_k; }
    void set_speed_smooth_left_k(double k)  { speed_smooth_left_k = k; }
    void set_speed_smooth_right_k(double k)  { speed_smooth_right_k = k; }

    long get_sample_rate()  { return sample_rate; }
    void set_sample_rate(long rate)  { sample_rate = rate; }
private:
    int motor_stby;

    TB6612* left_motor;
    TB6612* right_motor;

    Encoder* left_encoder;
    Encoder* right_encoder;

    uint32_t prev_enc_time;

    bool motors_enabled;
    long left_enc_pos, right_enc_pos;
    double left_enc_speed, right_enc_speed;  // ticks/s, smoothed
    double left_enc_speed_raw, right_enc_speed_raw;  // ticks/s
    int left_command, right_command;

    double speed_smooth_left_k, speed_smooth_right_k;
    uint32_t sample_rate;
};
