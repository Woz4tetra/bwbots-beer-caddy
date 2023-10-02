#pragma once
#include <Arduino.h>


class PID {
private:
    double target;
    double error_sum, prev_error;
    double feedforward;
    uint32_t current_time, prev_update_time;
    double dt;
    double out;

public:
    double Kp, Ki, Kd;
    double K_ff;  // feedforward constant
    double deadzone_command;
    double error_sum_clamp;
    double command_min, command_max;
    double epsilon;  // values that are basically zero

    PID();
    void set_target(double target);
    double get_target();
    void reset();
    double limit(double value);
    double compute(double measurement);
    double get_last_command()  { return out; };

    static double sign(double x);
    static int sign(int x);
};