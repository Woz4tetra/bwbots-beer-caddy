#pragma once
#include <Arduino.h>


class SpeedPID {
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
    int deadzone_command;
    double error_sum_clamp;
    int command_min, command_max;
    double epsilon;  // values that are basically zero

    SpeedPID();
    bool timed_out();
    void set_target(double target);
    double get_target();
    void reset();
    int limit(double value);
    int compute(double measurement);

    static double sign(double x);
    static int sign(int x);
};
