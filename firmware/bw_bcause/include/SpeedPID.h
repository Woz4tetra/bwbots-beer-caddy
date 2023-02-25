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
    double deadzone_command, standstill_deadzone_command;
    double K_ff;  // feedforward constant

public:
    double Kp, Ki, Kd;
    double error_sum_clamp;
    double command_min, command_max;
    double epsilon;  // values that are basically zero

    SpeedPID();
    void set_target(double target);
    double get_target();
    void reset();
    double limit(double value);
    double compute(double measurement);
    double get_last_command()  { return out; };
    void set_deadzones(double K_ff, double deadzone_command, double standstill_deadzone_command);

    static double sign(double x);
    static int sign(int x);
};
