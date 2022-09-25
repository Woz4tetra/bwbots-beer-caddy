#include <Arduino.h>


class SpeedPID {
private:
    double target;
    double error_sum, prev_error;
    double feedforward;
    uint32_t prev_setpoint_time;
    uint32_t current_time, prev_update_time;
    double dt;
    double out;
    bool is_timed_out;

public:
    double Kp, Ki, Kd;
    double K_ff;  // feedforward constant
    double deadzone;
    double error_sum_clamp;
    int command_min, command_max;

    SpeedPID();
    bool timed_out();
    void set_target(double target);
    double get_target();
    void reset();
    int limit(double value);
    int compute(double measurement);
};
