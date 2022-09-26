#include <SpeedPID.h>


SpeedPID::SpeedPID()
{
    target = 0.0;
    error_sum = 0.0;
    prev_error = 0.0;
    feedforward = 0.0;
    prev_setpoint_time = 0;
    current_time = 0;
    prev_update_time = 0;
    dt = 0.0;
    out = 0.0;
    is_timed_out = false;
    command_min = -255;
    command_max = 255;
    epsilon = 1E-4;

    K_ff = 0.0;
    deadzone_command = 0;
    error_sum_clamp = 0.0;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
}

double SpeedPID::sign(double x) {
    return (x > 0) - (x < 0);
}

int SpeedPID::sign(int x) {
    return (x > 0) - (x < 0);
}


bool SpeedPID::timed_out() {
    return is_timed_out;
}

void SpeedPID::set_target(double target) {
    feedforward = K_ff * target;
    this->target = target;
    prev_setpoint_time = millis();
    prev_update_time = micros();
    is_timed_out = false;
}

double SpeedPID::get_target() {
    return target;
}

void SpeedPID::reset() {
    prev_error = 0.0;
    error_sum = 0.0;
    set_target(0.0);
}

int SpeedPID::limit(double command) {
    if (abs(command) < epsilon) {
        return 0;
    }

    int value = command;
    if (value > command_max) {
        return command_max;
    }
    if (value < command_min) {
        return command_min;
    }

    if (abs(value) < deadzone_command) {
        return sign(value) * deadzone_command;
    }

    return (int)(value);
}

int SpeedPID::compute(double measurement)
{
    if (!is_timed_out && millis() - prev_setpoint_time > command_timeout_ms) {
        reset();
        is_timed_out = true;
    }

    if (micros() - prev_update_time == 0) {
        return out;
    }
    else if (micros() - prev_update_time < 0) {  // edge case for timer looping
        prev_update_time = micros();
        return out;
    }

    double error = target - measurement;
    current_time = micros();
    dt = (current_time - prev_update_time) * 1E-6;
    prev_update_time = current_time;

    out = 0.0;
    if (Kp > 0.0) {
        out += Kp * error;
    }
    if (Kd > 0.0) {
        out += Kd * (error - prev_error) / dt;
        prev_error = error;
    }
    if (Ki > 0.0) {
        out += Ki * error_sum * dt;
        error_sum += error;
        if (error_sum_clamp >= 0.0 && abs(error_sum) > error_sum_clamp) {
            error_sum = sign(error_sum) * error_sum_clamp;
        }
    }
    out += feedforward;

    return limit(out);
}
