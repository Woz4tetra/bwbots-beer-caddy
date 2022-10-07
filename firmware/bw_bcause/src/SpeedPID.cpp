#include <SpeedPID.h>


SpeedPID::SpeedPID()
{
    target = 0.0;
    error_sum = 0.0;
    prev_error = 0.0;
    feedforward = 0.0;
    current_time = 0;
    prev_update_time = 0;
    dt = 0.0;
    out = 0.0;
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


void SpeedPID::set_target(double target) {
    feedforward = K_ff * target;
    this->target = target;
    prev_update_time = 0;
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

    int value = (int)command;
    if (value > command_max) {
        return command_max;
    }
    if (value < command_min) {
        return command_min;
    }

    if (abs(value) < deadzone_command) {
        return sign(value) * deadzone_command;
    }

    return value;
}

int SpeedPID::compute(double measurement)
{
    current_time = millis();

    if (current_time - prev_update_time == 0) {
        Serial.println("PID timer didn't change!");
        return limit(out);
    }
    else if (current_time - prev_update_time < 0) {  // edge case for timer looping
        Serial.println("PID timer overflowed! Resetting.");
        prev_update_time = 0;
    }

    double error = target - measurement;
    dt = (current_time - prev_update_time) * 1E-3;
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
