#pragma once

#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <MotorControllerMC33926.h>
#include <SpeedPID.h>
#include <SpeedFilter.h>
#include <BwDriveModule.h>

class BwDriveTrain
{
private:
    unsigned int num_motors;
    unsigned int motor_enable_pin;
    bool is_enabled;
    static const unsigned int MAX_CHANNELS = 16;
    BwDriveModule** drive_modules;

public:
    BwDriveTrain(
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926** motors,
        Encoder** encoders,
        unsigned int num_motors,
        unsigned int motor_enable_pin,
        double output_ratio
    );
    void begin();
    void set_enable(bool state);
    bool get_enable();
    void reset();
    void drive(float vx, float vy, float vt);
    SpeedPID* get_pid(unsigned int channel);
    SpeedFilter* get_filter(unsigned int channel);
    unsigned int get_num_motors();
    double get_velocity(unsigned int channel);
    double get_position(unsigned int channel);
    double get_angle(unsigned int channel);
    void set_limits(
        unsigned int channel,
        double servo_min_angle,
        double servo_max_angle,
        int servo_min_command,
        int servo_max_command,
        double servo_max_velocity
    );
};
