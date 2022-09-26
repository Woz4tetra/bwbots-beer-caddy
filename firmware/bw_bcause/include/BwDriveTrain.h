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
    Adafruit_PWMServoDriver* servos;

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
    void drive(double vx, double vy, double vt);
    void set(unsigned int channel, double angle, double velocity);
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
        double servo_angle_1,
        double servo_angle_2,
        int servo_command_1,
        int servo_command_2,
        double servo_max_velocity
    );
};
