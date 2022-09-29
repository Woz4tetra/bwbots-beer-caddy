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
    double armature_length;
    uint32_t prev_time;
    double min_strafe_angle, max_strafe_angle;
    double reverse_min_strafe_angle, reverse_max_strafe_angle;
    double min_radius_of_curvature;
    BwDriveModule** drive_modules;
    Adafruit_PWMServoDriver* servos;

    double dt();

public:
    BwDriveTrain(
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926** motors,
        Encoder** encoders,
        unsigned int num_motors,
        unsigned int motor_enable_pin,
        double output_ratio,
        double armature_length,
        double min_radius_of_curvature,
        double* x_locations,
        double* y_locations
    );
    void begin();
    void set_enable(bool state);
    bool get_enable();
    void reset();
    void stop();
    void drive(double vx, double vy, double vt);
    void set(unsigned int channel, double angle, double velocity);
    SpeedPID* get_pid(unsigned int channel);
    SpeedFilter* get_filter(unsigned int channel);
    unsigned int get_num_motors();
    double get_wheel_velocity(unsigned int channel);
    double get_wheel_position(unsigned int channel);
    double get_azimuth(unsigned int channel);
    static double wrap_angle(double angle);
    void set_limits(
        unsigned int channel,
        double servo_min_angle,
        double servo_max_angle,
        double servo_angle_1,
        double servo_angle_2,
        int servo_command_1,
        int servo_command_2,
        double servo_max_velocity,
        bool flip_motor_commands
    );
};
