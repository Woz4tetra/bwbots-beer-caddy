#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <MotorControllerMC33926.h>
#include <SpeedPID.h>
#include <SpeedFilter.h>

class BwDriveModule
{
private:
    Adafruit_PWMServoDriver* servos;
    MotorControllerMC33926* motor;
    Encoder* encoder;

    long encoder_position;
    SpeedPID* speed_pid;
    SpeedFilter* speed_filter;
    int channel;
    double output_ratio;  // encoder counts * output_ratio = m/s at wheel
    double wheel_radius;
    double servo_min_angle, servo_max_angle;
    double servo_angle_1, servo_angle_2;
    int servo_command_1, servo_command_2;
    double servo_max_velocity;
    double setpoint_angle, predicted_angle, predicted_velocity;
    double x_location, y_location;
    double armature_length;
    double min_radius_of_curvature;
    double min_strafe_angle, max_strafe_angle;
    uint32_t prev_time;
    bool is_enabled;
    bool flip_motor_commands;

    double dt();
    void update_predicted_azimuth(double dt);
    double wrap_angle(double angle);
    void compute_state(double vx, double vy, double vt, double dt, double& azimuth, double& wheel_velocity);
    void update_wheel_velocity();
    void update_wheel_position();
public:
    BwDriveModule(
        int channel,
        double output_ratio,
        double x_location,
        double y_location,
        double min_radius_of_curvature,
        double armature_length,
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926* motor,
        Encoder* encoder
    );
    void begin();
    void reset();
    void set_enable(bool state);
    void set_limits(
        double servo_min_angle,
        double servo_max_angle,
        double servo_angle_1,
        double servo_angle_2,
        int servo_command_1,
        int servo_command_2,
        double servo_max_velocity,
        double wheel_radius,
        bool flip_motor_commands
    );
    void set_strafe_limits(double min_strafe_angle, double max_strafe_angle);
    double get_azimuth();
    double get_azimuth_setpoint();
    double get_wheel_position();
    double get_wheel_velocity();
    double get_wheel_velocity_setpoint();
    double get_x_location()  { return x_location; }
    double get_y_location()  { return y_location; }
    void set(double vx, double vy, double vt, double dt);
    void set_azimuth(double setpoint, double dt);
    void set_wheel_velocity(double velocity);
    void set_pwm_frequency(int frequency);
    void command_wheel_pwm(int command);
    SpeedPID* get_pid()  { return speed_pid; }
    SpeedFilter* get_filter()  { return speed_filter; }
    MotorControllerMC33926* get_motor()  { return motor; }
};
