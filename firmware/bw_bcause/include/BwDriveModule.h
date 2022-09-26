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
    double servo_min_angle, servo_max_angle;
    double servo_angle_1, servo_angle_2;
    int servo_command_1, servo_command_2;
    double servo_max_velocity;
    double setpoint_angle, predicted_angle;
    uint32_t prev_time;
    bool is_enabled;

    double dt();
    void update_predicted_angle();
public:
    BwDriveModule(int channel, double output_ratio, Adafruit_PWMServoDriver* servos, MotorControllerMC33926* motor, Encoder* encoder);
    void begin();
    void reset();
    void set_enable(bool state);
    void set_direction(double setpoint);
    void set_limits(
        double servo_min_angle,
        double servo_max_angle,
        double servo_angle_1,
        double servo_angle_2,
        int servo_command_1,
        int servo_command_2,
        double servo_max_velocity
    );
    double get_angle();
    double get_position();
    double get_velocity();
    double update_velocity();
    void set_velocity(double velocity);
    SpeedPID* get_pid()  { return speed_pid; }
    SpeedFilter* get_filter()  { return speed_filter; }
};
