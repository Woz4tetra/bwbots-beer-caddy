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
    double servo_min_angle;
    double servo_max_angle;
    unsigned int servo_min_command;
    unsigned int servo_max_command;
    double servo_max_velocity;
    double setpoint_angle, predicted_angle;
    uint32_t prev_time;

    double dt();
    void update_predicted_angle();
public:
    BwDriveModule(int channel, double output_ratio, Adafruit_PWMServoDriver* servos, MotorControllerMC33926* motor, Encoder* encoder);
    void begin();
    void reset();
    void set_direction(double setpoint);
    void set_limits(
        double servo_min_angle,
        double servo_max_angle,
        int servo_min_command,
        int servo_max_command,
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
