#include <BwDriveTrain.h>


BwDriveTrain::BwDriveTrain(
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
    )
{
    this->num_motors = num_motors;
    this->motor_enable_pin = motor_enable_pin;
    this->servos = servos;
    this->armature_length = armature_length;
    this->min_radius_of_curvature = min_radius_of_curvature;
    is_enabled = false;
    min_strafe_angle = -M_PI;
    max_strafe_angle = M_PI;
    reverse_min_strafe_angle = -M_PI;
    reverse_max_strafe_angle = M_PI;
    prev_drive_time = 0;
    prev_odom_time = 0;

    if (this->num_motors > BwDriveTrain::MAX_CHANNELS) {
        this->num_motors = BwDriveTrain::MAX_CHANNELS;
    }
    kinematics_channels_len = 2 * get_num_motors();
    inverse_kinematics = new mtx_type[kinematics_channels_len * CHASSIS_STATE_LEN];
    forward_kinematics = new mtx_type[CHASSIS_STATE_LEN * kinematics_channels_len];
    ik_transpose = new mtx_type[CHASSIS_STATE_LEN * kinematics_channels_len];
    kinematics_temp = new mtx_type[CHASSIS_STATE_LEN * CHASSIS_STATE_LEN];
    chassis_vector = new mtx_type[CHASSIS_STATE_LEN * 1];
    module_vector = new mtx_type[kinematics_channels_len * 1];
    drive_modules = new BwDriveModule*[get_num_motors()];

    for (unsigned int index = 0; index < kinematics_channels_len * CHASSIS_STATE_LEN; index++) {
        inverse_kinematics[index] = 0.0;
        forward_kinematics[index] = 0.0;
        ik_transpose[index] = 0.0;
    }
    for (unsigned int index = 0; index < CHASSIS_STATE_LEN * CHASSIS_STATE_LEN; index++) {
        kinematics_temp[index] = 0.0;
    }
    for (unsigned int index = 0; index < CHASSIS_STATE_LEN; index++) {
        chassis_vector[index] = 0.0;
    }
    for (unsigned int index = 0; index < kinematics_channels_len; index++) {
        module_vector[index] = 0.0;
    }

    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        double x_location = x_locations[channel];
        double y_location = y_locations[channel];
        drive_modules[channel] = new BwDriveModule(
            channel, output_ratio, x_location, y_location, min_radius_of_curvature, armature_length,
            servos, motors[channel], encoders[channel]
        );

        unsigned int y_row = CHASSIS_STATE_LEN * (2 * channel);
        unsigned int x_row = CHASSIS_STATE_LEN * (2 * channel + 1);
        //  0  1  2 -- channel 0, -y0
        //  3  4  5 -- channel 0, x0
        //  6  7  8 -- channel 1, -y1
        //  9 10 11 -- channel 1, x1
        // 12 13 14 -- channel 2, -y2
        // 15 16 17 -- channel 2, x2
        // 18 19 20 -- channel 3, -y3
        // 21 22 23 -- channel 3, x3

        inverse_kinematics[y_row + 0] = 1.0;
        inverse_kinematics[y_row + 1] = 0.0;
        inverse_kinematics[y_row + 2] = 0.0;  // filled in later
        inverse_kinematics[x_row + 0] = 0.0;
        inverse_kinematics[x_row + 1] = 1.0;
        inverse_kinematics[x_row + 2] = 0.0;  // filled in later
    }
}

unsigned int BwDriveTrain::get_num_motors() {
    return num_motors;
}

void BwDriveTrain::begin()
{
    is_enabled = true;
    set_enable(false);
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->begin();
    }
}

void BwDriveTrain::set_enable(bool state) {
    if (is_enabled == state) {
        return;
    }
    is_enabled = state;
    digitalWrite(motor_enable_pin, state);
    if (is_enabled) {
        reset();
    }
    else {
        for (unsigned int channel = 0; channel < MAX_CHANNELS; channel++) {
            servos->setPWM(channel, 0, 4096);
        }
    }

    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_enable(is_enabled);
    }
}

void BwDriveTrain::reset()
{
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->reset();
    }
}


bool BwDriveTrain::get_enable() {
    return is_enabled;
}

void BwDriveTrain::set_limits(
    unsigned int channel,
    double servo_min_angle,
    double servo_max_angle,
    double servo_angle_1,
    double servo_angle_2,
    int servo_command_1,
    int servo_command_2,
    double servo_max_velocity,
    bool flip_motor_commands)
{
    if (channel > get_num_motors()) {
        return;
    }
    drive_modules[channel]->set_limits(
        servo_min_angle,
        servo_max_angle,
        servo_angle_1,
        servo_angle_2,
        servo_command_1,
        servo_command_2,
        servo_max_velocity,
        flip_motor_commands
    );
    double min_angle = min(abs(wrap_angle(servo_min_angle)), abs(wrap_angle(servo_max_angle)));
    if (min_angle < max_strafe_angle) {
        max_strafe_angle = min_angle;
        min_strafe_angle = -min_angle;
        reverse_max_strafe_angle = M_PI - max_strafe_angle;
        reverse_min_strafe_angle = -M_PI - min_strafe_angle;
    }
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_strafe_limits(min_strafe_angle, max_strafe_angle);
    }
}

void BwDriveTrain::set(unsigned int channel, double azimuth, double wheel_velocity)
{
    if (channel <= get_num_motors()) {
        drive_modules[channel]->set_azimuth(azimuth);
        drive_modules[channel]->set_wheel_velocity(wheel_velocity);
    }
}


void BwDriveTrain::drive(double vx, double vy, double vt)
{
    double delta_time = dt(prev_drive_time);
    double v_theta = atan2(vy, vx);

    if ((v_theta > max_strafe_angle && v_theta < reverse_max_strafe_angle) || (v_theta < min_strafe_angle && v_theta > reverse_min_strafe_angle)) {
        double v_mag = sqrt(vx * vx + vy * vy);
        if (0.0 <= v_theta && v_theta < M_PI / 2.0) {
            v_theta = max_strafe_angle;
        }
        else if (M_PI / 2.0 <= v_theta && v_theta <= M_PI) {
            v_theta = reverse_max_strafe_angle;
        }
        else if (-M_PI / 2.0 <= v_theta && v_theta < 0.0) {
            v_theta = min_strafe_angle;
        }
        else if (-M_PI <= v_theta && v_theta < -M_PI / 2.0) {
            v_theta = reverse_min_strafe_angle;
        }
        
        vx = v_mag * cos(v_theta);
        vy = v_mag * sin(v_theta);
    }
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set(vx, vy, vt, delta_time);
    }
}

void BwDriveTrain::get_position(double& x, double& y, double& theta, double vx, double vy, double vt)
{
    double delta_time = dt(prev_odom_time);
    double dx = vx * delta_time;
    double dy = vy * delta_time;
    double dtheta = vt * delta_time;
    double sin_theta = sin(dtheta);
    double cos_theta = cos(dtheta);

    double s, c;
    if (abs(dtheta) < 1E-9) {  // if angle is too small, use taylor series linear approximation
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
    }
    else {
        s = sin_theta / dtheta;
        c = (1.0 - cos_theta) / dtheta;
    }

    double tx = dx * s - dy * c;
    double ty = dx * c + dy * s;

    double rotated_tx = tx * cos(theta) - ty * sin(theta);
    double rotated_ty = tx * sin(theta) + ty * cos(theta);

    x += rotated_tx;
    y += rotated_ty;
    theta += dtheta;
}

void BwDriveTrain::get_velocity(double& vx, double& vy, double& vt)
{
    vx = 0.0;
    vy = 0.0;
    vt = 0.0;
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        double azimuth = get_azimuth(channel);
        double x = drive_modules[channel]->get_x_location() + this->armature_length * cos(-azimuth);
        double y = drive_modules[channel]->get_y_location() + this->armature_length * sin(-azimuth);
        //  0  1  2 -- channel 0, -y0
        //  3  4  5 -- channel 0, x0
        //  6  7  8 -- channel 1, -y1
        //  9 10 11 -- channel 1, x1
        // 12 13 14 -- channel 2, -y2
        // 15 16 17 -- channel 2, x2
        // 18 19 20 -- channel 3, -y3
        // 21 22 23 -- channel 3, x3
        unsigned int y_index = CHASSIS_STATE_LEN * (2 * channel) + 2;
        unsigned int x_index = CHASSIS_STATE_LEN * (2 * channel + 1) + 2;
        inverse_kinematics[y_index] = -y;
        inverse_kinematics[x_index] = x;
        
        double wheel_velocity = get_wheel_velocity(channel);
        double module_vx = wheel_velocity * cos(azimuth);
        double module_vy = wheel_velocity * sin(azimuth);
        unsigned int even_index = 2 * channel;
        unsigned int odd_index = 2 * channel + 1;
        module_vector[even_index] = module_vx;
        module_vector[odd_index] = module_vy;
    }

    // Matrix.Print((mtx_type*)inverse_kinematics, kinematics_channels_len, CHASSIS_STATE_LEN, "M");

    // pinv(M) = (M' * M) ^-1 * M
    // (mtx_type*) cast "flattens" the matrix to a 1 x (col * rows) array

    // ik_transpose = M'
    Matrix.Transpose(
        inverse_kinematics,  // source
        kinematics_channels_len,  // rows
        CHASSIS_STATE_LEN,  // columns
        ik_transpose  // destination
    );
    // Matrix.Print((mtx_type*)ik_transpose, CHASSIS_STATE_LEN, kinematics_channels_len, "M.T");

    // kinematics_temp = M' * M
    Matrix.Multiply(
        ik_transpose,
        inverse_kinematics,
        CHASSIS_STATE_LEN,  // rows of ik_transpose
        kinematics_channels_len,  // columns of ik_transpose / rows of inverse_kinematics
        CHASSIS_STATE_LEN,  // columns of inverse_kinematics
        kinematics_temp  // destination
    );

    // Matrix.Print((mtx_type*)kinematics_temp, CHASSIS_STATE_LEN, CHASSIS_STATE_LEN, "M' * M");

    // kinematics_temp = (M' * M)^-1
    if (!Matrix.Invert(kinematics_temp, CHASSIS_STATE_LEN)) {
        Serial.println("Failed to invert matrix for velocity calculation!");
        return;
    }

    // Matrix.Print((mtx_type*)kinematics_temp, CHASSIS_STATE_LEN, CHASSIS_STATE_LEN, "(M' * M)^-1");

    // forward_kinematics = (M' * M)^-1 * M' = pinv(M)
    Matrix.Multiply(
        kinematics_temp,
        ik_transpose,
        CHASSIS_STATE_LEN,  // rows of kinematics_temp
        CHASSIS_STATE_LEN,  // columns of ik_transpose / rows of kinematics_temp
        kinematics_channels_len,  // columns of ik_transpose
        forward_kinematics  // destination
    );

    // Matrix.Print((mtx_type*)forward_kinematics, CHASSIS_STATE_LEN, kinematics_channels_len, "(M' * M)^-1 * M'");

    // chassis_vector = pinv(M) * module_vector
    Matrix.Multiply(
        forward_kinematics,
        module_vector,
        CHASSIS_STATE_LEN,  // rows of forward_kinematics
        kinematics_channels_len,  // columns of forward_kinematics / rows of module_vector
        1,  // columns of module_vector
        chassis_vector  // destination
    );

    // Matrix.Print((mtx_type*)module_vector, kinematics_channels_len, 1, "mv");
    // Matrix.Print((mtx_type*)chassis_vector, CHASSIS_STATE_LEN, 1, "pinv(M) * mv");

    vx = chassis_vector[0];
    vy = chassis_vector[1];
    vt = chassis_vector[2];
}

void BwDriveTrain::stop()
{
    for (unsigned int channel = 0; channel < get_num_motors(); channel++) {
        drive_modules[channel]->set_wheel_velocity(0.0);
    }
}

double BwDriveTrain::wrap_angle(double angle)
{
    // wrap to -pi..pi
    angle = fmod(angle, 2.0 * M_PI);
    if (angle >= M_PI) {
        angle -= 2.0 * M_PI;
    }
    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double BwDriveTrain::dt(uint32_t& prev_time)
{
    uint32_t current_time = millis();
    uint32_t delta_time = current_time - prev_time;
    prev_time = current_time;
    return (double)delta_time * 1E-3;
}


double BwDriveTrain::get_wheel_velocity(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_wheel_velocity();
    }
}

double BwDriveTrain::get_wheel_position(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_wheel_position();
    }
}

double BwDriveTrain::get_azimuth(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return 0.0;
    }
    else {
        return drive_modules[channel]->get_azimuth();
    }
}

SpeedPID* BwDriveTrain::get_pid(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return NULL;
    }
    else {
        return drive_modules[channel]->get_pid();
    }
}

SpeedFilter* BwDriveTrain::get_filter(unsigned int channel)
{
    if (channel > get_num_motors()) {
        return NULL;
    }
    else {
        return drive_modules[channel]->get_filter();
    }
}
