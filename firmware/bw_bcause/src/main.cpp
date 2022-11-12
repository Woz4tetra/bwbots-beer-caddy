#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <MotorControllerMC33926.h>
#include <BwDriveTrain.h>
#include <BwUISequencer.h>
#include <tunnel_serial.h>

#define I2C_BUS_1 Wire

// ---
// Serial
// ---

#define DEBUG_SERIAL Serial2
#define DEBUG_BAUD 9600

#define PROTOCOL_SERIAL Serial
#define PROTOCOL_BAUD 1000000

// ---
// Tunnel
// ---
TunnelSerial* tunnel;

// ---
// Status LEDs
// ---
const int STATUS_LED = 0;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;

// ---
// Buttons
// ---

const int BUTTON_IN = 15;
bool prev_button_state = false;

// ---
// Drive controllers
// ---
const int MOTOR_EN = 1;
const int NUM_CHANNELS = 4;

const int M1_SPEED = 9;  // FlexPWM2.2
const int M1_DIR_P = 11;
const int M1_DIR_N = 12;
const int M1_SF = 32;
const int M1_FB = 23;

const int M2_SPEED = 10;  // QuadTimer1.0	
const int M2_DIR_P = 28;
const int M2_DIR_N = 29;
const int M2_SF = 17;
const int M2_FB = 22;

const int M3_SPEED = 37;  // FlexPWM2.3
const int M3_DIR_P = 30;
const int M3_DIR_N = 31;
const int M3_SF = 38;
const int M3_FB = 21;

const int M4_SPEED = 36;  // FlexPWM2.3
const int M4_DIR_P = 35;
const int M4_DIR_N = 34;
const int M4_SF = 33;
const int M4_FB = 20;

// Set PWM frequency of the speed pin to X Hz.
// Default for pin 36, 37 on Teensy 4.1: 4.482 kHz
// https://www.pjrc.com/teensy/td_pulse.html
// Setting this to 500 Hz causes a weird exponential response
// Setting above 5000 Hz creates poor low speed performance
const int MOTOR_PWM_FREQUENCY = 3520;

MotorControllerMC33926* motors[NUM_CHANNELS] = {
    new MotorControllerMC33926(M1_SPEED, M1_DIR_P, M1_DIR_N, M1_SF, M1_FB, MOTOR_PWM_FREQUENCY),
    new MotorControllerMC33926(M2_SPEED, M2_DIR_P, M2_DIR_N, M2_SF, M2_FB, MOTOR_PWM_FREQUENCY),
    new MotorControllerMC33926(M3_SPEED, M3_DIR_P, M3_DIR_N, M3_SF, M3_FB, MOTOR_PWM_FREQUENCY),
    new MotorControllerMC33926(M4_SPEED, M4_DIR_P, M4_DIR_N, M4_SF, M4_FB, MOTOR_PWM_FREQUENCY)
};

const int MOTOR1_ENCA = 3;
const int MOTOR1_ENCB = 4;
const int MOTOR2_ENCA = 5;
const int MOTOR2_ENCB = 6;
const int MOTOR3_ENCA = 24;
const int MOTOR3_ENCB = 25;
const int MOTOR4_ENCA = 26;
const int MOTOR4_ENCB = 27;

Encoder* encoders[NUM_CHANNELS] = {
    new Encoder(MOTOR1_ENCA, MOTOR1_ENCB),
    new Encoder(MOTOR2_ENCA, MOTOR2_ENCB),
    new Encoder(MOTOR3_ENCA, MOTOR3_ENCB),
    new Encoder(MOTOR4_ENCA, MOTOR4_ENCB)
};

// ---
// Servos
// ---

const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
const int SERVO_OSC_FREQ = 27000000;
Adafruit_PWMServoDriver* servos = new Adafruit_PWMServoDriver(0x40 + 0b000010, I2C_BUS_1);

// ---
// Drive train
// ---
const double SPEED_TO_COMMAND = 255.0 / 1.0;  // calculated max speed: 0.843 m/s @ 12V
const double MAX_SERVO_SPEED = 6.5;  // calculated max speed: 5.950 rad/s @ 5V

const int DEADZONE_COMMAND = 50;
const int STANDSTILL_DEADZONE_COMMAND = 110;
const int MAX_SPEED_COMMAND = 255;

const double ALCOVE_ANGLE = 0.5236;  // 30 degrees
const double FRONT_ANGLE = -1.2967;  // -74.293 degrees
const double STRAIGHT_ANGLE = 0.0;  // 0 degrees

const double GEAR_RATIO = 54.0;
const double ENCODER_PPR = 64.0;  // pulses per rotation
const double WHEEL_DIAMETER = 0.115;  // meters
const double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;  // meters
const double OUTPUT_RATIO = -2.0 * M_PI * WHEEL_RADIUS / (GEAR_RATIO * ENCODER_PPR);  // encoder counts (pulses) * output_ratio = meters at wheel

const double WIDTH = 0.115;  // meters, chassis pivot to pivot Y dimension
const double LENGTH = 0.160;  // meters, chassis pivot to pivot X dimension
const double ARMATURE = 0.037;  // meters, pivot to wheel center dimension

// ordered by channel index
double MODULE_X_LOCATIONS[NUM_CHANNELS] = {-LENGTH / 2.0, -LENGTH / 2.0, LENGTH / 2.0, LENGTH / 2.0};
double MODULE_Y_LOCATIONS[NUM_CHANNELS] = {WIDTH / 2.0, -WIDTH / 2.0, WIDTH / 2.0, -WIDTH / 2.0};

const double MIN_RADIUS_OF_CURVATURE = 0.1;

const int FRONT_LEFT = 2;  // module 3
const int BACK_LEFT = 0;  // module 1
const int BACK_RIGHT = 1;  // module 2
const int FRONT_RIGHT = 3;  // module 4

// Servo commands that correspond to real dimensions
const int FRONT_LEFT_ALCOVE = 230;
const int FRONT_LEFT_STRAIGHT = 140;
const int BACK_LEFT_ALCOVE = 375;
const int BACK_LEFT_STRAIGHT = 470;
const int BACK_RIGHT_ALCOVE = 235;
const int BACK_RIGHT_STRAIGHT = 125;
const int FRONT_RIGHT_ALCOVE = 405;
const int FRONT_RIGHT_STRAIGHT = 515;

typedef enum CONTROL_MODE {
    CONTROL_GLOBAL = 0,
    CONTROL_INDIVIDUAL = 1,
    CONTROL_TONE = 2
} control_mode_t;

control_mode_t control_mode = CONTROL_GLOBAL;

BwDriveTrain* drive;
double vx_command = 0.0, vy_command = 0.0, vt_command = 0.0;
double odom_vx = 0.0, odom_vy = 0.0, odom_vt = 0.0;
double odom_x = 0.0, odom_y = 0.0, odom_t = 0.0;

// ---
// Power management
// ---

float shunt_voltage = 0.0f;
float bus_voltage = 0.0f;
float current_A = 0.0f;
float load_voltage = 0.0f;

bool was_charging = false;

const float DISABLE_THRESHOLD = 7.0;
const float CHARGE_CURRENT_THRESHOLD = 0.1;  // amps
Adafruit_INA219 charge_ina(0x40 + 0b01);

// ---
// LED ring
// ---

const int LED_RING = 39;
const int NUM_PIXELS = 24;
Adafruit_NeoPixel led_ring(NUM_PIXELS, LED_RING, NEO_GRBW + NEO_KHZ800);

// ---
// Timers
// ---

uint32_t current_time = 0;

uint32_t prev_command_time = 0;
uint32_t COMMAND_TIMEOUT_MS = 500;

uint32_t prev_control_time = 0;
const uint32_t CONTROL_UPDATE_INTERVAL_MS = 20;

uint32_t prev_slow_time = 0;
const uint32_t SLOW_UPDATE_INTERVAL_MS = 100;

uint32_t prev_nonglobal_time = 0;
const uint32_t NONGLOBAL_CONTROL_TIMEOUT_MS = 5000;

bool was_disabled_by_command = true;
const double MOVEMENT_EPSILON = 5E-3;
uint32_t prev_movement_time = 0;
const uint32_t MOVEMENT_DISABLE_TIMEOUT_MS = 3000;

// ---
// Sequencer
// ---

bool was_sequencer_active = false;
BwUISequencer* sequencer;


// ---
// Button functions
// ---

void set_status_led(bool state) {
    digitalWrite(STATUS_LED, state);
}

void set_builtin_led(bool state) {
    digitalWrite(BUILTIN_LED, state);
}

void set_button_led(bool state) {
    digitalWrite(BUTTON_LED, state);
}

bool read_button() {
    return !digitalRead(BUTTON_IN);
}

void write_button_state() {
    tunnel->writePacket("bu", "b", prev_button_state);
}

bool did_button_press(bool comparison_state) {
    bool state = read_button();
    set_button_led(!state);
    if (state != prev_button_state) {
        prev_button_state = state;
        write_button_state();
        return state == comparison_state;
    }
    else {
        return false;
    }
}

// ---
// Drive functions
// ---

void write_enable_state() {
    tunnel->writePacket("en", "b", drive->get_enable());
}

void set_control_mode(control_mode_t new_mode) {
    control_mode = new_mode;
    if (control_mode != CONTROL_GLOBAL) {
        prev_nonglobal_time = current_time;
    }
}

void set_motor_enable(bool enabled)
{
    if (enabled && load_voltage < DISABLE_THRESHOLD) {
        DEBUG_SERIAL.println("Enabling is blocked! Battery is too low.");
    }
    else {
        drive->set_enable(enabled);
        DEBUG_SERIAL.print("Setting enabled to ");
        DEBUG_SERIAL.println(enabled);
    }
    write_enable_state();
    if (enabled) {
        prev_movement_time = current_time;
    }
    else {
        was_disabled_by_command = true;
        sequencer->stop_sequence();
    }
    set_control_mode(CONTROL_GLOBAL);
}

void stop_motors()
{
    vx_command = 0.0;
    vy_command = 0.0;
    vt_command = 0.0;
    drive->stop();

    for (unsigned int channel = 0; channel < drive->get_num_motors(); channel++) {
        drive->get_module(channel)->get_motor()->set_frequency(MOTOR_PWM_FREQUENCY);
    }
}

bool is_moving(
    double vx_command, double vy_command, double vt_command,
    double odom_vx, double odom_vy, double odom_vt)
{
    return (
        abs(vx_command) > MOVEMENT_EPSILON ||
        abs(vy_command) > MOVEMENT_EPSILON ||
        abs(vt_command) > MOVEMENT_EPSILON ||
        abs(odom_vx) > MOVEMENT_EPSILON ||
        abs(odom_vy) > MOVEMENT_EPSILON ||
        abs(odom_vt) > MOVEMENT_EPSILON
    );
}

// ---
// Sequence functions
// ---

void report_sequence()
{
    tunnel->writePacket("|seq", "bcbg",
        sequencer->get_status() > 0,
        sequencer->get_current_sequence(),
        sequencer->is_current_from_flash(),
        sequencer->get_index()
    );
}

// ---
// Power management functions
// ---

void update_ina()
{
    shunt_voltage = charge_ina.getShuntVoltage_mV();
    bus_voltage = charge_ina.getBusVoltage_V();
    current_A = charge_ina.getCurrent_mA() / 1000.0;
    load_voltage = bus_voltage + (shunt_voltage / 1000);
}

// ---
// Packet callback
// ---

void packetCallback(PacketResult* result)
{
    // if the result is not set for some reason, don't do anything
    if (result == NULL) {
        return;
    }

    // Extract category and check which event it maps to
    String category = result->getCategory();
    // DEBUG_SERIAL.print("Received packet: ");
    // DEBUG_SERIAL.println(category);
    if (category.equals("ping")) {
        // Respond to ping by writing back the same value
        float value;
        if (!result->getFloat(value)) { DEBUG_SERIAL.println(F("Failed to get ping")); return; }
        tunnel->writePacket("ping", "f", value);
        DEBUG_SERIAL.print("Received ping: ");
        DEBUG_SERIAL.println(value);
    }
    else if (category.equals("d")) {
        float vx, vy, vt;
        if (!result->getFloat(vx)) { DEBUG_SERIAL.println(F("Failed to get vx")); return; }
        if (!result->getFloat(vy)) { DEBUG_SERIAL.println(F("Failed to get vy")); return; }
        if (!result->getFloat(vt)) { DEBUG_SERIAL.println(F("Failed to get vt")); return; }

        vx_command = (double)vx;
        vy_command = (double)vy;
        vt_command = (double)vt;
        prev_command_time = current_time;
    }
    else if (category.equals("en")) {
        bool enabled;
        if (!result->getBool(enabled)) { DEBUG_SERIAL.println(F("Failed to get enable state")); return; }
        set_motor_enable(enabled);
    }
    else if (category.equals("?en")) {
        tunnel->writePacket("en", "b", drive->get_enable());
    }
    else if (category.equals("mo")) {
        uint8_t channel;
        float azimuth_position;
        double wheel_position;
        float wheel_velocity;
        if (!result->getUInt8(channel)) { DEBUG_SERIAL.println(F("Failed to get channel")); return; }
        if (!result->getFloat(azimuth_position)) { DEBUG_SERIAL.println(F("Failed to get azimuth_position")); return; }
        if (!result->getDouble(wheel_position)) { DEBUG_SERIAL.println(F("Failed to get wheel_position")); return; }
        if (!result->getFloat(wheel_velocity)) { DEBUG_SERIAL.println(F("Failed to get wheel_velocity")); return; }

        set_control_mode(CONTROL_INDIVIDUAL);

        drive->set(channel, azimuth_position, wheel_velocity);
    }
    else if (category.equals("seq")) {
        uint8_t serial;
        uint16_t index;
        uint64_t parameters;
        if (!result->getUInt8(serial)) { DEBUG_SERIAL.println(F("Failed to get serial")); return; }
        if (!result->getUInt16(index)) { DEBUG_SERIAL.println(F("Failed to get index")); return; }
        if (!result->getUInt64(parameters)) { DEBUG_SERIAL.println(F("Failed to get parameters")); return; }
        bool success = sequencer->set_element(serial, index, parameters);
        tunnel->writePacket("seq", "b", success);

        if (success) {
            DEBUG_SERIAL.print(index);
            DEBUG_SERIAL.print(" set for sequence ");
            DEBUG_SERIAL.print(serial);
            DEBUG_SERIAL.print(". parameters=");
            DEBUG_SERIAL.println(parameters);
        }
        else {
            DEBUG_SERIAL.println(F("Failed to set sequence element"));
        }
    }
    else if (category.equals("lseq")) {
        uint8_t serial;
        uint16_t length;
        if (!result->getUInt8(serial)) { DEBUG_SERIAL.println(F("Failed to get serial")); return; }
        if (!result->getUInt16(length)) { DEBUG_SERIAL.println(F("Failed to get length")); return; }
        sequencer->allocate_sequence(serial, length);
        tunnel->writePacket("lseq", "b", true);
        DEBUG_SERIAL.print(length);
        DEBUG_SERIAL.print(" bits allocated for sequence ");
        DEBUG_SERIAL.println(serial);
    }
    else if (category.equals(">seq")) {
        uint8_t serial;
        bool should_loop, from_flash;
        if (!result->getUInt8(serial)) { DEBUG_SERIAL.println(F("Failed to get serial")); return; }
        if (!result->getBool(should_loop)) { DEBUG_SERIAL.println(F("Failed to get should_loop")); return; }
        if (!result->getBool(from_flash)) { DEBUG_SERIAL.println(F("Failed to get from_flash")); return; }
        bool success = sequencer->play_sequence(serial, should_loop, from_flash);
        tunnel->writePacket(">seq", "b", success);
        if (success) {
            DEBUG_SERIAL.print("Starting sequence ");
            DEBUG_SERIAL.print(serial);
            DEBUG_SERIAL.print(", should_loop=");
            DEBUG_SERIAL.print(should_loop);
            DEBUG_SERIAL.print(", from_flash=");
            DEBUG_SERIAL.println(from_flash);
            set_control_mode(CONTROL_TONE);
            set_motor_enable(true);
        }
        else {
            DEBUG_SERIAL.println(F("Failed to start sequence"));
        }
    }
    else if (category.equals("xseq")) {
        sequencer->stop_sequence();
        tunnel->writePacket("xseq", "b", true);
        DEBUG_SERIAL.println("Stopping current sequence");
    }
}

// ---
// Setup
// ---

void setup()
{
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    DEBUG_SERIAL.println("setup");

    PROTOCOL_SERIAL.begin(PROTOCOL_BAUD);

    tunnel = new TunnelSerial(&DEBUG_SERIAL, &PROTOCOL_SERIAL);

    drive = new BwDriveTrain(
        servos, motors, encoders,
        NUM_CHANNELS, MOTOR_EN,
        OUTPUT_RATIO, ARMATURE,
        MIN_RADIUS_OF_CURVATURE,
        MODULE_X_LOCATIONS, MODULE_Y_LOCATIONS
    );
    sequencer = new BwUISequencer(drive, &led_ring, NUM_PIXELS);
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);

    servos->begin();
    servos->setOscillatorFrequency(SERVO_OSC_FREQ);
    servos->setPWMFreq(SERVO_FREQ);

    charge_ina.begin();
    update_ina();

    led_ring.begin();

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    set_status_led(true);
    set_builtin_led(true);
    set_button_led(true);

    for (unsigned int channel = 0; channel < drive->get_num_motors(); channel++) {
        SpeedPID* pid = drive->get_pid(channel);
        pid->Kp = 100.0;
        pid->Ki = 0.001;
        pid->Kd = 0.01;
        pid->K_ff = SPEED_TO_COMMAND;
        pid->deadzone_command = DEADZONE_COMMAND;
        pid->standstill_deadzone_command = STANDSTILL_DEADZONE_COMMAND;
        pid->error_sum_clamp = 100.0;
        pid->command_min = -MAX_SPEED_COMMAND;
        pid->command_max = MAX_SPEED_COMMAND;
        SpeedFilter* filter = drive->get_filter(channel);
        filter->Kf = 0.99;
    }
    
    drive->set_limits(
        FRONT_LEFT,
        FRONT_ANGLE,  // -75 deg
        ALCOVE_ANGLE,  // 30 deg
        STRAIGHT_ANGLE,
        ALCOVE_ANGLE,
        FRONT_LEFT_ALCOVE,
        FRONT_LEFT_STRAIGHT,
        MAX_SERVO_SPEED,
        WHEEL_RADIUS,
        true
    );
    drive->set_limits(
        BACK_LEFT,
        M_PI - ALCOVE_ANGLE,  // 150 deg
        M_PI - FRONT_ANGLE,  // 225 deg
        STRAIGHT_ANGLE + M_PI,
        M_PI - ALCOVE_ANGLE,
        BACK_LEFT_ALCOVE,
        BACK_LEFT_STRAIGHT,
        MAX_SERVO_SPEED,
        WHEEL_RADIUS,
        false
    );
    drive->set_limits(
        BACK_RIGHT,
        FRONT_ANGLE + M_PI,  // 105 deg
        ALCOVE_ANGLE + M_PI,  // 210 deg
        STRAIGHT_ANGLE + M_PI,
        ALCOVE_ANGLE + M_PI,
        BACK_RIGHT_ALCOVE,
        BACK_RIGHT_STRAIGHT,
        MAX_SERVO_SPEED,
        WHEEL_RADIUS,
        true
    );
    
    drive->set_limits(
        FRONT_RIGHT,
        -ALCOVE_ANGLE,  // -30 deg
        -FRONT_ANGLE,  // 75 deg
        STRAIGHT_ANGLE,
        -ALCOVE_ANGLE,
        FRONT_RIGHT_ALCOVE,
        FRONT_RIGHT_STRAIGHT,
        MAX_SERVO_SPEED,
        WHEEL_RADIUS,
        false
    );

    drive->begin();
    drive->set_enable(false);

    if (!sequencer->play_sequence(STORED_SEQUENCE_STARTUP, false, true)) {
        for(int i = 0; i < NUM_PIXELS; i++) {
            led_ring.setPixelColor(i, led_ring.Color(0, 150, 0, 0));
            led_ring.show();
            delay(10);
        }
        for(int i = 0; i < NUM_PIXELS; i++) {
            led_ring.setPixelColor(i, led_ring.Color(0, 0, 0, 0));
            led_ring.show();
            delay(10);
        }
    }

    DEBUG_SERIAL.println("setup complete");
}

// ---
// Loop
// ---

void loop()
{
    current_time = millis();
    packetCallback(tunnel->readPacket());
    if (did_button_press(true)) {
        set_motor_enable(!drive->get_enable());
        stop_motors();
        drive->drive(vx_command, vy_command, vt_command);
    }

    update_ina();

    bool is_charging = current_A > CHARGE_CURRENT_THRESHOLD;
    if (was_charging != is_charging) {
        was_charging = is_charging;
        if (current_A > CHARGE_CURRENT_THRESHOLD) {
            DEBUG_SERIAL.println("Charging");
        }
        else {
            DEBUG_SERIAL.println("Unplugged");
        }
    }

    if (load_voltage < DISABLE_THRESHOLD) {
        drive->set_enable(false);
    }
    
    if (control_mode != CONTROL_GLOBAL && current_time - prev_nonglobal_time > NONGLOBAL_CONTROL_TIMEOUT_MS) {
        set_control_mode(CONTROL_GLOBAL);
        stop_motors();
        Serial.println("Switching to global control mode");
        sequencer->stop_sequence();
        report_sequence();
    }

    int sequencer_state = sequencer->update();
    if (sequencer_state == 2) {
        set_control_mode(CONTROL_TONE);
    }
    bool is_sequence_running = sequencer_state > 0;
    if (was_sequencer_active != is_sequence_running) {
        report_sequence();
        was_sequencer_active = is_sequence_running;
        if (is_sequence_running) {
            Serial.println("Sequence started");
        }
        else {
            Serial.println("Sequence stopped");
            set_control_mode(CONTROL_GLOBAL);
            stop_motors();
        }
    }

    if (current_time - prev_control_time > CONTROL_UPDATE_INTERVAL_MS) {
        prev_control_time = current_time;

        if (control_mode == CONTROL_GLOBAL) {
            if (current_time - prev_command_time > COMMAND_TIMEOUT_MS) {
                stop_motors();
            }
            else {
                drive->drive(vx_command, vy_command, vt_command);
            }

            if (is_moving(vx_command, vy_command, vt_command, odom_vx, odom_vy, odom_vt)) {
                prev_movement_time = current_time;
                if (!drive->get_enable() && !was_disabled_by_command) {
                    DEBUG_SERIAL.println("Re-enabling because robot started moving");
                    set_motor_enable(true);
                }
            }
            else if (drive->get_enable() && 
                    current_time - prev_movement_time > MOVEMENT_DISABLE_TIMEOUT_MS) {
                DEBUG_SERIAL.println("Disabling because robot isn't moving");
                set_motor_enable(false);
                was_disabled_by_command = false;
            }
        }
        else if (control_mode == CONTROL_TONE) {
            report_sequence();
        }

        drive->get_velocity(odom_vx, odom_vy, odom_vt);
        drive->get_position(odom_x, odom_y, odom_t, odom_vx, odom_vy, odom_vt);
        tunnel->writePacket("od", "eeefff",
            odom_x, odom_y, odom_t,
            odom_vx, odom_vy, odom_vt
        );

        for (unsigned int channel = 0; channel < drive->get_num_motors(); channel++) {
            tunnel->writePacket(
                "mo",
                "cfef",
                channel,
                drive->get_azimuth(channel),
                drive->get_wheel_position(channel),
                drive->get_wheel_velocity(channel)
            );
        }
    }
    if (current_time - prev_slow_time > SLOW_UPDATE_INTERVAL_MS) {
        prev_slow_time = current_time;

        write_button_state();
        write_enable_state();
        tunnel->writePacket("power", "ffb",
            load_voltage,
            current_A,
            is_charging
        );
    }
}
