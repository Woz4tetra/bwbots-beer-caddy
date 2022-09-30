#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <MotorControllerMC33926.h>
#include <BwDriveTrain.h>
#include "tunnel/serial.h"

#define I2C_BUS_1 Wire


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

// ---
// Drive controllers
// ---
const int MOTOR_EN = 1;
const int NUM_CHANNELS = 4;

const int M1_SPEED = 9;
const int M1_DIR_P = 11;
const int M1_DIR_N = 12;
const int M1_SF = 32;
const int M1_FB = 23;

const int M2_SPEED = 10;
const int M2_DIR_P = 28;
const int M2_DIR_N = 29;
const int M2_SF = 17;
const int M2_FB = 22;

const int M3_SPEED = 37;
const int M3_DIR_P = 30;
const int M3_DIR_N = 31;
const int M3_SF = 38;
const int M3_FB = 21;

const int M4_SPEED = 36;
const int M4_DIR_P = 35;
const int M4_DIR_N = 34;
const int M4_SF = 33;
const int M4_FB = 20;

MotorControllerMC33926* motors[NUM_CHANNELS] = {
    new MotorControllerMC33926(M1_SPEED, M1_DIR_P, M1_DIR_N, M1_SF, M1_FB),
    new MotorControllerMC33926(M2_SPEED, M2_DIR_P, M2_DIR_N, M2_SF, M2_FB),
    new MotorControllerMC33926(M3_SPEED, M3_DIR_P, M3_DIR_N, M3_SF, M3_FB),
    new MotorControllerMC33926(M4_SPEED, M4_DIR_P, M4_DIR_N, M4_SF, M4_FB)
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
const double SPEED_TO_COMMAND = 255.0 / 1.0;  // calculated max speed: 0.843 m/s
const double MAX_SERVO_SPEED = 5.950;  // rad/s

const int DEADZONE_COMMAND = 20;
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

BwDriveTrain drive(
    servos, motors, encoders,
    NUM_CHANNELS, MOTOR_EN,
    OUTPUT_RATIO, ARMATURE,
    MIN_RADIUS_OF_CURVATURE,
    MODULE_X_LOCATIONS, MODULE_Y_LOCATIONS
);
double vx_command = 0.0, vy_command = 0.0, vt_command = 0.0;
double odom_vx = 0.0, odom_vy = 0.0, odom_vt = 0.0;
double odom_x = 0.0, odom_y = 0.0, odom_t = 0.0;

// ---
// Power management
// ---

float shunt_voltage = 0.0f;
float bus_voltage = 0.0f;
float current_mA = 0.0f;
float load_voltage = 0.0f;

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

uint32_t prev_power_time = 0;
const uint32_t POWER_UPDATE_INTERVAL_MS = 500;

bool read_button() {
    return !digitalRead(BUTTON_IN);
}

bool prev_button_state = false;
bool did_button_press(bool comparison_state) {
    bool state = read_button();
    if (state != prev_button_state) {
        prev_button_state = state;
        return state == comparison_state;
    }
    else {
        return false;
    }
}

void set_status_led(bool state) {
    digitalWrite(STATUS_LED, state);
}

void set_builtin_led(bool state) {
    digitalWrite(BUILTIN_LED, state);
}

void set_button_led(bool state) {
    digitalWrite(BUTTON_LED, state);
}

void setup()
{
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);

    servos->begin();
    servos->setOscillatorFrequency(SERVO_OSC_FREQ);
    servos->setPWMFreq(SERVO_FREQ);

    charge_ina.begin();

    led_ring.begin();

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    set_status_led(true);
    set_builtin_led(true);
    set_button_led(true);

    for (unsigned int channel = 0; channel < drive.get_num_motors(); channel++) {
        SpeedPID* pid = drive.get_pid(channel);
        pid->Kp = 3.0;
        pid->Ki = 0.001;
        pid->Kd = 0.001;
        pid->K_ff = SPEED_TO_COMMAND;
        pid->deadzone_command = DEADZONE_COMMAND;
        pid->error_sum_clamp = 10.0;
        pid->command_min = -MAX_SPEED_COMMAND;
        pid->command_max = MAX_SPEED_COMMAND;
        SpeedFilter* filter = drive.get_filter(channel);
        filter->Kf = 0.9;
    }
    
    drive.set_limits(
        FRONT_LEFT,
        FRONT_ANGLE,  // -75 deg
        ALCOVE_ANGLE,  // 30 deg
        STRAIGHT_ANGLE,
        ALCOVE_ANGLE,
        FRONT_LEFT_ALCOVE,
        FRONT_LEFT_STRAIGHT,
        MAX_SERVO_SPEED,
        true
    );
    drive.set_limits(
        BACK_LEFT,
        M_PI - ALCOVE_ANGLE,  // 150 deg
        M_PI - FRONT_ANGLE,  // 225 deg
        STRAIGHT_ANGLE + M_PI,
        M_PI - ALCOVE_ANGLE,
        BACK_LEFT_ALCOVE,
        BACK_LEFT_STRAIGHT,
        MAX_SERVO_SPEED,
        false
    );
    drive.set_limits(
        BACK_RIGHT,
        FRONT_ANGLE + M_PI,  // 105 deg
        ALCOVE_ANGLE + M_PI,  // 210 deg
        STRAIGHT_ANGLE + M_PI,
        ALCOVE_ANGLE + M_PI,
        BACK_RIGHT_ALCOVE,
        BACK_RIGHT_STRAIGHT,
        MAX_SERVO_SPEED,
        true
    );
    
    drive.set_limits(
        FRONT_RIGHT,
        -ALCOVE_ANGLE,  // 30 deg
        -FRONT_ANGLE,  // 75 deg
        STRAIGHT_ANGLE,
        -ALCOVE_ANGLE,
        FRONT_RIGHT_ALCOVE,
        FRONT_RIGHT_STRAIGHT,
        MAX_SERVO_SPEED,
        false
    );

    drive.begin();
    drive.set_enable(false);

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

    tunnel_begin();
}

void packetCallback(PacketResult* result)
{
    // if the result is not set for some reason, don't do anything
    if (result == NULL) {
        return;
    }

    // Extract category and check which event it maps to
    String category = result->getCategory();
    // Serial.print("Received packet: ");
    // Serial.println(category);
    if (category.equals("ping")) {
        // Respond to ping by writing back the same value
        float value;
        if (!result->getFloat(value)) { DEBUG_SERIAL.println(F("Failed to get ping")); return; }
        tunnel_writePacket("ping", "f", value);
        Serial.print("Received ping: ");
        Serial.println(value);
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
        drive.set_enable(enabled);
        DEBUG_SERIAL.print("Setting enabled to ");
        DEBUG_SERIAL.println(enabled);
    }
    else if (category.equals("?en")) {
        tunnel_writePacket("?en", "b", drive.get_enable());
    }
}

void loop()
{
    current_time = millis();
    packetCallback(tunnel_readPacket());
    if (did_button_press(true)) {
        drive.set_enable(!drive.get_enable());
    }
    
    if (current_time - prev_control_time > CONTROL_UPDATE_INTERVAL_MS) {
        prev_control_time = current_time;
        if (current_time - prev_command_time > COMMAND_TIMEOUT_MS) {
            vx_command = 0.0;
            vy_command = 0.0;
            vt_command = 0.0;
            drive.stop();
        }
        else {
            drive.drive(vx_command, vy_command, vt_command);
        }
        
        drive.get_position(odom_x, odom_y, odom_t);
        drive.get_velocity(odom_vx, odom_vy, odom_vt);
        tunnel_writePacket("od", "eeefff",
            odom_x, odom_y, odom_t,
            odom_vx, odom_vy, odom_vt
        );
        tunnel_writePacket("ms", "c", drive.get_num_motors());
        for (unsigned int channel = 0; channel < drive.get_num_motors(); channel++) {
            // Serial.print(drive.get_wheel_velocity(channel));
            // Serial.print('\t');
            tunnel_writePacket(
                "mo",
                "cfef",
                channel,
                drive.get_azimuth(channel),
                drive.get_wheel_position(channel),
                drive.get_wheel_velocity(channel)
            );
        }
        // Serial.print('\n');
    }
    if (current_time - prev_power_time > POWER_UPDATE_INTERVAL_MS) {
        prev_power_time = current_time;

        shunt_voltage = charge_ina.getShuntVoltage_mV();
        bus_voltage = charge_ina.getBusVoltage_V();
        current_mA = charge_ina.getCurrent_mA();
        load_voltage = bus_voltage + (shunt_voltage / 1000);

        tunnel_writePacket("power", "ff",
            load_voltage,
            current_mA
        );
    }
}
