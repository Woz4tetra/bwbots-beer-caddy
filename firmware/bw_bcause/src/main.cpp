#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_BUS_1 Wire
// #define I2C_BUS_2 Wire1


const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int USMIN = 600; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
const int USMAX = 2400; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40, I2C_BUS_1);


const int STATUS_LED = 32;
const int MOTOR_ENABLE = 33;
const int USB_SENSE = 34;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;
const int UNLATCH_PIN = 30;
const int LATCH_STAT_PIN = 31;
const int CHARGE_RELAY_CTRL = 29;

const int M1_DR1 = 38;
const int M1_DR2 = 37;
const int M2_DR1 = 36;
const int M2_DR2 = 35;
const int M3_DR1 = 23;
const int M3_DR2 = 22;
const int M4_DR1 = 21;
const int M4_DR2 = 20;


void setM1(int speed) {
    if (speed > 0) {
        digitalWrite(M1_DR1, HIGH);
        digitalWrite(M1_DR2, LOW);
    }
    else if (speed < 0) {
        digitalWrite(M1_DR1, LOW);
        digitalWrite(M1_DR2, HIGH);
    }
    else {
        digitalWrite(M1_DR1, LOW);
        digitalWrite(M1_DR2, LOW);
    }
}


void setM2(int speed) {
    if (speed > 0) {
        digitalWrite(M2_DR1, HIGH);
        digitalWrite(M2_DR2, LOW);
    }
    else if (speed < 0) {
        digitalWrite(M2_DR1, LOW);
        digitalWrite(M2_DR2, HIGH);
    }
    else {
        digitalWrite(M2_DR1, LOW);
        digitalWrite(M2_DR2, LOW);
    }
}


void setM3(int speed) {
    if (speed > 0) {
        digitalWrite(M3_DR1, HIGH);
        digitalWrite(M3_DR2, LOW);
    }
    else if (speed < 0) {
        digitalWrite(M3_DR1, LOW);
        digitalWrite(M3_DR2, HIGH);
    }
    else {
        digitalWrite(M3_DR1, LOW);
        digitalWrite(M3_DR2, LOW);
    }
}



void setM4(int speed) {
    if (speed > 0) {
        digitalWrite(M4_DR1, HIGH);
        digitalWrite(M4_DR2, LOW);
    }
    else if (speed < 0) {
        digitalWrite(M4_DR1, LOW);
        digitalWrite(M4_DR2, HIGH);
    }
    else {
        digitalWrite(M4_DR1, LOW);
        digitalWrite(M4_DR2, LOW);
    }
}

void pointM1(int direction)
{
    servos.setPWM(1, 0, direction);  // M1
}


void pointM2(int direction)
{
    servos.setPWM(0, 0, direction);  // M2
}


void pointM3(int direction)
{
    servos.setPWM(2, 0, direction);  // M3
}


void pointM4(int direction)
{
    servos.setPWM(3, 0, direction);  // M4
}

void driveForward(int speed)
{
    pointM1(420);
    pointM2(250);
    pointM3(420);
    pointM4(250);
    delay(250);
    setM1(-speed);
    setM2(speed);
    setM3(speed);
    setM4(-speed);
}

void driveSideways(int speed)
{
    pointM1(170);
    pointM2(510);
    pointM3(180);
    pointM4(520);
    delay(250);
    setM1(speed);
    setM2(speed);
    setM3(-speed);
    setM4(-speed);
}

void setup()
{
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);

    servos.begin();
    servos.setOscillatorFrequency(27000000);
    servos.setPWMFreq(SERVO_FREQ);

    pinMode(BUTTON_LED, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);
    pinMode(UNLATCH_PIN, OUTPUT);
    pinMode(CHARGE_RELAY_CTRL, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(LATCH_STAT_PIN, INPUT_PULLUP);

    pinMode(M1_DR1, OUTPUT);
    pinMode(M1_DR2, OUTPUT);
    pinMode(M2_DR1, OUTPUT);
    pinMode(M2_DR2, OUTPUT);
    pinMode(M3_DR1, OUTPUT);
    pinMode(M3_DR2, OUTPUT);
    pinMode(M4_DR1, OUTPUT);
    pinMode(M4_DR2, OUTPUT);

    digitalWrite(UNLATCH_PIN, LOW);

    for (int count = 0; count < 5; count++)
    {
        digitalWrite(BUTTON_LED, LOW);
        digitalWrite(STATUS_LED, LOW);
        digitalWrite(BUILTIN_LED, LOW);
        delay(100);
        digitalWrite(BUTTON_LED, HIGH);
        digitalWrite(STATUS_LED, HIGH);
        digitalWrite(BUILTIN_LED, HIGH);
        delay(100);
    }
    digitalWrite(CHARGE_RELAY_CTRL, HIGH);
    digitalWrite(MOTOR_ENABLE, LOW);

    setM1(0);
    setM2(0);
    setM3(0);
    setM4(0);
    
    // driveSideways(255);
    
}

void loop()
{
    driveForward(255);
    // delay(500);

    // driveForward(0);
    // delay(500);

    // driveSideways(255);
    // delay(500);

    // driveSideways(0);
    // delay(500);
    
    // driveForward(-255);
    // delay(500);

    // driveForward(0);
    // delay(500);

    // driveSideways(-255);
    // delay(500);

    // driveSideways(0);
    // delay(500);

    // setM1(-255);
    // setM2(255);
    // setM3(255);
    // setM4(-255);
    // delay(1000);
    // setM1(255);
    // setM2(-255);
    // setM3(-255);
    // setM4(255);
    // delay(1000);
    // setM1(0);
    // setM2(0);
    // setM3(0);
    // setM4(0);
    // delay(1000);

    // setM1(-255);
    // delay(1000);
    // setM1(0);
    // setM2(255);
    // delay(1000);
    // setM2(0);
    // setM3(255);
    // delay(1000);
    // setM3(0);
    // setM4(-255);
    // delay(1000);
    // setM4(0);
    // delay(2000);

    // pointM1(250);
    // delay(500);
    // pointM1(450);
    // delay(500);

    // pointM2(250);
    // delay(500);
    // pointM2(450);
    // delay(500);

    // pointM3(250);
    // delay(500);
    // pointM3(450);
    // delay(500);

    // pointM4(250);
    // delay(500);
    // pointM4(450);
    // delay(3000);
}
