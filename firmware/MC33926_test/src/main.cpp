#include <Arduino.h>

const int MOTOR_EN = 0;

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

void set_m1(int speed)
{
    if (speed == 0) {
        digitalWrite(M1_DIR_P, LOW);
        digitalWrite(M1_DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(M1_DIR_P, HIGH);
        digitalWrite(M1_DIR_N, LOW);
    }
    else {
        digitalWrite(M1_DIR_P, LOW);
        digitalWrite(M1_DIR_N, HIGH);
    }
    analogWrite(M1_SPEED, abs(speed));
    Serial.print("M1 SF: ");
    Serial.println(digitalRead(M1_SF));
    Serial.print("M1 M1_FB: ");
    Serial.println(analogRead(M1_FB));
    Serial.print("M1 speed: ");
    Serial.println(speed);
}


void set_m2(int speed)
{
    if (speed == 0) {
        digitalWrite(M2_DIR_P, LOW);
        digitalWrite(M2_DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(M2_DIR_P, HIGH);
        digitalWrite(M2_DIR_N, LOW);
    }
    else {
        digitalWrite(M2_DIR_P, LOW);
        digitalWrite(M2_DIR_N, HIGH);
    }
    analogWrite(M2_SPEED, abs(speed));
    Serial.print("M2 SF: ");
    Serial.println(digitalRead(M2_SF));
    Serial.print("M2 M1_FB: ");
    Serial.println(analogRead(M2_FB));
    Serial.print("M2 speed: ");
    Serial.println(speed);
}

void setup()
{
    Serial.begin(9600);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(M1_SPEED, OUTPUT);
    pinMode(M1_DIR_P, OUTPUT);
    pinMode(M1_DIR_N, OUTPUT);
    pinMode(M1_SF, INPUT);
    pinMode(M1_FB, INPUT);

    pinMode(M2_SPEED, OUTPUT);
    pinMode(M2_DIR_P, OUTPUT);
    pinMode(M2_DIR_N, OUTPUT);
    pinMode(M2_SF, INPUT);
    pinMode(M2_FB, INPUT);

    digitalWrite(MOTOR_EN, HIGH);

}

void loop()
{
    for (int value = 0; value < 255; value++) {
        set_m1(value);
        set_m2(value);
        delay(5);
    }

    for (int value = 255; value >= -255; value--) {
        set_m1(value);
        set_m2(value);
        delay(5);
    }

    for (int value = -255; value < 0; value++) {
        set_m1(value);
        set_m2(value);
        delay(5);
    }
}
