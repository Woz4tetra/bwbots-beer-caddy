
#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
    Serial2.begin(1000000);
}

void loop()
{
    if (Serial.available()) {
        char c = Serial.read();
        Serial.write(c);
        Serial2.write(c);
    }
    if (Serial2.available()) {
        Serial2.write(Serial2.read());
    }
    // Serial2.println(millis());
    // Serial.println(Serial2.readStringUntil('\n'));
    // delay(500);
}
