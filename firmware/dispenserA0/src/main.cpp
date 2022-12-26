#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <PID.h>

#ifndef WIFI_SSID
#error WIFI_SSID is not defined
#endif

#ifndef WIFI_PASSWORD
#error WIFI_PASSWORD is not defined
#endif

#ifndef MQTT_SERVER
#error MQTT_SERVER is not defined
#endif

#define XSTR(x) STR(x)
#define STR(x) #x

#pragma message "ssid: " XSTR(WIFI_SSID)
#pragma message "mqtt server: " XSTR(MQTT_SERVER)

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const int MOTOR1_ENCA = 12;
const int MOTOR1_ENCB = 14;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1;

// const double GEAR_RATIO = 18.75;
const double GEAR_RATIO = 18.3;
const double ENCODER_PPR = 64.0;  // pulses per rotation
const double OUTPUT_RATIO = 360.0 / (GEAR_RATIO * ENCODER_PPR);  // encoder counts (pulses) * output_ratio = degrees at shaft

const int DEADZONE_COMMAND = 0;  // 60
const int MAX_SPEED_COMMAND = 100;

PID* pid;
const double RAMP_K = 0.05;  // ramp up damping constant
const double ERROR_TOLERANCE = 5.0;  // degrees

double prev_command = 0.0;
double prev_measurement = 0.0;
bool goal_reached = false;

const int BUTTON = 13;

volatile uint8_t enc1_state = 0;
volatile int32_t enc1_position = 0;

#define UPDATE_ENC1_STATE() \
    uint8_t new_state = enc1_state & 3; \
    if (digitalRead(MOTOR1_ENCA)) new_state |= 4; \
    if (digitalRead(MOTOR1_ENCB)) new_state |= 8; \
    switch (new_state) { \
        case 0: case 5: case 10: case 15: break; \
        case 1: case 7: case 8: case 14: enc1_position++; break; \
        case 2: case 4: case 11: case 13: enc1_position--; break; \
        case 3: case 12: enc1_position += 2; break; \
        default: enc1_position -= 2; break; \
    } \
    enc1_state = new_state >> 2;

IRAM_ATTR void channel_a_callback() {
    UPDATE_ENC1_STATE()
}

IRAM_ATTR void channel_b_callback() {
    UPDATE_ENC1_STATE()
}

void dispense() {
    goal_reached = false;
    pid->reset();
    // if (pid->get_target() > 0.0) {
    //     pid->set_target(0.0);
    // }
    // else {
    //     pid->set_target(360.0);
    // }
    pid->set_target(pid->get_target() + 360.0);
}

void set_speed(int speed) {
    if (speed > 0) {
        motor1->run(FORWARD);
    }
    else if (speed < 0) {
        motor1->run(BACKWARD);
    }
    else {
        motor1->run(RELEASE);
    }
    motor1->setSpeed(abs(speed));
}

double get_position() {
    return OUTPUT_RATIO * enc1_position;
}


void setup_wifi() {
    motor1 = AFMS.getMotor(1);

    pid = new PID();
    pid->Kp = 3.0;
    pid->Ki = 1.5;
    pid->Kd = 0.1;
    pid->K_ff = 0.0;
    pid->deadzone_command = (double)DEADZONE_COMMAND;
    pid->error_sum_clamp = 50000.0;
    pid->command_min = (double)(-MAX_SPEED_COMMAND);
    pid->command_max = (double)(MAX_SPEED_COMMAND);

    pinMode(MOTOR1_ENCA, INPUT_PULLUP);
    pinMode(MOTOR1_ENCB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCA), channel_a_callback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCB), channel_b_callback, CHANGE);

    pinMode(BUTTON, INPUT_PULLUP);

    Serial.begin(115200);
    delay(100);

    unsigned int blink = 0;
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (true) {
            digitalWrite(LED_BUILTIN, blink % 2 == 0);
            blink++;
            delay(50);
        }
    }

    set_speed(0);

    // Serial.print("Connecting to ");
    // Serial.println(ssid);
    // WiFi.begin(ssid, password);

    // while (WiFi.status() != WL_CONNECTED) {
    //     delay(500);
    //     digitalWrite(LED_BUILTIN, blink % 2 == 0);
    //     Serial.print(".");
    //     blink++;
    // }
    for (blink = 0; blink < 16; blink++) {
        digitalWrite(LED_BUILTIN, blink % 2 == 0);
        delay(50);
    }
    digitalWrite(LED_BUILTIN, LOW);

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
        digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
        // but actually the LED is on; this is because
        // it is active low on the ESP-01)
    }
    else {
        digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    }
}

void reconnect() {
  // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish("outTopic", "hello world");
            // ... and resubscribe
            client.subscribe("inTopic");
            digitalWrite(LED_BUILTIN, LOW);
        }
        else {
            for (unsigned int blink = 0; blink < 6; blink++) {
                digitalWrite(LED_BUILTIN, blink % 2 == 0);
                delay(50);
            }
            digitalWrite(LED_BUILTIN, LOW);
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 1 second");
            // Wait 1 second before retrying
            delay(1000);
        }
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    double measurement = get_position();
    double command = pid->compute(measurement);
    double error = pid->get_target() - measurement;
    if (goal_reached) {
        set_speed(0);
        prev_command = 0;
    }
    else {
        if (abs(error) < ERROR_TOLERANCE && !goal_reached) {
            set_speed(0);
            goal_reached = true;
            prev_command = 0;
        }
        else {
            prev_command += RAMP_K * (command - prev_command);
            set_speed((int)prev_command);
        }
    }
    Serial.print(measurement);
    Serial.print('\t');
    Serial.print(pid->get_target());
    Serial.print('\t');
    Serial.println(enc1_position);

    if (Serial.available()) {
        char c = Serial.read();
        switch (c)
        {
        case 's':
            dispense();
            break;
        case 'p':
            pid->Kp = (double)Serial.parseFloat();
            Serial.print("Kp: ");
            Serial.println(pid->Kp);
            break;
        case 'i':
            pid->Ki = (double)Serial.parseFloat();
            Serial.print("Ki: ");
            Serial.println(pid->Ki);
            break;
        case 'd':
            pid->Kd = (double)Serial.parseFloat();
            Serial.print("Kd: ");
            Serial.println(pid->Kd);
            break;
        default:
            break;
        }
    }
    // if (!client.connected()) {
    //     reconnect();
    // }
    // client.loop();

    // unsigned long now = millis();
    // if (now - lastMsg > 2000) {
    //     lastMsg = now;
    //     ++value;
    //     snprintf (msg, MSG_BUFFER_SIZE, "hello world #%d", value);
    //     Serial.print("Publish message: ");
    //     Serial.println(msg);
    //     client.publish("outTopic", msg);
    // }
}
