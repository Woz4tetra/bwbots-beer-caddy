#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MotorShield.h>

#ifndef WIFI_SSID
#error WIFI_SSID is not defined
#endif

#ifndef WIFI_PASSWORD
#error WIFI_PASSWORD is not defined
#endif

#ifndef MQTT_SERVER
#error MQTT_SERVER is not defined
#endif

#ifndef DEVICE_NAME
#error DEVICE_NAME is not defined
#endif

#define XSTR(x) STR(x)
#define STR(x) #x

#pragma message "ssid: " XSTR(WIFI_SSID)
#pragma message "mqtt server: " XSTR(MQTT_SERVER)
#pragma message "device name: " XSTR(DEVICE_NAME)

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
const char* device_name = DEVICE_NAME;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long prev_publish_time = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const int LIMIT_SWITCH_PIN = 12;

const int DISPENSE_SPEED = 255;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *dispense_motor = AFMS.getMotor(1);


void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void start_dispense();
void stop_dispense();
void check_dispense();
bool read_limit_switch();
bool get_limit_switch_state();
bool get_switch_falling_edge();
void publish_dispense_status(bool is_dispensing);
void publish_dispense_done(bool success);
void reconnect();


void setup_wifi() {
    
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    int blink = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        digitalWrite(LED_BUILTIN, blink % 2 == 0);
        Serial.print(".");
        blink++;
    }
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

    if (strcmp(topic, "start_dispense") == 0) {
        memcpy(msg, payload, length + 1);
        msg[length] = 0;
        if (strcmp(msg, device_name) == 0) {
            Serial.println("Starting dispense");
            start_dispense();
        }
    }
}

bool is_dispensing = false;
uint32_t dispense_start_time = 0;
const uint32_t min_dispense_time = 100;
const uint32_t max_dispense_time = 5000;


void set_dispense_motor_speed(int speed) {
    Serial.print("Set motor speed to ");
    Serial.println(speed);
    if (speed > 0) {
        dispense_motor->run(FORWARD);
    }
    else {
        dispense_motor->run(BACKWARD);
    }
    dispense_motor->setSpeed(abs(speed));
}

void start_dispense() {
    Serial.println("Starting dispense");
    is_dispensing = true;
    set_dispense_motor_speed(DISPENSE_SPEED);
    dispense_start_time = millis();
}

void stop_dispense() {
    Serial.println("Stopping dispense");
    is_dispensing = false;
    set_dispense_motor_speed(0);
    delay(250);
    dispense_motor->run(RELEASE);
}

void check_dispense() {
    if (!is_dispensing) {
        return;
    }
    uint32_t current_time = millis();
    if (current_time - dispense_start_time > min_dispense_time && get_switch_falling_edge()) {
        stop_dispense();
        publish_dispense_done(true);
    }
    if (current_time - dispense_start_time > max_dispense_time) {
        stop_dispense();
        publish_dispense_done(false);
    }
}

bool switch_state = false;
bool prev_switch_read_state = false;
uint32_t last_debounce_time = 0;
uint32_t DEBOUNCE_DELAY = 50;

bool read_limit_switch() {
    return !digitalRead(LIMIT_SWITCH_PIN);
}

bool get_limit_switch_state() {
    uint32_t current_time = millis();
    bool state = read_limit_switch();
    if (state != prev_switch_read_state) {
        last_debounce_time = current_time;
    }
    prev_switch_read_state = state;

    if (current_time - last_debounce_time > DEBOUNCE_DELAY) {
        switch_state = state;
    }
    return switch_state;
}

bool prev_switch_falling_edge_state = false;
bool get_switch_falling_edge() {
    bool state = get_limit_switch_state();
    bool did_fall = false;
    if (!state && prev_switch_falling_edge_state) {
        did_fall = true;
    }
    prev_switch_falling_edge_state = state;
    return did_fall;
}


void publish_dispense_status(bool is_dispensing) {
    snprintf(msg, MSG_BUFFER_SIZE, "%s\t%d", device_name, is_dispensing);
    Serial.print("Publish dispense status: ");
    Serial.println(msg);
    client.publish("is_dispensing", msg);
}

void publish_dispense_done(bool success) {
    snprintf(msg, MSG_BUFFER_SIZE, "%s\t%d", device_name, success);
    Serial.print("Publish dispense done: ");
    Serial.println(msg);
    client.publish("dispense_done", msg);
}

void reconnect() {
  // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection on ");
        Serial.print(mqtt_server);
        Serial.print("...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            client.subscribe("start_dispense");
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
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(100);

    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        while (true) {
            Serial.println("Could not find Motor Shield. Check wiring.");
            delay(1000);
        }
    }

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c)
        {
        case 's':
            start_dispense();
            break;
        default:
            break;
        }
    }
    check_dispense();

    if (!client.connected() && !is_dispensing) {
        reconnect();
    }
    client.loop();

    unsigned long now = millis();
    if (now - prev_publish_time > 1000) {
        prev_publish_time = now;
        publish_dispense_status(is_dispensing);
    }
}
