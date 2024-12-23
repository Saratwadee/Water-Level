#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Wi-Fi Configuration
const char* ssid = "99207HOME_2.4G";
const char* password = "theP@ssw0rdis555";

// MQTT Configuration
const char* mqtt_server = "110.164.181.55";
const int mqtt_port = 1883;
const char* mqtt_topic = "sensor/data";

// Device Name
const char* device_name = "WaterLevelSensor_01";

// LoRaWAN Configuration
static const u1_t PROGMEM APPEUI[8] = {0x3b, 0x52, 0xba, 0x6b, 0x1b, 0x8c, 0x9a, 0x99};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = {0x3b, 0x52, 0xba, 0x6b, 0x1b, 0x8c, 0x9a, 0x99};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = {0x27, 0x04, 0x1c, 0x41, 0x04, 0x14, 0xe0, 0xe4, 0x99, 0xd2, 0xd5, 0x31, 0x5a, 0x76, 0x68, 0x02};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping for LoRaWAN
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 13,
    .dio = {2, 4, LMIC_UNUSED_PIN},
};

// Ultrasonic Sensor Pins
const int trigPin = 12;
const int echoPin = 14;

// GPS configuration
HardwareSerial mySerial(1);  // Use UART1 for GPS
TinyGPSPlus gps;

// Variables for LoRaWAN and MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
osjob_t sendjob;

unsigned long previousMillisMQTT = 0;
unsigned long previousMillisLoRa = 0;
const long intervalMQTT = 60000; // 1 minute
const long intervalLoRa = 60000; // 1 minutes

// Sensor data
float water_level = 0.0; // Water level will be calculated from the ultrasonic sensor
float latitude = 0.0;
float longitude = 0.0;

// Function to connect to Wi-Fi
void setup_wifi() {
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    mqttClient.setServer(mqtt_server, mqtt_port);
}

// Function to connect to MQTT broker
void reconnect_mqtt() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect(device_name)) {
            Serial.println("connected.");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            delay(5000);
        }
    }
}

// Function to publish data via MQTT
void publishMQTT() {
    StaticJsonDocument<256> doc;
    doc["device_name"] = device_name;
    doc["water_level"] = water_level;
    doc["latitude"] = latitude;
    doc["longitude"] = longitude;

    char buffer[256];
    serializeJson(doc, buffer);
    mqttClient.publish(mqtt_topic, buffer);
    // Print data to Serial Monitor
    Serial.println("Publishing MQTT data...");
    Serial.print("Water Level: ");
    Serial.println(water_level);
    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude);
    Serial.println("MQTT data sent.");
}

// Function to send data via LoRaWAN
void do_send(osjob_t* j) {
    char payload[12];
    snprintf(payload, sizeof(payload), "%.1f,%.4f,%.4f", water_level, latitude, longitude);

    LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
    Serial.println("LoRaWAN data sent.");
    Serial.print("Water Level: ");
    Serial.println(water_level);
    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude);
    Serial.println("LoRaWAN data sent.");
}

// LoRaWAN event handler
void onEvent(ev_t ev) {
    switch (ev) {
        case EV_TXCOMPLETE:
            Serial.println("LoRaWAN transmission complete.");
            break;
        default:
            Serial.println("LoRaWAN event.");
            break;
    }
}

// Function to read water level from ultrasonic sensor
float readWaterLevel() {
    long duration, distance;
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.0344 / 2; // Distance in cm
    
    return distance; // Water level (in cm)
}

// Function to read GPS data
void readGPS() {
    while (mySerial.available() > 0) {
        gps.encode(mySerial.read());
        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
        }
    }
}

void setup() {
    Serial.begin(9600);
    setup_wifi();
    
    // Initialize LoRaWAN
    os_init();
    LMIC_reset();
    LMIC_setTxData2(1, (uint8_t*)"Init", 4, 0);
    
    // Initialize ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    // Initialize GPS
    mySerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for GPS
}

void loop() {
    unsigned long currentMillis = millis();

    // Handle MQTT
    if (!mqttClient.connected()) reconnect_mqtt();
    mqttClient.loop();
    if (currentMillis - previousMillisMQTT >= intervalMQTT) {
        previousMillisMQTT = currentMillis;
        water_level = readWaterLevel(); // Update water level
        readGPS(); // Update GPS coordinates
        publishMQTT();
    }

    // Handle LoRaWAN
    if (currentMillis - previousMillisLoRa >= intervalLoRa) {
        previousMillisLoRa = currentMillis;
        do_send(&sendjob);
    }

    // Run LoRaWAN events
    os_runloop_once();
}
