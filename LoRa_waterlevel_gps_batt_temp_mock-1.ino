#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>

// Deep sleep mode
// Saves the LMIC structure during DeepSleep
RTC_DATA_ATTR lmic_t RTC_LMIC;
bool GOTO_DEEPSLEEP = false;
int cout_timer;
int count_numb = 0;

// Data arrays for sensor values and payload
char waterLevelStr[5];
char latitudeStr[9];
char longitudeStr[9];
char batteryStr[3];
char tempStr[3];
char payloadValue[25];

// Simulated sensor values (using double or float types)
double waterLevel;
double latitude;
double longitude;
double batteryVoltage;
double temperature;

// LoRaWAN Configuration
static const u1_t PROGMEM APPEUI[8] = {0x3b, 0x52, 0xba, 0x6b, 0x1b, 0x8c, 0x9a, 0x99};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = {0x3b, 0x52, 0xba, 0x6b, 0x1b, 0x8c, 0x9a, 0x99};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = {0x27, 0x04, 0x1c, 0x41, 0x04, 0x14, 0xe0, 0xe4, 0x99, 0xd2, 0xd5, 0x31, 0x5a, 0x76, 0x68, 0x02};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

uint8_t mydata[25];
static osjob_t sendjob;

// Time deep sleep mode.
const unsigned TX_INTERVAL = 300;  // 5 minutes
const unsigned TX_INTERVAL_LoRa = 60;  // 1 minute
int sequnNow = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 13,
    .dio = {2, 15, LMIC_UNUSED_PIN},
};

// WiFi and MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
BluetoothSerial SerialBT;  // Bluetooth Serial
Preferences preferences;  // To store WiFi credentials

const char *mqttServer = "110.164.181.55";  // Replace with your MQTT server
const int mqttPort = 1883;
const char *mqttUser = "YOUR_MQTT_USER";  // Optional
const char *mqttPassword = "YOUR_MQTT_PASSWORD";  // Optional

const char *mqttTopic = "device/status";  // MQTT topic to publish status

String ssid = "";
String password = "";
bool isWiFiConfigured = false;
bool wifiConnectionFailed = false;  // ตัวแปรใหม่สำหรับป้องกันการส่งข้อความซ้ำ
unsigned long wifiStartTime = 0;
const unsigned long wifiTimeout = 30000;  // เวลา Timeout 1 นาที (60000 ms)

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.

            
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
         case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            //                    :)
            if(sequnNow == 0){
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL_LoRa), do_send);
            }else if(sequnNow > 0){
              GOTO_DEEPSLEEP = true;             
            }else{
              Serial.println("LoRa ERROR");
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
   
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
       // test();
        getDataAndCreatePayload();

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F(mydata));
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void PrintRuntime()
{
    long seconds = millis() / 1000;
    Serial.print("Runtime: ");
    Serial.print(seconds);
    Serial.println(" seconds");
}

void deepSleepMode() {
    Serial.println("Node sensor going to deep sleep");
    Serial.flush();
    esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000);
    esp_deep_sleep_start();
}

void SaveLMICToRTC(int deepsleep_sec)
{
    Serial.println(F("Save LMIC to RTC"));
    RTC_LMIC = LMIC;

    // ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
    // Therefore reset DutyCyles

    unsigned long now = millis();

    // EU Like Bands
    #if defined(CFG_LMIC_EU_like)
    Serial.println(F("Reset CFG_LMIC_EU_like band avail"));
    for (int i = 0; i < MAX_BANDS; i++)
    {
        ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
        if (correctedAvail < 0)
        {
            correctedAvail = 0;
        }
        RTC_LMIC.bands[i].avail = correctedAvail;
    }

    RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (RTC_LMIC.globalDutyAvail < 0)
    {
        RTC_LMIC.globalDutyAvail = 0;
    }
  #else
    Serial.println(F("No DutyCycle recalculation function!"));
  #endif
}

void LoadLMICFromRTC()
{
    Serial.println(F("Load LMIC from RTC"));
    LMIC = RTC_LMIC;
}

void LoraWANDebug(lmic_t lmic_check)
{
    Serial.println("");
    Serial.println("");

    //LoraWANPrintLMICOpmode();
    Serial.println("");

    Serial.print(F("LMIC.seqnoUp = "));
    Serial.println(lmic_check.seqnoUp);

    Serial.print(F("LMIC.globalDutyRate = "));
    Serial.print(lmic_check.globalDutyRate);
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(lmic_check.globalDutyRate) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("LMIC.globalDutyAvail = "));
    Serial.print(lmic_check.globalDutyAvail);
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(lmic_check.globalDutyAvail) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("LMICbandplan_nextTx = "));
    Serial.print(LMICbandplan_nextTx(os_getTime()));
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(LMICbandplan_nextTx(os_getTime())) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("os_getTime = "));
    Serial.print(os_getTime());
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(os_getTime()) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("LMIC.txend = "));
    Serial.println(lmic_check.txend);
    Serial.print(F("LMIC.txChnl = "));
    Serial.println(lmic_check.txChnl);

    Serial.println(F("Band \tavail \t\tavail_sec\tlastchnl \ttxcap"));
    for (u1_t bi = 0; bi < MAX_BANDS; bi++)
    {
        Serial.print(bi);
        Serial.print("\t");
        Serial.print(lmic_check.bands[bi].avail);
        Serial.print("\t\t");
        Serial.print(osticks2ms(lmic_check.bands[bi].avail) / 1000);
        Serial.print("\t\t");
        Serial.print(lmic_check.bands[bi].lastchnl);
        Serial.print("\t\t");
        Serial.println(lmic_check.bands[bi].txcap);
    }
    Serial.println("");
    Serial.println("");
}

void LoraWANPrintLMICOpmode(void)
{
    Serial.print(F("LMIC.opmode: "));
    if (LMIC.opmode & OP_NONE)
    {
        Serial.print(F("OP_NONE "));
    }
    if (LMIC.opmode & OP_SCAN)
    {
        Serial.print(F("OP_SCAN "));
    }
    if (LMIC.opmode & OP_TRACK)
    {
        Serial.print(F("OP_TRACK "));
    }
    if (LMIC.opmode & OP_JOINING)
    {
        Serial.print(F("OP_JOINING "));
    }
    if (LMIC.opmode & OP_TXDATA)
    {
        Serial.print(F("OP_TXDATA "));
    }
    if (LMIC.opmode & OP_POLL)
    {
        Serial.print(F("OP_POLL "));
    }
    if (LMIC.opmode & OP_REJOIN)
    {
        Serial.print(F("OP_REJOIN "));
    }
    if (LMIC.opmode & OP_SHUTDOWN)
    {
        Serial.print(F("OP_SHUTDOWN "));
    }
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.print(F("OP_TXRXPEND "));
    }
    if (LMIC.opmode & OP_RNDTX)
    {
        Serial.print(F("OP_RNDTX "));
    }
    if (LMIC.opmode & OP_PINGINI)
    {
        Serial.print(F("OP_PINGINI "));
    }
    if (LMIC.opmode & OP_PINGABLE)
    {
        Serial.print(F("OP_PINGABLE "));
    }
    if (LMIC.opmode & OP_NEXTCHNL)
    {
        Serial.print(F("OP_NEXTCHNL "));
    }
    if (LMIC.opmode & OP_LINKDEAD)
    {
        Serial.print(F("OP_LINKDEAD "));
    }
    if (LMIC.opmode & OP_LINKDEAD)
    {
        Serial.print(F("OP_LINKDEAD "));
    }
    if (LMIC.opmode & OP_TESTMODE)
    {
        Serial.print(F("OP_TESTMODE "));
    }
    if (LMIC.opmode & OP_UNJOIN)
    {
        Serial.print(F("OP_UNJOIN "));
    }
}


void setup() {
    cout_timer = 0;
    Serial.begin(9600);
    while (!Serial) {}

    SerialBT.begin("ESP32_BT");  // ตั้งชื่อ Bluetooth ของ ESP32
    preferences.begin("wifi", false);  // ใช้ Preferences สำหรับบันทึกข้อมูล

    mqttClient.setServer(mqttServer, mqttPort);  // ตั้งค่า MQTT Server

    // โหลดค่าที่บันทึกไว้
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");

    os_init();
    LMIC_reset();

    // Set dummy values for testing
    waterLevel = 50.0;  // Simulated water level in cm
    latitude = 13.7563;  // Simulated latitude (Bangkok)
    longitude = 100.5018;  // Simulated longitude (Bangkok)
    batteryVoltage = 3.7;  // Simulated battery voltage in V
    temperature = 25.0;  // Simulated temperature in C

    // ค้นหา Wi-Fi และเชื่อมต่อจนกว่าจะสำเร็จ
    while (WiFi.status() != WL_CONNECTED) {
        if (ssid.isEmpty() || password.isEmpty()) {
            SerialBT.println("No Wi-Fi credentials stored, waiting for Bluetooth setup...");
            return;  // รอข้อมูลผ่าน Bluetooth
        }

        Serial.println("Attempting to connect to Wi-Fi...");
        WiFi.begin(ssid.c_str(), password.c_str());  // Try to connect to Wi-Fi
        unsigned long wifiConnectTimeout = millis();

        // Wait for Wi-Fi connection to be established
        while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectTimeout < 30000) {
            delay(1000);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Connected to Wi-Fi!");
            Serial.println("IP Address: " + WiFi.localIP().toString());
            isWiFiConfigured = true;
            wifiConnectionFailed = false;
        } else {
            Serial.println("Wi-Fi connection failed. Retrying...");
        }
    }

    // หลังจากเชื่อมต่อ Wi-Fi สำเร็จ จะเริ่มส่งข้อมูล
    do_send(&sendjob);
}

void loop() {
    // ตรวจสอบคำสั่งจาก Bluetooth
    if (SerialBT.available()) {
        String data = SerialBT.readStringUntil('\n');
        data.trim();  // ลบช่องว่างหรืออักขระพิเศษ

        if (data == "RESET") {  // คำสั่งรีเซ็ต Wi-Fi
            Serial.println("Resetting Wi-Fi credentials...");
            preferences.clear();  // ลบข้อมูล Wi-Fi ที่เก็บไว้
            ssid = "";
            password = "";
            isWiFiConfigured = false;
            wifiConnectionFailed = false;  // รีเซ็ตสถานะการเชื่อมต่อ
            WiFi.disconnect();
            SerialBT.println("Wi-Fi reset. Send new credentials in format: SSID,PASSWORD");
        } else if (data.indexOf(',') != -1) {
            int commaIndex = data.indexOf(',');
            ssid = data.substring(0, commaIndex);
            password = data.substring(commaIndex + 1);
            
            Serial.println("New credentials received:");
            Serial.println("SSID: " + ssid);
            Serial.println("Password: " + password);
            
            preferences.putString("ssid", ssid);
            preferences.putString("password", password);
            
            WiFi.begin(ssid.c_str(), password.c_str());
            unsigned long wifiConnectTimeout = millis();

            // Wait for Wi-Fi connection to be established
            while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectTimeout < 30000) {
                delay(1000);
                Serial.print(".");
            }

            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Connected to Wi-Fi!");
                Serial.println("IP Address: " + WiFi.localIP().toString());
                isWiFiConfigured = true;
                wifiConnectionFailed = false;
            } else {
                Serial.println("Wi-Fi connection failed.");
            }
        }
    }

    // ถ้าเชื่อมต่อ Wi-Fi แล้ว ให้เริ่มตรวจสอบ MQTT
    if (WiFi.status() != WL_CONNECTED && !wifiConnectionFailed) {
        // เชื่อมต่อ Wi-Fi ไม่สำเร็จภายในเวลา 30 วินาที
        wifiConnectionFailed = true;
        Serial.println("Wi-Fi connection failed. Requesting new credentials...");
        SerialBT.println("Wi-Fi connection failed. Please send new credentials in format: SSID,PASSWORD");
    }

    if (isWiFiConfigured && WiFi.status() == WL_CONNECTED) {
        // Attempt MQTT connection if not already connected
        if (!mqttClient.connected()) {
            Serial.println("Connecting to MQTT...");
            if (mqttClient.connect("ESP32_Client")) {
                Serial.println("Connected to MQTT broker");

                // Create and send JSON payload with Wi-Fi SSID
                String jsonPayload = "{\"SSID\": \"" + ssid + "\"}";
                if (mqttClient.publish(mqttTopic, jsonPayload.c_str())) {
                    Serial.println("MQTT message sent: " + jsonPayload);
                } else {
                    Serial.println("Failed to send MQTT message");
                }
            } else {
                Serial.println("MQTT connection failed");
            }
        }

        // ส่งข้อมูลผ่าน LoRa
        os_runloop_once();

        // ตรวจสอบว่า LoRa สามารถเข้าสู่โหมด Deep Sleep ได้หรือไม่
        const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((TX_INTERVAL * 1000)));
        if (!timeCriticalJobs && GOTO_DEEPSLEEP == true && !(LMIC.opmode & OP_TXRXPEND)) {
            Serial.print(F("Can go to sleep "));
            LoraWANPrintLMICOpmode();
            SaveLMICToRTC(TX_INTERVAL);
            deepSleepMode();  // Enter deep sleep mode
        } else {
            // แสดงข้อมูลการดีบักหากไม่ได้อยู่ในโหมด sleep
            static unsigned long lastPrintTime = 0;
            if (lastPrintTime + 2000 < millis()) {
                Serial.print(F("Cannot sleep "));
                Serial.print(F("TimeCriticalJobs: "));
                Serial.print(timeCriticalJobs);
                Serial.print(" ");
                PrintRuntime();
                lastPrintTime = millis();
            }
        }
    }
}

// ฟังก์ชันในการเชื่อมต่อ Wi-Fi
void connectWiFi() {
    if (ssid.isEmpty() || password.isEmpty()) {
        SerialBT.println("No Wi-Fi credentials stored, waiting for Bluetooth setup...");
        return;  // รอข้อมูลผ่าน Bluetooth
    }

    Serial.println("Attempting to connect to Wi-Fi...");
    WiFi.begin(ssid.c_str(), password.c_str());  // Try to connect to Wi-Fi
    unsigned long wifiConnectTimeout = millis();
    
    // Wait for Wi-Fi connection to be established
    while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectTimeout < 30000) {
        delay(1000);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to Wi-Fi!");
        Serial.println("IP Address: " + WiFi.localIP().toString());
        isWiFiConfigured = true;
        wifiConnectionFailed = false;

        // ส่งข้อมูลผ่าน MQTT เมื่อเชื่อมต่อ Wi-Fi สำเร็จ
        String jsonPayload = "{\"SSID\": \"" + ssid + "\"}";
        if (mqttClient.publish(mqttTopic, jsonPayload.c_str())) {
            Serial.println("MQTT message sent: " + jsonPayload);
        } else {
            Serial.println("Failed to send MQTT message");
        }
    } else {
        Serial.println("Wi-Fi connection failed.");
    }
}


void sendMQTTStatus(const char *status) {
    if (!mqttClient.connected()) {
        Serial.println("Reconnecting to MQTT...");
        while (!mqttClient.connected()) {
            if (mqttClient.connect("ESP32", mqttUser, mqttPassword)) {
                Serial.println("Connected to MQTT");
            } else {
                delay(5000);
            }
        }
    }

    String payload = "{\"status\":\"" + String(status) + "\"}";
    mqttClient.publish(mqttTopic, payload.c_str());
}

void getDataAndCreatePayload() {
    // Simulate reading values
    readWaterLevel();  // Simulated water level
    readGPS();  // Simulated GPS
    readBattery();  // Simulated battery voltage
    readTemperature();  // Simulated temperature

    // Convert values to hexadecimal strings for payload
    sprintf(waterLevelStr, "%04x", (int)waterLevel);               // Water Level (4 byte, Hex)
    sprintf(latitudeStr, "%08x", (uint32_t)(latitude * 1000000));  // Latitude (4 byte, Hex)
    sprintf(longitudeStr, "%08x", (uint32_t)(longitude * 1000000)); // Longitude (4 byte, Hex)
    sprintf(batteryStr, "%02x", (uint8_t)(batteryVoltage * 10));   // Battery Voltage (1 byte, Hex)
    sprintf(tempStr, "%02x", (uint8_t)(temperature + 40));         // Temperature (1 byte, Hex)

    // Create the payload by concatenating all values
    String payload = String(waterLevelStr) + String(latitudeStr) + String(longitudeStr) + String(batteryStr) + String(tempStr);
    payload.toCharArray(payloadValue, payload.length() + 1);

    // Debug Payload
    Serial.println("\nDebug:");
    Serial.print("Water Level (cm): ");
    Serial.println(waterLevel);
    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude);
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
    Serial.print("Temperature: ");
    Serial.println(temperature);

    Serial.print("\nPayload: ");
    Serial.println(payload);

    // Copy the payload to the transmission data
    memcpy_P(mydata, payloadValue, payload.length());
}

void readWaterLevel() {
    waterLevel = 50.0;  // 50 cm
}

void readGPS() {
    latitude = 13.7563;  // Bangkok latitude
    longitude = 100.5018;  // Bangkok longitude
}

void readBattery() {
    batteryVoltage = 3.7;  // 3.7 V
}

void readTemperature() {
    temperature = 25.0;  // 25°C
}
