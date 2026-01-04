#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

// ==================== CONFIGURATION ====================
// Device settings
const char* DEVICE_NAME = "ina228_power_monitor";
const char* DEVICE_FRIENDLY_NAME = "INA228 Power Monitor";

// INA228 settings
const uint8_t INA228_ADDR = 0x40;  // Default I2C address (A0=GND, A1=GND)
const float SHUNT_RESISTOR = 0.015; // 15mΩ shunt resistor (adjust for your setup)
const float MAX_CURRENT = 10.0;     // Maximum expected current in Amps

// Sleep intervals (seconds) - varies based on battery voltage
const uint64_t SLEEP_INTERVAL_FAST = 5;     // < 3.3V (low battery alert)
const uint64_t SLEEP_INTERVAL_NORMAL = 30;  // 3.3V - 4.0V
const uint64_t SLEEP_INTERVAL_SLOW = 60;    // > 4.0V (full battery, save power)

// Voltage thresholds for interval adjustment
const float VOLTAGE_LOW = 3.3;
const float VOLTAGE_HIGH = 4.0;

// Publish HA discovery every N boots (saves time/power)
const int DISCOVERY_INTERVAL = 20;

// ==================== INA228 REGISTERS ====================
#define INA228_REG_CONFIG      0x00
#define INA228_REG_ADC_CONFIG  0x01
#define INA228_REG_SHUNT_CAL   0x02
#define INA228_REG_VSHUNT      0x04
#define INA228_REG_VBUS        0x05
#define INA228_REG_DIETEMP     0x06
#define INA228_REG_CURRENT     0x07
#define INA228_REG_POWER       0x08
#define INA228_REG_MANUFACTURER_ID 0x3E
#define INA228_REG_DEVICE_ID   0x3F

// ==================== RTC MEMORY (survives deep sleep) ====================
RTC_DATA_ATTR int bootCount = 0;

// ==================== GLOBALS ====================
WiFiClient espClient;
PubSubClient mqtt(espClient);
float currentLSB;

// ==================== INA228 FUNCTIONS ====================
void writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t readRegister16(uint8_t reg) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(INA228_ADDR, (uint8_t)2);
    uint16_t value = Wire.read() << 8;
    value |= Wire.read();
    return value;
}

uint32_t readRegister24(uint8_t reg) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(INA228_ADDR, (uint8_t)3);
    uint32_t value = (uint32_t)Wire.read() << 16;
    value |= (uint32_t)Wire.read() << 8;
    value |= Wire.read();
    return value;
}

bool initINA228() {
    uint16_t mfgId = readRegister16(INA228_REG_MANUFACTURER_ID);
    if (mfgId != 0x5449) {
        Serial.printf("INA228 not found! (ID: 0x%04X)\n", mfgId);
        return false;
    }

    // Reset the device
    writeRegister16(INA228_REG_CONFIG, 0x8000);
    delay(10);

    // Configure ADC: continuous mode, all measurements
    // Mode=1111, VBUSCT=101 (1052µs), VSHCT=101, VTCT=101, AVG=011 (16 avg)
    writeRegister16(INA228_REG_ADC_CONFIG, 0xFB6B);

    // Calculate and set calibration
    currentLSB = MAX_CURRENT / 524288.0;  // 2^19
    uint16_t shuntCal = (uint16_t)(13107.2e6 * currentLSB * SHUNT_RESISTOR);
    writeRegister16(INA228_REG_SHUNT_CAL, shuntCal);

    return true;
}

float readBusVoltage() {
    uint32_t raw = readRegister24(INA228_REG_VBUS);
    return (raw >> 4) * 195.3125e-6;
}

float readCurrent() {
    int32_t raw = readRegister24(INA228_REG_CURRENT);
    if (raw & 0x800000) raw |= 0xFF000000;
    return (raw >> 4) * currentLSB;
}

float readPower() {
    uint32_t raw = readRegister24(INA228_REG_POWER);
    return raw * 3.2 * currentLSB;
}

float readTemperature() {
    uint16_t raw = readRegister16(INA228_REG_DIETEMP);
    return (raw >> 4) * 7.8125e-3;
}

// ==================== MQTT FUNCTIONS ====================
void publishHADiscovery() {
    String baseTopic = "homeassistant/sensor/" + String(DEVICE_NAME);

    JsonDocument deviceDoc;
    deviceDoc["identifiers"][0] = DEVICE_NAME;
    deviceDoc["name"] = DEVICE_FRIENDLY_NAME;
    deviceDoc["model"] = "INA228";
    deviceDoc["manufacturer"] = "Texas Instruments";

    struct SensorConfig {
        const char* id;
        const char* name;
        const char* unit;
        const char* deviceClass;
        const char* icon;
    };

    SensorConfig sensors[] = {
        {"voltage", "Bus Voltage", "V", "voltage", "mdi:flash"},
        {"current", "Current", "A", "current", "mdi:current-dc"},
        {"power", "Power", "W", "power", "mdi:lightning-bolt"},
        {"temperature", "Temperature", "°C", "temperature", "mdi:thermometer"}
    };

    for (auto& sensor : sensors) {
        JsonDocument doc;
        doc["name"] = sensor.name;
        doc["unique_id"] = String(DEVICE_NAME) + "_" + sensor.id;
        doc["state_topic"] = String(DEVICE_NAME) + "/state";
        doc["value_template"] = "{{ value_json." + String(sensor.id) + " }}";
        doc["unit_of_measurement"] = sensor.unit;
        doc["device_class"] = sensor.deviceClass;
        doc["icon"] = sensor.icon;
        doc["device"] = deviceDoc;

        String configTopic = baseTopic + "_" + sensor.id + "/config";
        String payload;
        serializeJson(doc, payload);

        mqtt.publish(configTopic.c_str(), payload.c_str(), true);
    }
    Serial.println("Published HA discovery");
}

bool connectMQTT() {
    String clientId = String(DEVICE_NAME) + "_" + String(random(0xffff), HEX);

    if (strlen(MQTT_USER) > 0) {
        return mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
    }
    return mqtt.connect(clientId.c_str());
}

void publishReadings(float voltage, float current, float power, float temperature) {
    JsonDocument doc;
    doc["voltage"] = round(voltage * 1000) / 1000.0;
    doc["current"] = round(current * 1000) / 1000.0;
    doc["power"] = round(power * 1000) / 1000.0;
    doc["temperature"] = round(temperature * 10) / 10.0;

    String payload;
    serializeJson(doc, payload);

    mqtt.publish((String(DEVICE_NAME) + "/state").c_str(), payload.c_str());
    Serial.printf("Published: V=%.3f, I=%.3f, P=%.3f, T=%.1f\n",
                  voltage, current, power, temperature);
}

// ==================== DEEP SLEEP ====================
void goToSleep(uint64_t seconds) {
    Serial.printf("Sleeping for %llu seconds...\n", seconds);
    Serial.flush();

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
    esp_deep_sleep_start();
}

uint64_t getSleepInterval(float voltage) {
    if (voltage < VOLTAGE_LOW) {
        return SLEEP_INTERVAL_FAST;  // Low battery - report frequently
    } else if (voltage > VOLTAGE_HIGH) {
        return SLEEP_INTERVAL_SLOW;  // Full battery - conserve power
    }
    return SLEEP_INTERVAL_NORMAL;
}

// ==================== SETUP (runs every wake) ====================
void setup() {
    Serial.begin(115200);
    delay(100);

    bootCount++;
    Serial.printf("\n\nBoot #%d\n", bootCount);

    // Initialize I2C (FireBeetle ESP32-E default pins: SDA=21, SCL=22)
    Wire.begin(21, 22);
    Wire.setClock(400000);

    // Initialize INA228
    if (!initINA228()) {
        Serial.println("INA228 init failed! Sleeping 60s...");
        goToSleep(60);
    }

    // Wait for ADC conversion (with averaging)
    delay(100);

    // Read sensor values
    float voltage = readBusVoltage();
    float current = readCurrent();
    float power = readPower();
    float temperature = readTemperature();

    Serial.printf("Read: V=%.3f, I=%.3f, P=%.3f, T=%.1f\n",
                  voltage, current, power, temperature);

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int wifiAttempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
        delay(500);
        wifiAttempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi failed! Sleeping...");
        goToSleep(getSleepInterval(voltage));
    }
    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());

    // Connect to MQTT
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(1024);

    if (!connectMQTT()) {
        Serial.println("MQTT failed! Sleeping...");
        goToSleep(getSleepInterval(voltage));
    }
    Serial.println("MQTT connected");

    // Publish HA discovery periodically (not every boot)
    if (bootCount == 1 || bootCount % DISCOVERY_INTERVAL == 0) {
        publishHADiscovery();
    }

    // Publish readings
    publishReadings(voltage, current, power, temperature);

    // Give MQTT time to send
    mqtt.loop();
    delay(100);
    mqtt.loop();

    // Calculate sleep time based on voltage
    uint64_t sleepTime = getSleepInterval(voltage);
    goToSleep(sleepTime);
}

void loop() {
    // Never reached - device sleeps after setup()
}
