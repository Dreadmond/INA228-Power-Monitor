#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ==================== CONFIGURATION ====================
// WiFi credentials
const char* WIFI_SSID = "Plumdog";
const char* WIFI_PASSWORD = "Zemeckis";

// MQTT broker settings
const char* MQTT_SERVER = "YOUR_MQTT_BROKER_IP";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "";      // Leave empty if no auth
const char* MQTT_PASSWORD = "";  // Leave empty if no auth

// Device settings
const char* DEVICE_NAME = "ina228_power_monitor";
const char* DEVICE_FRIENDLY_NAME = "INA228 Power Monitor";

// INA228 settings
const uint8_t INA228_ADDR = 0x40;  // Default I2C address (A0=GND, A1=GND)
const float SHUNT_RESISTOR = 0.015; // 15mΩ shunt resistor (adjust for your setup)
const float MAX_CURRENT = 10.0;     // Maximum expected current in Amps

// Update interval (ms)
const unsigned long UPDATE_INTERVAL = 5000;

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

// ==================== GLOBALS ====================
WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastUpdate = 0;
bool haDiscoveryPublished = false;
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
    // Check manufacturer ID (should be 0x5449 = "TI")
    uint16_t mfgId = readRegister16(INA228_REG_MANUFACTURER_ID);
    Serial.printf("Manufacturer ID: 0x%04X\n", mfgId);

    if (mfgId != 0x5449) {
        Serial.println("INA228 not found!");
        return false;
    }

    // Reset the device
    writeRegister16(INA228_REG_CONFIG, 0x8000);
    delay(10);

    // Configure ADC: continuous mode, all measurements
    // Bits 15-12: Mode = 1111 (continuous bus, shunt, temp)
    // Bits 11-9: VBUSCT = 101 (1052µs)
    // Bits 8-6: VSHCT = 101 (1052µs)
    // Bits 5-3: VTCT = 101 (1052µs)
    // Bits 2-0: AVG = 011 (16 averages)
    writeRegister16(INA228_REG_ADC_CONFIG, 0xFB6B);

    // Calculate and set calibration
    // SHUNT_CAL = 13107.2 × 10^6 × CURRENT_LSB × Rshunt
    // CURRENT_LSB = Max Expected Current / 2^19
    currentLSB = MAX_CURRENT / 524288.0;  // 2^19
    uint16_t shuntCal = (uint16_t)(13107.2e6 * currentLSB * SHUNT_RESISTOR);
    writeRegister16(INA228_REG_SHUNT_CAL, shuntCal);

    Serial.printf("Current LSB: %.9f A\n", currentLSB);
    Serial.printf("Shunt Cal: %u\n", shuntCal);

    return true;
}

float readBusVoltage() {
    uint32_t raw = readRegister24(INA228_REG_VBUS);
    // VBUS LSB = 195.3125µV, shift right 4 bits (20-bit value in 24-bit register)
    return (raw >> 4) * 195.3125e-6;
}

float readShuntVoltage() {
    int32_t raw = readRegister24(INA228_REG_VSHUNT);
    // Sign extend from 24-bit to 32-bit
    if (raw & 0x800000) raw |= 0xFF000000;
    // VSHUNT LSB = 312.5nV, shift right 4 bits
    return (raw >> 4) * 312.5e-9;
}

float readCurrent() {
    int32_t raw = readRegister24(INA228_REG_CURRENT);
    // Sign extend from 24-bit to 32-bit
    if (raw & 0x800000) raw |= 0xFF000000;
    // Shift right 4 bits (20-bit value)
    return (raw >> 4) * currentLSB;
}

float readPower() {
    uint32_t raw = readRegister24(INA228_REG_POWER);
    // Power LSB = 3.2 × Current_LSB
    return raw * 3.2 * currentLSB;
}

float readTemperature() {
    uint16_t raw = readRegister16(INA228_REG_DIETEMP);
    // Temp LSB = 7.8125m°C
    return (raw >> 4) * 7.8125e-3;
}

// ==================== MQTT FUNCTIONS ====================
void publishHADiscovery() {
    String baseTopic = "homeassistant/sensor/" + String(DEVICE_NAME);

    // Device info JSON (shared across all entities)
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
        Serial.printf("Published discovery: %s\n", sensor.id);
    }

    haDiscoveryPublished = true;
}

void reconnectMQTT() {
    while (!mqtt.connected()) {
        Serial.print("Connecting to MQTT...");

        String clientId = String(DEVICE_NAME) + "_" + String(random(0xffff), HEX);
        bool connected;

        if (strlen(MQTT_USER) > 0) {
            connected = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
        } else {
            connected = mqtt.connect(clientId.c_str());
        }

        if (connected) {
            Serial.println("connected!");
            haDiscoveryPublished = false; // Re-publish discovery on reconnect
        } else {
            Serial.printf("failed, rc=%d, retrying in 5s\n", mqtt.state());
            delay(5000);
        }
    }
}

void publishReadings(float voltage, float current, float power, float temperature) {
    JsonDocument doc;
    doc["voltage"] = round(voltage * 1000) / 1000.0;  // 3 decimal places
    doc["current"] = round(current * 1000) / 1000.0;
    doc["power"] = round(power * 1000) / 1000.0;
    doc["temperature"] = round(temperature * 10) / 10.0;  // 1 decimal place

    String payload;
    serializeJson(doc, payload);

    String topic = String(DEVICE_NAME) + "/state";
    mqtt.publish(topic.c_str(), payload.c_str());

    Serial.printf("Published: V=%.3f, I=%.3f, P=%.3f, T=%.1f\n",
                  voltage, current, power, temperature);
}

// ==================== SETUP & LOOP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\nINA228 Power Monitor Starting...");

    // Initialize I2C (FireBeetle ESP32-E default pins: SDA=21, SCL=22)
    Wire.begin(21, 22);
    Wire.setClock(400000);  // 400kHz I2C

    // Initialize INA228
    if (!initINA228()) {
        Serial.println("Failed to initialize INA228! Check wiring.");
        while (1) delay(1000);
    }
    Serial.println("INA228 initialized successfully");

    // Connect to WiFi
    Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

    // Setup MQTT
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(1024);  // Larger buffer for HA discovery messages
}

void loop() {
    // Ensure MQTT connection
    if (!mqtt.connected()) {
        reconnectMQTT();
    }
    mqtt.loop();

    // Publish Home Assistant discovery (once after connect)
    if (!haDiscoveryPublished) {
        publishHADiscovery();
    }

    // Read and publish at interval
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();

        float voltage = readBusVoltage();
        float current = readCurrent();
        float power = readPower();
        float temperature = readTemperature();

        publishReadings(voltage, current, power, temperature);
    }
}
