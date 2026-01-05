#include "secrets.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

// ==================== CONFIGURATION ====================
// Device settings
const char *DEVICE_NAME = "ina228_power_monitor";
const char *DEVICE_FRIENDLY_NAME = "INA228 Power Monitor";

// INA228 settings
const uint8_t INA228_ADDR = 0x40;   // Default I2C address (A0=GND, A1=GND)
const float SHUNT_RESISTOR = 0.015; // 15mΩ shunt resistor (Adafruit INA228)
const float MAX_CURRENT =
    0.1; // Maximum expected current in Amps (100mA for solar panel)

// Sleep intervals (seconds) - titrated based on battery voltage
// Indoor solar produces only ~200µW peak (~50µA at 3.7V)
// One wake cycle uses ~0.15mAh, needs ~3 hours of peak sun to recover!
// Lower voltage = longer sleep to conserve power and protect battery
const uint64_t SLEEP_MIN = 900;     // Minimum sleep at full charge (15 minutes)
const uint64_t SLEEP_MAX = 14400;   // Maximum sleep when battery critical (4 hours)

// Voltage thresholds for titration
const float VOLTAGE_CRITICAL = 3.4; // Below this: maximum sleep (emergency mode)
const float VOLTAGE_LOW = 3.6;      // Below this: long sleep
const float VOLTAGE_FULL = 3.9;     // Above this: minimum sleep

// Publish HA discovery every N boots (saves time/power)
const int DISCOVERY_INTERVAL = 20;

// GPIO to power the INA228 (allows complete power-off during sleep)
const int INA228_POWER_PIN = 13;

// ==================== INA228 REGISTERS ====================
#define INA228_REG_CONFIG 0x00
#define INA228_REG_ADC_CONFIG 0x01
#define INA228_REG_SHUNT_CAL 0x02
#define INA228_REG_VSHUNT 0x04
#define INA228_REG_VBUS 0x05
#define INA228_REG_DIETEMP 0x06
#define INA228_REG_CURRENT 0x07
#define INA228_REG_POWER 0x08
#define INA228_REG_MANUFACTURER_ID 0x3E
#define INA228_REG_DEVICE_ID 0x3F

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
  currentLSB = MAX_CURRENT / 524288.0; // 2^19
  uint16_t shuntCal = (uint16_t)(13107.2e6 * currentLSB * SHUNT_RESISTOR);
  writeRegister16(INA228_REG_SHUNT_CAL, shuntCal);

  return true;
}

void sleepINA228() {
  // Put INA228 into shutdown mode (MODE=0x0 in ADC_CONFIG register)
  // This sets all mode bits to 0, which shuts down the ADC
  writeRegister16(INA228_REG_ADC_CONFIG, 0x0000);
  Serial.println("INA228 in shutdown mode");
}

float readBusVoltage() {
  uint32_t raw = readRegister24(INA228_REG_VBUS);
  Serial.printf("  VBUS raw: 0x%06X (%u)\n", raw, raw);
  return (raw >> 4) * 195.3125e-6;
}

float readShuntVoltage() {
  int32_t raw = readRegister24(INA228_REG_VSHUNT);
  Serial.printf("  VSHUNT raw: 0x%06X (%d)\n", raw, raw);
  // Sign extend 24-bit to 32-bit, divide by 16, scale by 312.5 nV/LSB
  if (raw & 0x800000)
    raw |= 0xFF000000;
  return (float)raw / 16.0 * 312.5e-9; // Returns voltage in V
}

float readCurrent() {
  int32_t raw = readRegister24(INA228_REG_CURRENT);
  Serial.printf("  CURRENT raw: 0x%06X (%d)\n", raw, raw);
  // Sign extend 24-bit to 32-bit, then divide by 16 (per Adafruit library)
  if (raw & 0x800000)
    raw |= 0xFF000000;
  return (float)raw / 16.0 * currentLSB;
}

float readPower() {
  uint32_t raw = readRegister24(INA228_REG_POWER);
  Serial.printf("  POWER raw: 0x%06X (%u)\n", raw, raw);
  return raw * 3.2 * currentLSB;
}

float readTemperature() {
  int16_t raw = readRegister16(INA228_REG_DIETEMP);
  Serial.printf("  DIETEMP raw: 0x%04X (%d)\n", raw, raw);
  // INA228 uses full 16-bit signed value, no shift (per Adafruit library)
  return (float)raw * 7.8125 / 1000.0;
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
    const char *id;
    const char *name;
    const char *unit;
    const char *deviceClass;
    const char *icon;
  };

  SensorConfig sensors[] = {
      {"voltage", "Bus Voltage", "mV", nullptr, "mdi:flash"},
      {"current", "Current", "uA", nullptr, "mdi:current-dc"},
      {"power", "Power", "uW", nullptr, "mdi:lightning-bolt"},
      {"temperature", "Temperature", "°C", "temperature", "mdi:thermometer"},
      {"battery", "Battery", "V", "voltage", "mdi:battery"}};

  for (auto &sensor : sensors) {
    JsonDocument doc;
    doc["name"] = sensor.name;
    doc["unique_id"] = String(DEVICE_NAME) + "_" + sensor.id;
    doc["state_topic"] = String(DEVICE_NAME) + "/state";
    doc["value_template"] = "{{ value_json." + String(sensor.id) + " }}";
    doc["unit_of_measurement"] = sensor.unit;
    if (sensor.deviceClass != nullptr) {
      doc["device_class"] = sensor.deviceClass;
    }
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

void publishReadings(float voltage, float current, float power,
                     float temperature, float battery) {
  JsonDocument doc;
  doc["voltage"] = round(voltage * 1000000) / 1000.0;    // V to mV
  doc["current"] = round(current * 1000000000) / 1000.0; // A to µA
  doc["power"] = round(power * 1000000000) / 1000.0;     // W to µW
  doc["temperature"] = round(temperature * 10) / 10.0;
  doc["battery"] = round(battery * 100) / 100.0;

  String payload;
  serializeJson(doc, payload);

  mqtt.publish((String(DEVICE_NAME) + "/state").c_str(), payload.c_str());
  Serial.printf("Published: V=%.1fmV, I=%.1fuA, P=%.1fuW, T=%.1fC, Bat=%.2fV\n",
                voltage * 1000, current * 1000000, power * 1000000, temperature,
                battery);
}

// ==================== DEEP SLEEP ====================
void goToSleep(uint64_t seconds) {
  Serial.printf("Sleeping for %llu seconds...\n", seconds);

  // Power off the INA228 completely via GPIO (saves ~5mA including LED)
  digitalWrite(INA228_POWER_PIN, LOW);
  Serial.println("INA228 powered off");

  Serial.flush();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
  esp_deep_sleep_start();
}

uint64_t getSleepInterval(float voltage) {
  // Titrate sleep duration based on battery voltage
  // Lower voltage = longer sleep to protect battery from going below 3.3V
  
  if (voltage <= VOLTAGE_CRITICAL) {
    // Emergency mode - sleep as long as possible
    Serial.println("CRITICAL: Battery at minimum! Maximum sleep.");
    return SLEEP_MAX;
  }
  
  if (voltage >= VOLTAGE_FULL) {
    // Full battery - can afford frequent updates
    return SLEEP_MIN;
  }
  
  // Linear interpolation between VOLTAGE_CRITICAL and VOLTAGE_FULL
  // As voltage decreases, sleep time increases
  float range = VOLTAGE_FULL - VOLTAGE_CRITICAL;
  float normalized = (voltage - VOLTAGE_CRITICAL) / range; // 0.0 to 1.0
  
  // Invert: low voltage = high sleep, high voltage = low sleep
  uint64_t sleepTime = SLEEP_MAX - (uint64_t)(normalized * (SLEEP_MAX - SLEEP_MIN));
  
  Serial.printf("Titrated sleep: %.2fV -> %llu seconds\n", voltage, sleepTime);
  return sleepTime;
}

// ==================== SETUP (runs every wake) ====================
void setup() {
  Serial.begin(115200);
  delay(100);

  bootCount++;
  Serial.printf("\n\nBoot #%d\n", bootCount);

  // Power on the INA228 via GPIO
  pinMode(INA228_POWER_PIN, OUTPUT);
  digitalWrite(INA228_POWER_PIN, HIGH);
  delay(10); // Let power stabilize

  // Initialize I2C (FireBeetle ESP32-E default pins: SDA=21, SCL=22)
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Initialize INA228
  if (!initINA228()) {
    Serial.println("INA228 init failed! Sleeping 60s...");
    goToSleep(60);
  }

  // Wait for ADC conversion (with 16x averaging)
  delay(200);

  // Read sensor values
  float voltage = readBusVoltage();
  float shuntV = readShuntVoltage();
  float current = readCurrent();
  float power = readPower();
  float temperature = readTemperature();

  // Read FireBeetle battery voltage via ADC (GPIO34 on FireBeetle ESP32-E)
  float batteryVoltage = analogRead(34) * 2.0 * 3.3 / 4095.0;
  Serial.printf("FireBeetle battery: %.2fV\n", batteryVoltage);

  Serial.printf("Read: V=%.3f, Vshunt=%.6f, I=%.3f, P=%.3f, T=%.1f\n", voltage,
                shuntV, current, power, temperature);

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
    goToSleep(getSleepInterval(batteryVoltage));
  }
  Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());

  // Connect to MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setBufferSize(1024);

  if (!connectMQTT()) {
    Serial.println("MQTT failed! Sleeping...");
    goToSleep(getSleepInterval(batteryVoltage));
  }
  Serial.println("MQTT connected");

  // Publish HA discovery periodically (not every boot)
  if (bootCount == 1 || bootCount % DISCOVERY_INTERVAL == 0) {
    publishHADiscovery();
  }

  // Publish readings
  publishReadings(voltage, current, power, temperature, batteryVoltage);

  // Give MQTT time to send
  mqtt.loop();
  delay(100);
  mqtt.loop();

  // Calculate sleep time based on FireBeetle battery voltage
  uint64_t sleepTime = getSleepInterval(batteryVoltage);
  goToSleep(sleepTime);
}

void loop() {
  // Never reached - device sleeps after setup()
}
