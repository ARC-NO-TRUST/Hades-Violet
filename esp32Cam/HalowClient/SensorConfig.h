#ifndef SENSORCONFIG_H
#define SENSORCONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>

#include <HaLow.h>
#include <WiFi.h>
#include <Update.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <SensirionI2cScd4x.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_BME680.h>

// ===== Pins =====
#define BATTERY_ADC_PIN 1
#define CHARGING_STATUS_PIN 17
#define MQ_PIN 10
#define I2C_SDA_PIN 15
#define I2C_SCL_PIN 16

// ===== Dummy Data Constants =====
#define UV_DUMMY_VALUE      5.2
#define WIND_BASE           3.0
#define WIND_VARIATION      1.5
#define RAINDROP_ANALOG_PIN 34

#define STATUS_INTERVAL 5000

// WiFi credentials
const char* ssid = "HT-H7608-CC1D";
const char* password = "heltec.org";

// MQTT
const char* mqtt_server = "192.168.1.137";
const int mqtt_port = 1883;

// MQTT Topics
const char* mqtt_topic_data      = "sensor/data";
const char* mqtt_topic_ota_start = "sensor/ota/start";
const char* mqtt_topic_ota_chunk = "sensor/ota/chunk";
const char* mqtt_topic_ota_done  = "sensor/ota/done";


#endif // SENSORCONFIG_H