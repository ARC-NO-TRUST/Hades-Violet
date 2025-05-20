#ifndef SENSORGLOBAL_H
#define SENSORGLOBAL_H

#include "SensorConfig.h"

// Global sensor objects
WiFiClient espClient;
PubSubClient client(espClient);
SensirionI2cScd4x scd4x;
Adafruit_BME680 bme;
SensirionI2CSen5x sen5x;

// OTA state
size_t ota_total_size = 0;
size_t ota_received_size = 0;
bool ota_in_progress = false;
bool ota_reboot_pending = false;

// FreeRTOS task handles
TaskHandle_t mqttTaskHandle;
TaskHandle_t deviceInfoTaskHandle;
TaskHandle_t sensorPublishTaskHandle;
TaskHandle_t scd40TaskHandle;
TaskHandle_t bme680TaskHandle;
TaskHandle_t mqTaskHandle;
TaskHandle_t sen55TaskHandle;

// FreeRTOS queue
QueueHandle_t sensorQueue;
QueueHandle_t jsonQueue;

SemaphoreHandle_t mqttLock;

#endif // SENSORGLOBAL_H