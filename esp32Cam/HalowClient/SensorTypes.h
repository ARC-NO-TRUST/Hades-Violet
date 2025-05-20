#ifndef SENSORTYPES_H
#define SENSORTYPES_H

struct SensorData {
    float temperature = 0.0;
    float humidity = 0.0;
    float co2 = 0.0;
    float uv = 0.0;
    float pressure = 0.0;
    float iaq = 0.0;
    float wind_speed = 0.0;
    float wind_direction = 0.0;
    int co = 0;
    int battery_percent = 0;
    int battery_status = 0;
    int raindrop = 0;
    int wifi_strength = 0;
    
    float pm1p0;
    float pm2p5;
    float pm4p0;
    float pm10p0;
    float voc_index;
    float nox_index;
};

enum SensorType {
    SENSOR_DEVICE_INFO,
    SENSOR_SCD40,
    SENSOR_BME680,
    SENSOR_MQ,
    SENSOR_SEN55
};

struct SensorMessage {
    SensorType type;
    SensorData data;
};

#endif // SENSORTYPES_H
