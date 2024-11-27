#include "TemperatureSensor.h"

TemperatureSensor::SensorData TemperatureSensor::readData(SensorType sensorType, const int gpioPin) {
    SensorData output;
    DHT dht(gpioPin, sensorType);
    dht.begin();
    UBaseType_t currentPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY+(configMAX_PRIORITIES-1));

    output.temperature = dht.readTemperature();
    output.humidity = dht.readHumidity();
    output.heatIndex = dht.computeHeatIndex(output.temperature, output.humidity, false);

    vTaskPrioritySet(NULL, currentPriority);
    Serial.printf("Sensor readings: T=%.1f H=%.1f\n", output.temperature, output.humidity);

    return output;
}