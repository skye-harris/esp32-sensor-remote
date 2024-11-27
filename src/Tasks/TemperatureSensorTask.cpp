#include <app.h>
#include "TemperatureSensorTask.h"
#include "Sensors/TemperatureSensor.h"

void TemperatureSensorTask::runTask(void* args) {
    TemperatureSensorTask *self = (TemperatureSensorTask*)args;

    Serial.printf("TemperatureSensorTask running on core #%i\n",xPortGetCoreID());

    for (;;) {
        unsigned int startTime = millis();
        Serial.println("Temperature Sensor update in progress...");

        // max priority while we read the sensor, so nothing interrupts our timings
        TemperatureSensor::SensorData sensorData = TemperatureSensor::readData(TemperatureSensor::DHT_22, DHT_PIN);

        StaticJsonDocument<256> sensorAttribs;
        sensorAttribs["temperature"] = sensorData.temperature;
        sensorAttribs["humidity"] = sensorData.humidity;
        sensorAttribs["heat_index"] = round(sensorData.heatIndex*10)/10;

        String sensorAttribsJson;
        serializeJson(sensorAttribs, sensorAttribsJson);

        self->mqttClient->publish(self->mqttTopicBase + "/sensor", sensorAttribsJson.c_str(), true);

        unsigned int currentTime = millis();
        unsigned int timeDiff = currentTime - startTime;
        unsigned int pauseTime = (TIME_TO_SLEEP * 1000) - timeDiff;
        float displayTime = (float)pauseTime / 1000;

        Serial.printf("Next Temperature Sensor update in %.2f seconds\n", displayTime);
        TemperatureSensorTask::pauseCurrentTask(pauseTime);
    }

    Serial.println("TemperatureSensorTask End");
    self->kill();
}

TemperatureSensorTask::TemperatureSensorTask(MqttClient *mqttClient, String mqttTopicBase) {
    this->mqttClient = mqttClient;
    this->mqttTopicBase = mqttTopicBase;
}

void TemperatureSensorTask::start() {
    xTaskCreatePinnedToCore(
        this->runTask,          /* Function to implement the task */
        "SENSOR_TASK",          /* Name of the task */
        10000,                  /* Stack size in words */
        this,                   /* Task input parameter */
        tskIDLE_PRIORITY,       /* Priority of the task */
        &this->taskHandle,      /* Task handle. */
        0);                     /* Core where the task should run */
}

void TemperatureSensorTask::kill() {
    vTaskDelete(this->taskHandle);
}

void TemperatureSensorTask::pauseCurrentTask(unsigned int millis) {
    vTaskDelay(millis / portTICK_PERIOD_MS);
}