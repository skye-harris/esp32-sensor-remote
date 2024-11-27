#include <Arduino.h>
#include "MQTT/MqttClient.h"

class TemperatureSensorTask {
    public:
        void start();
        void kill();
        TemperatureSensorTask(MqttClient *mqttClient, String mqttTopicBase);

    private:
        MqttClient *mqttClient;
        TaskHandle_t taskHandle;
        String mqttTopicBase;

        static void runTask(void* args);
        static void pauseCurrentTask(unsigned int millis);
        static void endCurrentTask();
};
