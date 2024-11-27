#include "MqttClient.h"
#include "app.h"

bool MqttClient::loop() {
    return mqttClient.loop();
}

bool MqttClient::connect(String username, String password, String serverAddress, int port, String deviceId, void (*callback)(char*, uint8_t*, unsigned int)) {    
    if (mqttClient.connected()) {
        return true;
    }

    Serial.print("Connecting to MQTT...");

    mqttClient.setClient(wifiClient);
    mqttClient.setServer(serverAddress.c_str(), port);
    mqttClient.setCallback(callback);
    mqttClient.setBufferSize(1024);

    if (mqttClient.connect(deviceId.c_str(), username.c_str(), password.c_str())) {
        Serial.println("Connected!");
        return true;
    }

    Serial.println("Failed");
    return false;
}

void MqttClient::disconnect() {
    mqttClient.disconnect();

    Serial.println("Disconnected from MQTT");
}

bool MqttClient::isConnected() {
    return mqttClient.connected();
}

bool MqttClient::publish(String mqttTopic, int value, bool retain) {
    char buffer[8];

    sprintf(buffer, "%i", value);
    return publish(mqttTopic, buffer, retain);
}

bool MqttClient::publish(String mqttTopic, float value, bool retain) {
    char buffer[8];

    sprintf(buffer, "%.1f", value);
    return publish(mqttTopic, buffer, retain);
}

bool MqttClient::publish(String mqttTopic, String value, bool retain) {
    bool result = mqttClient.publish(mqttTopic.c_str(), value.c_str(), retain);
    String resultStr = result ? "Successfully published" : "Failed to publish";

    Serial.printf("%s to %s: %s\n", resultStr.c_str(), mqttTopic.c_str(), value.c_str());

    return result;
}

bool MqttClient::subscribeTopic(String mqttTopic) {
    Serial.println("Subscribing to mqtt topic: " + mqttTopic);
    return mqttClient.subscribe(mqttTopic.c_str());
}

bool MqttClient::unsubscribeTopic(String mqttTopic) {
    return mqttClient.unsubscribe(mqttTopic.c_str());
}