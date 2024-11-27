#include <PubSubClient.h>
#include <WiFiClient.h>

class MqttClient {
  private:
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    void (*receiveCallback)(char*, uint8_t*, unsigned int);
    
  public:
    bool connect(String username, String password, String serverAddress, int port, String deviceId, void (*callback)(char*, uint8_t*, unsigned int));
    void disconnect();
    bool isConnected();
    bool loop();
    bool publish(String mqttTopic, int value, bool retain);
    bool publish(String mqttTopic, float value, bool retain);
    bool publish(String mqttTopic, String value, bool retain);
    bool subscribeTopic(String mqttTopic);
    bool unsubscribeTopic(String mqttTopic);
};