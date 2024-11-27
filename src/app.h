#include <Arduino.h>
#include <ArduinoJson.h>

#define PROJECT_NAME "LivingroomSensor"

// DHT22 IO
#define DHT_PIN 18

// LD2410 IO
#define RADAR_TX_PIN 16
#define RADAR_RX_PIN 17
#define RADAR_PRESENCE_PIN 22

#define WIFI_CONNECT_TIMEOUT 10

// Conversion factor for micro seconds to seconds
#define uS_TO_S_FACTOR 1000000
#define uS_TO_MS_FACTOR 1000

// Time ESP32 will go to sleep (in seconds) minus the time it took for an update instance to run
#define TIME_TO_SLEEP 120