#include "Sensors/TemperatureSensor.h"
#include "WiFi/WiFiHelper.h"
#include "Sensors/IRHelpers.h"

#include "Tasks/TemperatureSensorTask.h"
#include <ir_Daikin.h>
#include <ld2410.h>

#include <Preferences.h>
#include "app.h"

// Ideally these would be stored on flash, but this is really just for my own personal
const char *wifiSsid = "";
const char *wifiPassword = "";

// Ditto for these
String mqttUser = "";
String mqttPass = "";
String mqttAddress = "";
String mqttDeviceId = PROJECT_NAME;
String mqttTopicBase = strlwr(PROJECT_NAME);
const int mqttPort = 1883;

TemperatureSensorTask *temperatureSensorTask;
MqttClient mqttClient;

struct RadarData {
    bool presenceDetected = false;
    bool sendImmediate = true;

    uint16_t stationaryTargetDistance = 0;
    uint16_t movingTargetDistance = 0;

    uint8_t stationaryTargetEnergy = 0;
    uint8_t movingTargetEnergy = 0;
};

ld2410 radar;
RadarData radarData;

// Connect to the LD2410 via UART
bool tryLd2410(int rx, int tx) {
    Serial1.begin(256000, SERIAL_8N1, rx, tx);
    delay(500);
    Serial.print(F("\nLD2410 radar sensor initialising: "));
    if (radar.begin(Serial1)) {
        Serial.println(F("OK"));

        return true;
    }

    Serial.println(F("Failed"));

    return false;
}

// Publish the current LD2410 gate configuration to mqtt
void reportRadarSettings() {
    if(radar.requestCurrentConfiguration()) {
        StaticJsonDocument<1024> radarAttribs;

        radarAttribs["max_moving_gate"] = radar.max_moving_gate;
        radarAttribs["max_stationary_gate"] = radar.max_stationary_gate;
        radarAttribs["sensor_idle_time"] = radar.sensor_idle_time;

        for (uint8_t gate = 0; gate <= 8; gate++) {
            char sensorAttrib[38];

            sprintf(sensorAttrib, "gate_%i_motion_sensitivity\0", gate);
            radarAttribs[sensorAttrib] = radar.motion_sensitivity[gate];

            sprintf(sensorAttrib, "gate_%i_stationary_sensitivity\0", gate);
            radarAttribs[sensorAttrib] = radar.stationary_sensitivity[gate];
        }

        String radarAttribsJson;
        serializeJson(radarAttribs, radarAttribsJson);

        String mqttTopic = mqttTopicBase + "/presence/config";
        mqttClient.publish(mqttTopic, radarAttribsJson.c_str(), false);
    }
}

// Persist our aircon state to flash, in case of device reboot/power-loss
void persistAirconState() {
    IrHelpers::AirconState airconState = IrHelpers::airconGetState();

    // save last aircon state
    Preferences preferences;
    preferences.begin("irsensor", false);

    preferences.putBool("swingAuto", airconState.swingAuto);
    preferences.putBool("comfort", airconState.comfort);
    preferences.putBool("quiet", airconState.quiet);
    preferences.putString("fanSpeed", airconState.fanSpeed);
    preferences.putUInt("temperature", airconState.temperature);
    preferences.putUInt("mode", airconState.mode);

    Serial.println(airconState.temperature);
    preferences.end();
}

// Load our persisted aircon state from flash
void loadAirconState() {
    Preferences preferences;
    preferences.begin("irsensor", true);

    IrHelpers::AirconState airconState;
    airconState.setSwing = true;
    airconState.swingAuto = preferences.getBool("swingAuto",true);

    airconState.setFan = true;
    airconState.fanSpeed = preferences.getString("fanSpeed","auto");

    airconState.setTemperature = true;
    airconState.temperature = preferences.getUInt("temperature",24);

    airconState.setComfort = true;
    airconState.comfort = preferences.getBool("comfort",false);

    airconState.setQuiet = true;
    airconState.quiet = preferences.getBool("quiet",false);

    airconState.setMode = true;
    airconState.mode = preferences.getUInt("mode",kDaikinAuto);

    airconState.setPower = false;
    IrHelpers::airconSetState(airconState, false);

    preferences.end();
}

// Publish current aircon state to MQTT
void reportCurrentAirconState() {
    StaticJsonDocument<256> airconAttribs;
    IrHelpers::AirconState airconState = IrHelpers::airconGetState();

    airconAttribs["temperature"] = airconState.temperature;
    airconAttribs["swing"] = airconState.swingAuto ? "on" : "off";
    airconAttribs["fan"] = airconState.fanSpeed;

    switch (airconState.mode) {
        case kDaikinCool:
            airconAttribs["mode"] = "cool";
            break;

        case kDaikinHeat:
            airconAttribs["mode"] = "heat";
            break;

        case kDaikinDry:
            airconAttribs["mode"] = "dry";
            break;

        case kDaikinFan:
            airconAttribs["mode"] = "fan_only";
            break;

        case kDaikinAuto:
            airconAttribs["mode"] = "auto";
            break;
    }

    if (!airconState.power) {
        airconAttribs["mode"] = "off";
    }

    String airconAttribsJson;
    serializeJson(airconAttribs, airconAttribsJson);

    mqttClient.publish(mqttTopicBase+"/aircon/state", airconAttribsJson.c_str(), true);
}

// Process MQTT messages
void mqttTopicReceived(char *topic, uint8_t *payload, unsigned int length) {
    char buffer[length+1];
    memset(&buffer,0,length+1);
    memcpy(&buffer,payload,length);
    Serial.printf("Received message '%s' from topic '%s'\n", buffer, topic);

    DynamicJsonDocument jsonPayload(1024);
    DeserializationError error = deserializeJson(jsonPayload, buffer);
    String topicStr = String(topic);

    if (topicStr.equalsIgnoreCase(mqttTopicBase+"/aircon")) {
        IrHelpers::AirconState airconState;

        if (jsonPayload.containsKey("swing")) {
            airconState.setSwing = true;
            airconState.swingAuto = jsonPayload["swing"] == "on";
        }

        if (jsonPayload.containsKey("fan")) {
            airconState.setFan = true;
            airconState.fanSpeed = jsonPayload["fan"].as<String>();
            airconState.fanSpeed.toLowerCase();
        }

        if (jsonPayload.containsKey("temperature")) {
            airconState.setTemperature = true;
            airconState.temperature = jsonPayload["temperature"].as<int>();
        }

        if (jsonPayload.containsKey("comfort")) {
            airconState.setComfort = true;
            airconState.comfort = jsonPayload["comfort"] == "on";
        }

        if (jsonPayload.containsKey("quiet")) {
            airconState.setQuiet = true;
            airconState.quiet = jsonPayload["quiet"] == "on";
        }

        if (jsonPayload.containsKey("mode")) {
            airconState.setMode = true;
            airconState.setPower = true;
            airconState.power = true;

            String mode = jsonPayload["mode"].as<String>();
            mode.toLowerCase();
            
            if (mode == "heat") {
                airconState.mode = kDaikinHeat;
            } else if (mode == "cool") {
                airconState.mode = kDaikinCool;
            } else if (mode == "dry") {
                airconState.mode = kDaikinDry;
            } else if (mode == "fan" || mode == "fan_only") {
                airconState.mode = kDaikinFan;
            } else if (mode == "auto") {
                airconState.mode = kDaikinAuto;
            } else {
                airconState.setMode = false;
                airconState.power = false;
            }
        }

        IrHelpers::airconSetState(airconState, true);
    
        persistAirconState();
        reportCurrentAirconState();
    }

    else if (topicStr.equalsIgnoreCase(mqttTopicBase+"/presence/config/reset")) {
        radar.requestFactoryReset();
        delay(500);
        radar.requestRestart();
        delay(500);
        
        reportRadarSettings();
    }

    else if (topicStr.equalsIgnoreCase(mqttTopicBase+"/presence/config/get")) {
        reportRadarSettings();
    }

    else if (topicStr.equalsIgnoreCase(mqttTopicBase+"/presence/config/set")) {
        for (uint8_t gate = 0; gate <= 8; gate++) {
            char sensorAttrib[38];

            uint8_t motionSensitivity = radar.motion_sensitivity[gate];
            uint8_t stationarySensitivity = radar.stationary_sensitivity[gate];

            sprintf(sensorAttrib, "gate_%i_motion_sensitivity\0", gate);
            if (jsonPayload.containsKey(sensorAttrib)) {
                motionSensitivity = jsonPayload[sensorAttrib].as<uint8_t>();
            }

            sprintf(sensorAttrib, "gate_%i_stationary_sensitivity\0", gate);
            if (jsonPayload.containsKey(sensorAttrib)) {
                stationarySensitivity = jsonPayload[sensorAttrib].as<uint8_t>();
            }

            radar.setGateSensitivityThreshold(gate, motionSensitivity, stationarySensitivity);
        }

        uint8_t maxMovingGate = radar.max_moving_gate;
        uint8_t maxStationaryGate = radar.max_stationary_gate;
        uint8_t sensorIdleTime = radar.sensor_idle_time;

        if (jsonPayload.containsKey("max_moving_gate")) {
            maxMovingGate = jsonPayload["max_moving_gate"].as<uint8_t>();
        }

        if (jsonPayload.containsKey("max_stationary_gate")) {
            maxStationaryGate = jsonPayload["max_stationary_gate"].as<uint8_t>();
        }

        if (jsonPayload.containsKey("sensor_idle_time")) {
            sensorIdleTime = jsonPayload["sensor_idle_time"].as<uint8_t>();
        }

        radar.setMaxValues(maxMovingGate, maxStationaryGate, sensorIdleTime);
        
        delay(100);
        radar.requestRestart();

        delay(500);
        reportRadarSettings();
    }

    else if (topicStr.equalsIgnoreCase(mqttTopicBase+"/sound")) {
        if (jsonPayload.containsKey("command")) {
            if (jsonPayload["command"] == "on") {
                IrHelpers::soundSystemOn();
            } else if (jsonPayload["command"] == "off") {
                IrHelpers::soundSystemOff();
            }
        }
    }

    delay(10);
}

// Ensure our WiFi & MQTT connections are active and havent dropped
void ensureWiFiConnected() {
    bool wifiConnected = WiFiHelper::isConnected();

    if (!wifiConnected) {
        // Attempt wifi connection
        wifiConnected = WiFiHelper::connect(wifiSsid, wifiPassword, PROJECT_NAME);

        if (!wifiConnected) {
            // we've lost our WiFi connection and are unable to reconnect, lets wait a few seconds before the next loop retries
            delay(5000);

            return;
        }
    }

    // We have wifi, however mqtt is disconnected
    if (wifiConnected && !mqttClient.isConnected()) {
        mqttClient.connect(mqttUser, mqttPass, mqttAddress, mqttPort, PROJECT_NAME, mqttTopicReceived);
        
        // Resubscribe to our topics
        mqttClient.subscribeTopic(mqttTopicBase+"/aircon");
        mqttClient.subscribeTopic(mqttTopicBase+"/sound");
        mqttClient.subscribeTopic(mqttTopicBase+"/presence/config/get");
        mqttClient.subscribeTopic(mqttTopicBase+"/presence/config/set");
        mqttClient.subscribeTopic(mqttTopicBase+"/presence/config/reset");
    }
}

// Publish the current target detection data that we retrieved via UART from the LD2410
void publishRadarValues() {
    StaticJsonDocument<2048> radarAttribs;

    radarAttribs["presence_detected"] = radarData.presenceDetected;
    radarAttribs["stationary_target_distance"] = radarData.stationaryTargetDistance;
    radarAttribs["stationary_target_energy"] = radarData.stationaryTargetEnergy;
    radarAttribs["moving_target_distance"] = radarData.movingTargetDistance;
    radarAttribs["moving_target_energy"] = radarData.movingTargetEnergy;

    String radarAttribsJson;
    serializeJson(radarAttribs, radarAttribsJson);

    mqttClient.publish(mqttTopicBase + "/presence", radarAttribsJson, false);
}

// Check for presence updates
void radarLoop() {
    bool thisIsDetected = digitalRead(RADAR_PRESENCE_PIN);

    radar.read();
    if (radar.isConnected()) {
        // If UART is connected, read data from the sensor
        radarData.stationaryTargetDistance = radar.stationaryTargetDistance();
        radarData.stationaryTargetEnergy = radar.stationaryTargetEnergy();
        radarData.movingTargetDistance = radar.movingTargetDistance();
        radarData.movingTargetEnergy = radar.movingTargetEnergy();
    }

    if (thisIsDetected != radarData.presenceDetected
         || radarData.sendImmediate == true) {
        radarData.presenceDetected = thisIsDetected;

        // Publish our update to MQTT
        publishRadarValues();
        radarData.sendImmediate = false;
    }
}

void setup() {
    Serial.begin(115200);
    ensureWiFiConnected();

    // Load the persisted aircon state from flash
    loadAirconState();

    // Start the temperature/humidity sensor task
    temperatureSensorTask = new TemperatureSensorTask(&mqttClient, mqttTopicBase);
    temperatureSensorTask->start();

    // UART for monitoring the ld2410 radar
    if (!tryLd2410(RADAR_RX_PIN, RADAR_TX_PIN)) {
        Serial.println("Failed to connect to LD2410 Sensor via Serial. Check the RX/TX connections.");
    }

    // We'll be using the binary presence detection output the radar as well
    pinMode(RADAR_PRESENCE_PIN, INPUT);
}

void loop() {
    // Lets ensure wifi has not dropped out at all, reconnect if it has, and run our mqtt & radar loops
    ensureWiFiConnected();
    mqttClient.loop();

    // Todo: move to a Task
    radarLoop();

    delay(50);
}