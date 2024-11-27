#include "WiFiHelper.h"

#include <WiFi.h>
#include <WiFiClient.h>

#include "app.h"

bool WiFiHelper::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiHelper::connect(String ssid, String password, String hostname) {
    WiFi.setHostname(hostname.c_str());
    WiFi.mode(WIFI_STA);

    //check wi-fi is connected to wi-fi network
    unsigned long startMillis = millis();
    Serial.print("Connecting to WiFi...");    
    WiFi.begin(ssid.c_str(), password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
        delay(10);
        if (millis() - startMillis > 10000) {
            Serial.println("Failed after 10 seconds");

            return false;
        }
    }
    
    unsigned long durationMillis = millis() - startMillis;
    Serial.printf("Connected in %lu millis!\n",durationMillis);
    Serial.print("DHCP Assigned IP: ");
    Serial.println(WiFi.localIP());

    return true;
}

void WiFiHelper::disconnect() {
    WiFi.disconnect();

    Serial.println("Disconnected from WiFi");
}