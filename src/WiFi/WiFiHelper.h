#include <Arduino.h>

class WiFiHelper {
    private:
        WiFiHelper();

    public:
        static bool connect(String ssid, String password, String hostname);
        static void disconnect();
        static bool isConnected();
};