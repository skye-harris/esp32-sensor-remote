#include <Arduino.h>

#define IR_TRANSMIT_PIN 23

class IrHelpers {
    public:
        struct AirconState {
            bool setTemperature = false;
            uint8_t temperature = 0;

            bool setMode = false;
            uint8_t mode = 0;

            bool setFan = false;
            String fanSpeed = "auto";

            bool setEconomy = false;
            bool economy = false;

            bool setPower = false;
            bool power = false;

            bool setSwing = false;
            bool swingAuto = false;

            bool setComfort = false;
            bool comfort = false;

            bool setQuiet = false;
            bool quiet = false;
        };

        static const uint16_t SOUND_SYSTEM_ADDRESS = 0x4583;

        static void airconSetState(AirconState airconState, bool send = true);
        static IrHelpers::AirconState airconGetState();

        static void soundSystemOn();
        static void soundSystemOff();

    private:
        static void sendNecCommand(uint8_t command);
};
