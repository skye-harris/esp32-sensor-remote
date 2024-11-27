#include "IrHelpers.h"

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Daikin.h>

//#include <IRremote.h>
IRDaikinESP ac(IR_TRANSMIT_PIN);

IrHelpers::AirconState IrHelpers::airconGetState() {
    AirconState airconState;

    uint16_t fanVal = ac.getFan();
    switch(fanVal) {
        case DAIKIN_FAN_AUTO:
            airconState.fanSpeed = "Auto";
            break;

        case DAIKIN_FAN_QUIET:
            airconState.fanSpeed = "Quiet";
            break;

        default:
            airconState.fanSpeed = fanVal;
            break;
    }

    airconState.temperature = ac.getTemp();
    airconState.mode = ac.getMode();
    airconState.power = ac.getPower();
    airconState.swingAuto = ac.getSwingVertical();

    return airconState;
}

void IrHelpers::airconSetState(AirconState airconState, bool send) {
    ac.begin();
    delay(50);

    if (airconState.setMode) {
        Serial.printf("Setting mode to %i\n", airconState.mode);
        ac.setMode(airconState.mode);
    }

    if (airconState.setFan) {
        int fanSpeed = DAIKIN_FAN_AUTO;
        
        if (airconState.fanSpeed.equals("auto")) {
            Serial.printf("Setting fan speed to %s\n", airconState.fanSpeed);
            fanSpeed = DAIKIN_FAN_AUTO;
        } else if (airconState.fanSpeed.equals("quiet")) {
            Serial.printf("Setting fan speed to %s\n", airconState.fanSpeed);
            fanSpeed = DAIKIN_FAN_QUIET;
        } else if (airconState.fanSpeed.equals("low")) {
            Serial.printf("Setting fan speed to %s\n", airconState.fanSpeed);
            fanSpeed = 1;
        } else if (airconState.fanSpeed.equals("medium")) {
            Serial.printf("Setting fan speed to %s\n", airconState.fanSpeed);
            fanSpeed = 3;
        } else if (airconState.fanSpeed.equals("high")) {
            Serial.printf("Setting fan speed to %s\n", airconState.fanSpeed);
            fanSpeed = 5;
        } else {
            fanSpeed = atoi(airconState.fanSpeed.c_str());
            fanSpeed = min(max(1,fanSpeed),5);
        }

        ac.setFan(fanSpeed);
    }

    if (airconState.setTemperature) {
        Serial.printf("Setting temperature to %i\n", airconState.temperature);
        ac.setTemp(airconState.temperature);
    }

    if (airconState.setPower) {
        Serial.printf("Setting power to %i\n", airconState.power);
        ac.setPower(airconState.power);
    }

    if (airconState.setSwing) {
        Serial.printf("Setting swing to %s\n", airconState.swingAuto ? "on" : "off");
        ac.setSwingVertical(airconState.swingAuto);
    }

    if (airconState.setComfort) {
        Serial.printf("Setting comfort to %s\n", airconState.comfort ? "on" : "off");
        ac.setComfort(airconState.comfort);
    }

    if (airconState.setQuiet) {
        Serial.printf("Setting quiet to %s\n", airconState.quiet ? "on" : "off");
        ac.setQuiet(airconState.quiet);
    }

    if (send) {
        Serial.printf("Setting AC\n");
        ac.send();
    }
}

// Turn the sound system for the TV on :)
void IrHelpers::soundSystemOn() {
    IrHelpers::sendNecCommand(0x11);
}

// Turn the sound system for the TV off :)
void IrHelpers::soundSystemOff() {
    IrHelpers::sendNecCommand(0x1D);
}

void IrHelpers::sendNecCommand(uint8_t command) {
    IRsend irSend(IR_TRANSMIT_PIN);
    irSend.begin();

    Serial.println("Sending NEC");
    irSend.sendNEC(irSend.encodeNEC(0x4583, command), 32);
}