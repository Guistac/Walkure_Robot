#pragma once
#include <Arduino.h>

class Remote{
public:

    bool initialize();

    bool receiveProcessData();
    void sendProcessData();

    bool b_disable = false;
    bool b_enable = false;
    bool b_speedToggle = false;
    bool b_sleepToggle = false;
    uint8_t modeToggle = 0;
    float leftJoystickX = 0.0;
    float leftJoystickY = 0.0;
    float rightJoystickX = 0.0;
    float rightJoystickY = 0.0;
    bool b_safetyClear = false;

    uint32_t lastProcessDataReceiveMillis;
    uint32_t processDataTimeoutMillis = 250;

    uint32_t lastSafetyDataReceiveMillis;
    uint32_t safetyDataTimeoutMillis = 500;

    bool b_isConnected = false;
    float signalStrength = 0.0;
    float signalToNoise = 0.0;

    uint8_t messageCounter = 0;

};