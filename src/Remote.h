#pragma once
#include <Arduino.h>

class Remote{
public:

    bool initialize();

    bool receiveProcessData();
    void sendProcessData();

    bool b_disable = false;
    bool b_enable = false;
    uint8_t speedMode = 0;
    bool b_modeToggle = false;
    bool b_leftButton = false;
    bool b_rightButton = false;

    float leftJoystickX = 0.0;
    float leftJoystickY = 0.0;
    float rightJoystickX = 0.0;
    float rightJoystickY = 0.0;

    bool b_safetyClear = false;

    bool b_isConnected = false;
    uint32_t lastProcessDataReceiveMillis;
    uint32_t processDataTimeoutMillis = 400;
    float signalStrength = 0.0;
    uint8_t messageCounter = 0;
};