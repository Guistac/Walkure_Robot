#include "Remote.h"
#include "Robot.h"
#include "Safety.h"
#include "MotionControl.h" 
#include "CRC.h"

bool Remote::initialize(){
    return true;
}

bool Remote::receiveProcessData(){
    uint8_t incomingFrame[12];

    bool frameReceived = false;

    if(Robot::radio.receive(incomingFrame, 12)) {

        frameReceived = true;
        uint8_t* processDataBuffer = &incomingFrame[0]; //8 bytes in here
        uint8_t* safetyDataBuffer = &incomingFrame[8]; //4 bytes in here

        //———— Process Data
        uint16_t processDataCRC = processDataBuffer[7] << 8 | processDataBuffer[6];
        if(processDataCRC == calcCRC16(processDataBuffer, 6)){
            uint8_t controlWord = processDataBuffer[0];
            b_disable =  controlWord & 0x1;
            b_enable =   controlWord & 0x2;
            b_speedToggle =    controlWord & 0x4;
            modeToggle =    (controlWord & 0x18) >> 3;
            b_sleepToggle =    controlWord |= 0x20;
            if(processDataBuffer[1] == 127) leftJoystickX = 0.0;
            else leftJoystickX = map((float)processDataBuffer[1], 0.0, 255.0, -1.0, 1.0);
            if(processDataBuffer[2] == 127) leftJoystickY = 0.0;
            else leftJoystickY = map((float)processDataBuffer[2], 0.0, 255.0, -1.0, 1.0);
            if(processDataBuffer[3] == 127) rightJoystickX = 0.0;
            else rightJoystickX = map((float)processDataBuffer[3], 0.0, 255.0, -1.0, 1.0);
            if(processDataBuffer[4] == 127) rightJoystickY = 0.0;
            else rightJoystickY = map((float)processDataBuffer[4], 0.0, 255.0, -1.0, 1.0);
            messageCounter = processDataBuffer[5];
            lastProcessDataReceiveMillis = millis();

            if(!b_isConnected){
                b_isConnected = true;
                Serial.println("——— Remote Connected.");
            }
        }

        //———— Safety Data
        uint16_t safetyDataCRC = safetyDataBuffer[3] << 8 | safetyDataBuffer[2];
        if(calcCRC16(safetyDataBuffer, 2) == safetyDataCRC){
            uint16_t safetyNumber = safetyDataBuffer[1] << 8 | safetyDataBuffer[0];
            b_safetyClear = Safety::isNumberClear(safetyNumber);
            lastSafetyDataReceiveMillis = millis();
        }

    }

    if(millis() - lastProcessDataReceiveMillis > processDataTimeoutMillis) {
        if(b_isConnected){
            b_isConnected = false;
            Serial.println("——— Remote Connection Timed Out.");
        }
    }
    if(millis() - lastSafetyDataReceiveMillis > safetyDataTimeoutMillis){
        if(b_safetyClear){
            b_safetyClear = false;
            Serial.println("——— Safety Data Timed Out.");;
        }
    } 

    return frameReceived;
}

void Remote::sendProcessData(){

    uint8_t outgoingFrame[10];

    uint8_t robotStatusWord = uint8_t(Robot::getState()) & 0xF;
    bool b_modeDisplay = false;
    bool b_speedModeDisplay = false;
    if(b_modeDisplay)       robotStatusWord |= 0x10;
    if(b_speedModeDisplay)  robotStatusWord |= 0x20;

    uint8_t motorStatusWord = 0x0;
    if(Robot::servoFrontLeft.hasAlarm())    motorStatusWord |= 0x1;
    if(Robot::servoBackLeft.hasAlarm())     motorStatusWord |= 0x2;
    if(Robot::servoFrontRight.hasAlarm())   motorStatusWord |= 0x4;
    if(Robot::servoBackRight.hasAlarm())    motorStatusWord |= 0x8;
    if(Robot::servoFrontLeft.isEnabled())   motorStatusWord |= 0x10;
    if(Robot::servoBackLeft.isEnabled())    motorStatusWord |= 0x20;
    if(Robot::servoFrontRight.isEnabled())  motorStatusWord |= 0x40;
    if(Robot::servoBackRight.isEnabled())   motorStatusWord |= 0x80;

    int8_t xVelocity = map(MotionControl::getXVelocityNormalized(), -1.0, 1.0, -127, 127);
    int8_t yVelocity = map(MotionControl::getYVelocityNormalized(), -1.0, 1.0, -127, 127);
    int8_t rVelocity = map(MotionControl::getRotationalVelocityNormalized(), -1.0, 1.0, -127, 127);

    uint8_t receivedSignalStrength = Robot::radio.getSignalStrength() + 150;
    uint8_t batteryVoltage = millis() % 255;

    outgoingFrame[0] = robotStatusWord;
    outgoingFrame[1] = motorStatusWord;
    outgoingFrame[2] = xVelocity;
    outgoingFrame[3] = yVelocity;
    outgoingFrame[4] = rVelocity;
    outgoingFrame[5] = receivedSignalStrength;
    outgoingFrame[6] = batteryVoltage;
    outgoingFrame[7] = messageCounter;

    uint16_t processDataCRC = calcCRC16(outgoingFrame, 8);
    outgoingFrame[8] = processDataCRC;
    outgoingFrame[9] = processDataCRC >> 8;

    Robot::radio.send(outgoingFrame, 10);
}