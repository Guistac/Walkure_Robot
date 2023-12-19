#include "Remote.h"
#include "Robot.h"
#include "Safety.h"
#include "MotionControl.h" 
#include "CRC.h"

bool Remote::initialize(){
    return true;
}

bool Remote::receiveProcessData(){

    if(millis() - lastProcessDataReceiveMillis > processDataTimeoutMillis) {
        if(b_isConnected){
            b_isConnected = false;
            b_safetyClear = false;
            Serial.println("——— Remote Connection Timed Out.");
        }
    }

    uint8_t incomingFrameSize = 7;
    uint8_t incomingFrame[incomingFrameSize];
    uint8_t actualFrameSize = incomingFrameSize;
    Radio::ReceptionResult result = Robot::radio.receive(incomingFrame, incomingFrameSize);

    if(incomingFrameSize != actualFrameSize){
        return false;
    }

    switch(result){
        case Radio::ReceptionResult::NOTHING_RECEIVED:
            return false;
        case Radio::ReceptionResult::BAD_CRC:
            Serial.printf("Receive Corrupted Frame (%i)\n", millis());
            return false;
        case Radio::ReceptionResult::BAD_NODEID:
            Serial.printf("Frame received from wrong nodeID (%i)\n", millis());
            return false;
        case Radio::ReceptionResult::GOOD_RECEPTION:
            //Serial.printf("Valid Frame Received (%i)\n", millis());
            break;
    }

    lastProcessDataReceiveMillis = millis();
    if(!b_isConnected){
        b_isConnected = true;
        Serial.println("——— Remote Connected.");
    }

    uint8_t controlWord = incomingFrame[0];
    b_disable =     controlWord & 0x1;
    b_enable =      controlWord & 0x2;
    speedMode =     (controlWord >> 2) & 0x3;
    b_modeToggle =  controlWord & 0x10;
    b_leftButton =  controlWord & 0x20;
    b_rightButton = controlWord & 0x40;

    if(incomingFrame[1] == 127) leftJoystickX = 0.0;
    else leftJoystickX = map((float)incomingFrame[2], 0.0, 255.0, -1.0, 1.0);
    if(incomingFrame[2] == 127) leftJoystickY = 0.0;
    else leftJoystickY = map((float)incomingFrame[3], 0.0, 255.0, -1.0, 1.0);
    if(incomingFrame[3] == 127) rightJoystickX = 0.0;
    else rightJoystickX = map((float)incomingFrame[4], 0.0, 255.0, -1.0, 1.0);
    if(incomingFrame[4] == 127) rightJoystickY = 0.0;
    else rightJoystickY = map((float)incomingFrame[5], 0.0, 255.0, -1.0, 1.0);

    uint16_t safetyNumber = incomingFrame[6] << 8 | incomingFrame[5];
    b_safetyClear = Safety::isNumberClear(safetyNumber);

    return true;
}

void Remote::sendProcessData(){

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

    uint8_t outgoingFrameSize = 7;
    uint8_t outgoingFrame[outgoingFrameSize];
    outgoingFrame[0] = robotStatusWord;
    outgoingFrame[1] = motorStatusWord;
    outgoingFrame[2] = xVelocity;
    outgoingFrame[3] = yVelocity;
    outgoingFrame[4] = rVelocity;
    outgoingFrame[5] = receivedSignalStrength;
    outgoingFrame[6] = batteryVoltage;

    //linear velocity
    //translation angle
    //rotational velocity
    //front left velocity
    //front right velocity
    //back left velocity
    //back right velocity

    Robot::radio.send(outgoingFrame, outgoingFrameSize);
}