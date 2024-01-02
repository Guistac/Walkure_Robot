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

    uint8_t incomingFrame[11];
    if(!Robot::radio.receive(incomingFrame, 11)) return false;

    //———— Process Data
    uint16_t receivedCRC = incomingFrame[10] << 8 | incomingFrame[9];

    if(receivedCRC != calcCRC16(incomingFrame, 9)) {
        Serial.printf("Receive Corrupted Frame (%i)\n", millis());
        return false;
    }

    uint8_t expectedNodeID = incomingFrame[0];
    uint8_t nodeID = int(Robot::radio.getFrequency() * 10.0) % 255;

    if(expectedNodeID != nodeID){
        Serial.printf("Frame received from wrong nodeID %i and not %i (%i)\n", nodeID, expectedNodeID, millis());
        return false;
    }

    if(false) Serial.printf("%i Valid Frame Received (N°%i)\n", millis(), incomingFrame[6]);

    lastProcessDataReceiveMillis = millis();
    if(!b_isConnected){
        b_isConnected = true;
        Serial.println("——— Remote Connected.");
    }

    uint8_t controlWord = incomingFrame[1];
    b_disable =     controlWord & 0x1;
    b_enable =      controlWord & 0x2;
    speedMode =     (controlWord >> 2) & 0x3;
    b_modeToggle =  controlWord & 0x10;
    b_leftButton =  controlWord & 0x20;
    b_rightButton = controlWord & 0x40;

    if(incomingFrame[2] == 127) leftJoystickX = 0.0;
    else leftJoystickX = map((float)incomingFrame[2], 0.0, 255.0, -1.0, 1.0);
    if(incomingFrame[3] == 127) leftJoystickY = 0.0;
    else leftJoystickY = map((float)incomingFrame[3], 0.0, 255.0, -1.0, 1.0);
    if(incomingFrame[4] == 127) rightJoystickX = 0.0;
    else rightJoystickX = map((float)incomingFrame[4], 0.0, 255.0, -1.0, 1.0);
    if(incomingFrame[5] == 127) rightJoystickY = 0.0;
    else rightJoystickY = map((float)incomingFrame[5], 0.0, 255.0, -1.0, 1.0);

    messageCounter = incomingFrame[6];

    uint16_t safetyNumber = incomingFrame[8] << 8 | incomingFrame[7];
    b_safetyClear = Safety::isNumberClear(safetyNumber);

    return true;
}

void Remote::sendProcessData(){

    uint8_t outgoingFrame[14];

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

    auto getServoVelEightBits = [](ServoMotor& servo, bool invert)->uint8_t{
        float norm = servo.getActualVelocity() / servo.getVelocityLimit();
        if(invert) norm *= -1.0;
        int8_t s_integer = norm * 128;
        if(s_integer == 0){
            if(norm > 0.0) s_integer = 1;
            else if(norm < 0.0) s_integer = -1;
        }
        return s_integer + 128;
    };
    uint8_t fl_vel = getServoVelEightBits(Robot::servoFrontLeft, false);
    uint8_t bl_vel = getServoVelEightBits(Robot::servoBackLeft, true);
    uint8_t fr_vel = getServoVelEightBits(Robot::servoFrontRight, false);
    uint8_t br_vel = getServoVelEightBits(Robot::servoBackRight, true);

    uint8_t receivedSignalStrength = Robot::radio.getSignalStrength() + 150;
    uint8_t nodeID = int(Robot::radio.getFrequency() * 10) % 255;

    outgoingFrame[0] = nodeID;
    outgoingFrame[1] = messageCounter;
    outgoingFrame[2] = receivedSignalStrength;
    outgoingFrame[3] = robotStatusWord;
    outgoingFrame[4] = motorStatusWord;
    outgoingFrame[5] = xVelocity;
    outgoingFrame[6] = yVelocity;
    outgoingFrame[7] = rVelocity;
    outgoingFrame[8] = fl_vel;
    outgoingFrame[9] = bl_vel;
    outgoingFrame[10] = fr_vel;
    outgoingFrame[11] = br_vel;

    uint16_t processDataCRC = calcCRC16(outgoingFrame, 12);
    outgoingFrame[12] = processDataCRC;
    outgoingFrame[13] = processDataCRC >> 8;

    Robot::radio.send(outgoingFrame, 14);
}