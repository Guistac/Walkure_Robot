#pragma once
#include <Arduino.h>

#include "Radio.h"
#include "Remote.h"
#include "ServoMotor.h"

#define PUL_1_FL 30
#define DIR_1_FL 29
#define ENA_1_FL 28
#define PEN_1_FL 31
#define ALM_1_FL 32

#define PUL_2_BL 21
#define DIR_2_BL 22
#define ENA_2_BL 23
#define PEN_2_BL 20
#define ALM_2_BL 19

#define PUL_3_FR 16
#define DIR_3_FR 17
#define ENA_3_FR 18
#define PEN_3_FR 15
#define ALM_3_FR 14

#define PUL_4_BR 38
#define DIR_4_BR 39
#define ENA_4_BR 40
#define PEN_4_BR 37
#define ALM_4_BR 36

namespace Robot{

    extern Radio radio;
    extern Remote remote;
    extern ServoMotor servoFrontLeft;
    extern ServoMotor servoBackLeft;
    extern ServoMotor servoFrontRight;
    extern ServoMotor servoBackRight;
    extern ServoMotor* servoMotors[4];


    struct Configuration{

        double radioFrequency_MHz = 434.0;
        double radioBandwidth_KHz = 125.0;
        int radioSpreadingFactor = 7;

        double highSetting_translationVelocitLimit = 0.0;
        double mediumSetting_translationVelocityLimit = 0.0;
        double lowSetting_translationVelocityLimit = 0.0;
        double highSetting_rotationVelocityLimit = 0.0;
        double mediumSetting_rotationVelocityLimit = 0.0;
        double lowSetting_rotationVelocityLimit = 0.0;

        double translationAcceleration_mmpss = 0.0;
        double rotationAcceleration_degpss = 0.0;
        double translationEmergencyDeceleration_mmps = 0.0;
        double rotationEmergencyDeceleration_mmps = 0.0;

        Vec2f frontLeft_wheelPosition;
        Vec2f backLeft_wheelPosition;
        Vec2f frontRight_wheelPosition;
        Vec2f backRight_wheelPosition;

        Vec2f frontLeft_wheelFrictionVector;
        Vec2f backLeft_wheelFrictionVector;
        Vec2f frontRight_wheelFrictionVector;
        Vec2f backRight_wheelFrictionVector;

        bool b_swapXandY = false;
        bool b_invertX = false;
        bool b_invertY = false;
        bool b_invertR = false;
    };


    void initialize(Configuration& config);
    void update();

    float getBatteryVoltage();

    enum class State{
        DISABLING = 0x1,
        DISABLED = 0x2,
        ENABLING = 0x3,
        ENABLED = 0x4,
        EMERGENCY_STOPPING = 0x5,
        EMERGENCY_STOPPED = 0x6
    };

    State getState();

    void pinTest();

};