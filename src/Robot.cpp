#include "Robot.h"

#include "ServoMotor.h"
#include "MotionControl.h"

namespace Robot{

    Radio radio;
    Remote remote;
    ServoMotor servoFrontLeft(PUL_1_FL, DIR_1_FL, ENA_1_FL, ALM_1_FL, PEN_1_FL);
    ServoMotor servoBackLeft(PUL_2_BL, DIR_2_BL, ENA_2_BL, ALM_2_BL, PEN_2_BL);
    ServoMotor servoFrontRight(PUL_3_FR, DIR_3_FR, ENA_3_FR, ALM_3_FR, PEN_3_FR);
    ServoMotor servoBackRight(PUL_4_BR, DIR_4_BR, ENA_4_BR, ALM_4_BR, PEN_4_BR);
    ServoMotor* servoMotors[4] = {
        &servoFrontLeft,
        &servoBackLeft,
        &servoFrontRight,
        &servoBackRight
    };

    Configuration configuration;

    bool areAllServosEnabled(){
        for(int i = 0; i < 4; i++){
            if(!servoMotors[i]->isEnabled()) return false;
        }
        return true;
    }
    bool areAllServosAlarmFree(){
        for(int i = 0; i < 4; i++){
            if(servoMotors[i]->hasAlarm()) return false;
        }
        return true;
    }
    void disableAllMotors(){ for(int i = 0; i < 4; i++) servoMotors[i]->disable(); }
    void enableAllMotors(){ for(int i = 0; i < 4; i++) servoMotors[i]->enable(); }
    
    State robotState = State::EMERGENCY_STOPPED;
    uint32_t enableRequestMillis;
    float batteryVoltage = 0.0;

    bool b_swapXandY = false;
    bool b_invertX = false;
    bool b_invertY = false;
    bool b_invertR = false;

    State getState(){ return robotState; } 
    float getBatteryVoltage(){ return batteryVoltage; }


    void startupError(){
        pinMode(13, OUTPUT);
        while(true){
            digitalWrite(13, HIGH);
            delay(250);
            digitalWrite(13, LOW);
            delay(250);
        }
    }


    void initialize(Configuration& config){

        //while(!Serial){}
        Serial.println("———— Robot Start ————");

        configuration = config;

        if(!radio.initialize(config.radioFrequency_MHz, config.radioBandwidth_KHz, config.radioSpreadingFactor)) {
            Serial.println("Unable to initialize radio.");
            startupError();
        }
        if(!remote.initialize()) {
            Serial.println("Unable to initialize remote");
            startupError();
        }

        MotionControl::Configuration motionConfiguration = {
            .highSetting_translationVelocitLimit_mmps = config.highSetting_translationVelocitLimit,
            .mediumSetting_translationVelocityLimit_mmps = config.mediumSetting_translationVelocityLimit,
            .lowSetting_translationVelocityLimit_mmps = config.lowSetting_translationVelocityLimit,
            .highSetting_rotationVelocityLimit_degps = config.highSetting_rotationVelocityLimit,
            .mediumSetting_rotationVelocityLimit_degps = config.mediumSetting_rotationVelocityLimit,
            .lowSetting_rotationVelocityLimit_degps = config.lowSetting_rotationVelocityLimit,
            .translationAcceleration_mmpss = config.translationAcceleration_mmpss,
            .rotationAcceleration_degpss = config.rotationAcceleration_degpss,
            .translationEmergencyDeceleration_mmpss = config.translationEmergencyDeceleration_mmps,
            .rotationEmergencyDeceleration_degpss = config.rotationEmergencyDeceleration_mmps
        };
        if(!MotionControl::initialize(motionConfiguration)){
            Serial.println("Unable to initialize Motion Control");
            startupError();
        }

        servoFrontLeft.initialize([]{ servoFrontLeft.isr(); });
        servoBackLeft.initialize([]{ servoBackLeft.isr(); });
        servoFrontRight.initialize([]{ servoFrontRight.isr(); });
        servoBackRight.initialize([]{ servoBackRight.isr(); });
        servoFrontLeft.wheelPosition_mm =  config.frontLeft_wheelPosition;
        servoBackLeft.wheelPosition_mm =   config.backLeft_wheelPosition;
        servoFrontRight.wheelPosition_mm = config.frontRight_wheelPosition;
        servoBackRight.wheelPosition_mm =  config.backRight_wheelPosition;
        servoFrontLeft.wheelFrictionVector_mmPerRev =   config.frontLeft_wheelFrictionVector;
        servoBackLeft.wheelFrictionVector_mmPerRev =    config.backLeft_wheelFrictionVector;
        servoFrontRight.wheelFrictionVector_mmPerRev =  config.frontRight_wheelFrictionVector;
        servoBackRight.wheelFrictionVector_mmPerRev =   config.backRight_wheelFrictionVector;
        for(int i = 0; i < 4; i++) servoMotors[i]->disable();

        pinMode(25, INPUT);
        analogReadResolution(12);
        
        Serial.println("Robot initialized.");
    }


    void update(){

        for(int i = 0; i < 4; i++) servoMotors[i]->update();

        //voltage divider is 460 / 10000 ohm
        float r1 = 464.0;
        float r2 = 9830.0;
        batteryVoltage = map(float(analogRead(25)), 0.0, 4096.0, 0.0, 3.3) * (r1+r2) / r1;
        //Serial.println(batteryVoltage);

        bool b_processDataReceived = remote.receiveProcessData();

        if(!remote.b_safetyClear){
            if(robotState != State::EMERGENCY_STOPPED && robotState != State::EMERGENCY_STOPPING){
                MotionControl::requestEmergencyDeceleration();
                robotState = State::EMERGENCY_STOPPING;
                Serial.println("——— Emergency Stop Requested.");
            }
        }

        switch(Robot::remote.speedMode){
            case 0:
                MotionControl::setSpeedMode(MotionControl::SpeedMode::MEDIUM_SPEED);
                break;
            case 1:
                MotionControl::setSpeedMode(MotionControl::SpeedMode::HIGH_SPEED);
                break;
            case 2:
                MotionControl::setSpeedMode(MotionControl::SpeedMode::LOW_SPEED);
                break;
        }

        switch(robotState){
            case State::ENABLING:{
                uint32_t timeSinceEnableRequestMillis = millis() - enableRequestMillis;
                bool b_timeout = timeSinceEnableRequestMillis > 2000;
                if(areAllServosEnabled() && areAllServosAlarmFree() && timeSinceEnableRequestMillis > 1500){
                    robotState = State::ENABLED;
                    Serial.printf("——— Robot Enabled. (took %ims)\n", timeSinceEnableRequestMillis);
                }
                else if(remote.b_disable || !remote.b_isConnected || timeSinceEnableRequestMillis > 2000){
                    disableAllMotors();
                    robotState = State::DISABLED;
                    if(b_timeout)                   Serial.println("——— Enable Request Timed Out.");
                    else if(!remote.b_isConnected)  Serial.println("——— Enable Request Aborted : remote was disconnected.");
                    else if(remote.b_disable)       Serial.println("——— Cancelled Enable Request.");
                }
            }break;
            case State::ENABLED:{
                if(!areAllServosAlarmFree() || !areAllServosEnabled() || remote.b_disable || !remote.b_isConnected){
                    MotionControl::setControlTargetsNormalized(0.0, 0.0, 0.0);
                    robotState = State::DISABLING;
                    if(!areAllServosAlarmFree())        Serial.println("——— Disabling Robot : Not all servos were alarm free.");
                    else if(!areAllServosAlarmFree())   Serial.println("——— Disabling Robot : Not all servos were enabled.");
                    else if(!remote.b_isConnected)      Serial.println("——— Disabling Robot : Remote was disconnected.");
                    else if(remote.b_disable)           Serial.println("——— Disabling Robot by user request.");
                }
                else {
                    float xCommand = remote.leftJoystickX;
                    float yCommand = remote.leftJoystickY;
                    float rCommand = remote.rightJoystickX;
                    if(configuration.b_swapXandY) std::swap(xCommand, yCommand);
                    if(configuration.b_invertX) xCommand = -xCommand;
                    if(configuration.b_invertY) yCommand = -yCommand;
                    if(configuration.b_invertR) rCommand = -rCommand;
                    MotionControl::setControlTargetsNormalized(xCommand, yCommand, rCommand);
                }
            }break;
            case State::DISABLING:{
                MotionControl::setControlTargetsNormalized(0.0, 0.0, 0.0);
                if(MotionControl::isStopped()){
                    for(int i = 0; i < 4; i++) servoMotors[i]->disable();
                    robotState = State::DISABLED;
                    Serial.println("——— Robot Disabled.");
                }
            }break;
            case State::DISABLED:{
                MotionControl::reset();
                if(remote.b_enable) {
                    for(int i = 0; i < 4; i++) servoMotors[i]->enable();
                    enableRequestMillis = millis();
                    robotState = State::ENABLING;
                    Serial.println("——— Enabling Robot ...");
                }
            }break;
            case State::EMERGENCY_STOPPING:{
                MotionControl::requestEmergencyDeceleration();
                if(MotionControl::isStopped()){
                    for(int i = 0; i < 4; i++) servoMotors[i]->disable();
                    robotState = State::EMERGENCY_STOPPED;
                    Serial.println("——— Emergency Stopped.");
                }
            }break;
            case State::EMERGENCY_STOPPED:{
                MotionControl::reset();
                for(int i = 0; i < 4; i++) servoMotors[i]->disable();
                if(remote.b_safetyClear) {
                    robotState = State::DISABLED;
                    Serial.println("——— Emergency Stop Released.");
                }
            }break;
        }
        
        MotionControl::update();

        if(b_processDataReceived) remote.sendProcessData();

    }



    void pinTest(){

        struct DigitalPin{
            DigitalPin(uint8_t pin_, const char* name_) : pin(pin_), name(name_){}
            uint8_t pin;
            const char* name;
        };

        DigitalPin inputPins[20] = {
            DigitalPin(PUL_1_FL, "PUL_1_FL"),
            DigitalPin(DIR_1_FL, "DIR_1_FL"),
            DigitalPin(ENA_1_FL, "ENA_1_FL"),
            DigitalPin(PEN_1_FL, "PEN_1_FL"),
            DigitalPin(ALM_1_FL, "ALM_1_FL"),
            DigitalPin(PUL_2_BL, "PUL_2_BL"),
            DigitalPin(DIR_2_BL, "DIR_2_BL"),
            DigitalPin(ENA_2_BL, "ENA_2_BL"),
            DigitalPin(PEN_2_BL, "PEN_2_BL"),
            DigitalPin(ALM_2_BL, "ALM_2_BL"),
            DigitalPin(PUL_3_FR, "PUL_3_FR"),
            DigitalPin(DIR_3_FR, "DIR_3_FR"),
            DigitalPin(ENA_3_FR, "ENA_3_FR"),
            DigitalPin(PEN_3_FR, "PEN_3_FR"),
            DigitalPin(ALM_3_FR, "ALM_3_FR"),
            DigitalPin(PUL_4_BR, "PUL_4_BR"),
            DigitalPin(DIR_4_BR, "DIR_4_BR"),
            DigitalPin(ENA_4_BR, "ENA_4_BR"),
            DigitalPin(PEN_4_BR, "PEN_4_BR"),
            DigitalPin(ALM_4_BR, "ALM_4_BR")
        };

        for(int i = 0; i < 20; i++) pinMode(inputPins[i].pin, OUTPUT);

        while(true){
            for(int i = 0; i < 20; i++){
                digitalWrite(inputPins[i].pin, HIGH);
                Serial.println(inputPins[i].name);
                delay(3000);
                digitalWrite(inputPins[i].pin, LOW);
            }
        }

    }

};