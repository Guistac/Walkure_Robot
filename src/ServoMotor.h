#pragma once

#include "Arduino.h"

#include "Vector2.h"

class FilteredDigitalInput{
public:

    void initialize(uint8_t pin_, uint8_t pinConfiguration){
        pin = pin_;
        pinMode(pin, pinConfiguration);
    }

    void update(){
        if(digitalRead(pin)) {
            if(readingIntegrator < integratorLimit) readingIntegrator++;
        }
        else if(readingIntegrator > -integratorLimit) readingIntegrator--;
    }

    bool isHigh(){ return readingIntegrator > integratorThreshold; }
    bool isLow(){ return readingIntegrator < integratorThreshold; }

private:

    uint8_t pin;
    uint8_t pinConfiguration;

    int8_t readingIntegrator = 0;
    uint8_t integratorLimit = 8;
    uint8_t integratorThreshold = 4;
};




class ServoMotor{
public:

    ServoMotor(uint8_t pulPin_, uint8_t dirPin_, uint8_t enaPin_, uint8_t almPin_, uint8_t penPin_){
        pulsePin = pulPin_;
        directionPin = dirPin_;
        enablePin = enaPin_;
        alarmPin = almPin_;
        pendingPin = penPin_;
    }

    Vec2f wheelPosition_mm;
    Vec2f wheelFrictionVector_mmPerRev;

    void initialize(void(*timedFunction)()){
        pinMode(pulsePin, OUTPUT);
        pinMode(directionPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
        alarmInput.initialize(alarmPin, INPUT_PULLUP);
        pendingInput.initialize(pendingPin, INPUT_PULLUP);

        disable();
        digitalWrite(pulsePin, b_pulseState);
        digitalWrite(directionPin, LOW);

        timer.begin(timedFunction, 1.0);
    }

    void update(){
        uint32_t nowMicros = micros();
        if(nowMicros - lastFeedbackUpdateMicros < feedbackUpdateIntervalMicros) return;
        lastFeedbackUpdateMicros = nowMicros;
        alarmInput.update();
        pendingInput.update();
    }

    void isr(){

        if(millis() - lastVeloctyRequest_millis > velocityRequestTimeout_millis){
            velocityTarget = 0.0;
        }

        //only update motion profile on a low pulse
        if(!b_pulseState){

            //update time delta in seconds
            uint32_t now_microseconds = micros();
            double deltaT_seconds = (double)(now_microseconds - previousIsrTimeMicros) / 1000000.0;
            previousIsrTimeMicros = now_microseconds;

            //increment velocity to reach target velocity
            float velocityIncrement = fixedAcceleration * deltaT_seconds;
            if(velocityActual < velocityTarget){
                velocityActual += velocityIncrement;
                velocityActual = min(velocityTarget, velocityActual);
            }else if(velocityActual > velocityTarget){
                velocityActual -= velocityIncrement;
                velocityActual = max(velocityTarget, velocityActual);
            }

            float actAbs = velocityActual < 0.0 ? -velocityActual : velocityActual;

            if(actAbs >= zeroVelocityTreshold){
                b_stopped = false;
                float pulseFrequency = actAbs * pulsesPerRevolution;
                float pulseEdgeInterval_microseconds = 1000000.0 / (2.0 * pulseFrequency);
                timer.update(pulseEdgeInterval_microseconds);
            }
            else {
                b_stopped = true;
                velocityActual = 0.0;
                float pulseEdgeInterval_microseconds = 1000000.0 / (2.0 * minPulseFrequency);
                timer.update(pulseEdgeInterval_microseconds);
            }
        }

        if(b_stopped) {
            b_pulseState = false;
            b_directionState = false;
        }
        else {
            b_pulseState = !b_pulseState;
            b_directionState = velocityActual > 0.0;
        }

        digitalWrite(pulsePin, b_pulseState);
        digitalWrite(directionPin, b_directionState);
    }

    void enable(){ digitalWrite(enablePin, HIGH); }
    void disable(){ digitalWrite(enablePin, LOW); }

    bool hasAlarm(){ return alarmInput.isLow(); }
    bool isEnabled(){ return pendingInput.isLow(); }

    void setVelocityTarget(float target){
        if(target > velocityLimit) velocityTarget = velocityLimit;
        else if(target < -velocityLimit) velocityTarget = -velocityLimit;
        else velocityTarget = target;
        lastVeloctyRequest_millis = millis();
    }

    float getVelocityLimit(){ return velocityLimit; }

    float getActualVelocity(){ return velocityActual; }

private:

    uint8_t pulsePin;
    uint8_t directionPin;
    uint8_t enablePin;
    uint8_t alarmPin;
    uint8_t pendingPin;
    IntervalTimer timer;

    FilteredDigitalInput alarmInput;
    FilteredDigitalInput pendingInput;

    float velocityTarget = 0.0;
    float velocityActual = 0.0;
    bool b_pulseState = false;
    bool b_directionState = false;
    bool b_stopped = true;
    uint32_t previousIsrTimeMicros = UINT32_MAX;

    uint32_t lastFeedbackUpdateMicros = UINT32_MAX;
    uint32_t feedbackUpdateIntervalMicros = 500;

    const float fixedAcceleration = 5.0;
    const uint16_t pulsesPerRevolution = 12800;
    const float velocityLimit = 2.0;
    const float zeroVelocityTreshold = 0.01;
    const float minPulseFrequency = 10.0;

    uint32_t lastVeloctyRequest_millis = 0;
    uint32_t velocityRequestTimeout_millis = 100;
};