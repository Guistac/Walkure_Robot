#pragma once

#include <Arduino.h>

#include "Robot.h"
#include "Vector2.h"

namespace MotionControl{

    enum class SpeedMode{
        LOW_SPEED,
        MEDIUM_SPEED,
        HIGH_SPEED
    };

    struct Configuration{
        double highSetting_translationVelocitLimit_mmps = 0.0;
        double mediumSetting_translationVelocityLimit_mmps = 0.0;
        double lowSetting_translationVelocityLimit_mmps = 0.0;
        double highSetting_rotationVelocityLimit_degps = 0.0;
        double mediumSetting_rotationVelocityLimit_degps = 0.0;
        double lowSetting_rotationVelocityLimit_degps = 0.0;
        double translationAcceleration_mmpss = 0.0;
        double rotationAcceleration_degpss = 0.0;
        double translationEmergencyDeceleration_mmpss = 0.0;
        double rotationEmergencyDeceleration_degpss = 0.0;
    };

    bool initialize(Configuration& config);

    void setControlTargetsNormalized(float x, float y, float r);
    void setSpeedMode(SpeedMode mode);
    void requestEmergencyDeceleration();
    void reset();

    bool isStopped();
    float getXVelocityNormalized();
    float getYVelocityNormalized();
    float getRotationalVelocityNormalized();

    bool update();

};