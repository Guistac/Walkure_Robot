#pragma once

#include <Arduino.h>

#include "Robot.h"
#include "Vector2.h"

namespace MotionControl{

    void setControlTargetsNormalized(float x, float y, float r);
    void requestEmergencyDeceleration();
    void reset();

    bool isStopped();
    float getXVelocityNormalized();
    float getYVelocityNormalized();
    float getRotationalVelocityNormalized();

    bool update();

};