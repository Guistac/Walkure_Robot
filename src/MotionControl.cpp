#include "MotionControl.h"

namespace MotionControl{

    const float translVelLim_mmPerSec = 600.0;
    const float rotVelLim_degPerSec = 60.0;
    const float translAcc_mmPerSecSq = 400.0;
    const float rotAcc_degPerSecSq = 30.0;
    const float translEmergencyAcc_mmPerSecSq = 1000.0;
    const float rotEmergencyAcc_degPerSecSq = 100.0;

    float xVelTarg_mmPerSec = 0.0;
    float yVelTarg_mmPerSec = 0.0;
    float rotVelTarg_degPerSec = 0.0;
    float translAccTarget_mmPerSecSq = 0.0;
    float rotAccTarget_degPerSecSq = 0.0;

    void setControlTargetsNormalized(float x, float y, float r){
        xVelTarg_mmPerSec = map(x, -1.0, 1.0, -translVelLim_mmPerSec, translVelLim_mmPerSec);
        yVelTarg_mmPerSec = map(y, -1.0, 1.0, -translVelLim_mmPerSec, translVelLim_mmPerSec);
        rotVelTarg_degPerSec = map(r, -1.0, 1.0, -rotVelLim_degPerSec, rotVelLim_degPerSec);
        xVelTarg_mmPerSec = min(xVelTarg_mmPerSec, translVelLim_mmPerSec);
        xVelTarg_mmPerSec = max(xVelTarg_mmPerSec, -translVelLim_mmPerSec);
        yVelTarg_mmPerSec = min(yVelTarg_mmPerSec, translVelLim_mmPerSec);
        yVelTarg_mmPerSec = max(yVelTarg_mmPerSec, -translVelLim_mmPerSec);
        rotVelTarg_degPerSec = min(rotVelTarg_degPerSec, rotVelLim_degPerSec);
        rotVelTarg_degPerSec = max(rotVelTarg_degPerSec, -rotVelLim_degPerSec);
        translAccTarget_mmPerSecSq = translAcc_mmPerSecSq;
        rotAccTarget_degPerSecSq = rotAcc_degPerSecSq;
    }

    void requestEmergencyDeceleration(){
        translAccTarget_mmPerSecSq = translEmergencyAcc_mmPerSecSq;
        rotAccTarget_degPerSecSq = rotEmergencyAcc_degPerSecSq;
        xVelTarg_mmPerSec = 0.0;
        yVelTarg_mmPerSec = 0.0;
        rotVelTarg_degPerSec = 0.0;
    }

    float xVelActual_mmPerSec = 0.0;
    float yVelActual_mmPerSec = 0.0;
    float rotVelActual_degPerSec = 0.0;

    bool isStopped(){
        return xVelActual_mmPerSec == 0.0 && yVelActual_mmPerSec == 0.0 && rotVelActual_degPerSec == 0.0;
    }

    float getXVelocityNormalized(){ return map(xVelActual_mmPerSec, -translVelLim_mmPerSec, translVelLim_mmPerSec, -1.0, 1.0); }
    float getYVelocityNormalized(){ return map(yVelActual_mmPerSec, -translVelLim_mmPerSec, translVelLim_mmPerSec, -1.0, 1.0); }
    float getRotationalVelocityNormalized(){ return map(rotVelActual_degPerSec, -rotVelLim_degPerSec, rotVelLim_degPerSec, -1.0, 1.0); }

    void reset(){
        xVelTarg_mmPerSec = 0.0;
        yVelTarg_mmPerSec = 0.0;
        rotVelTarg_degPerSec = 0.0;
        xVelActual_mmPerSec = 0.0;
        yVelActual_mmPerSec = 0.0;
        rotVelActual_degPerSec = 0.0;
    }

    uint32_t lastControlLoopMicros = UINT32_MAX;
    uint32_t controlLoopIntervalMicros = 100;

    bool update(){

        uint32_t nowMicros = micros();
        if(nowMicros - lastControlLoopMicros < controlLoopIntervalMicros) return false;
        double deltaT_seconds = (double)(nowMicros - lastControlLoopMicros) / 1000000.0;
        lastControlLoopMicros = nowMicros;

        float xVelocityTarget = xVelTarg_mmPerSec;
        float yVelocityTarget = yVelTarg_mmPerSec;

        //limit the translation vector magnitude to the max translation velocity
        //this effectively transforms the xy square of the left joystick into a circle
        float translationTargetVectorMagnitude = sqrt(sq(xVelocityTarget) + sq(yVelocityTarget));
        if(translationTargetVectorMagnitude != 0.0){
            xVelocityTarget /= translationTargetVectorMagnitude;
            yVelocityTarget /= translationTargetVectorMagnitude;
            if(translationTargetVectorMagnitude > translVelLim_mmPerSec) translationTargetVectorMagnitude = translVelLim_mmPerSec;
            xVelocityTarget *= translationTargetVectorMagnitude;
            yVelocityTarget *= translationTargetVectorMagnitude;
        }
        //limit the target velocity to the velocity limit
        if(rotVelTarg_degPerSec > rotVelLim_degPerSec) rotVelTarg_degPerSec = rotVelLim_degPerSec;
        else if(rotVelTarg_degPerSec < -rotVelLim_degPerSec) rotVelTarg_degPerSec = -rotVelLim_degPerSec;

        //update motion ramp profilers
        double deltaVTranslation_millimetersPerSecond = translAccTarget_mmPerSecSq * deltaT_seconds;
        if(xVelActual_mmPerSec < xVelocityTarget){
            xVelActual_mmPerSec += deltaVTranslation_millimetersPerSecond;
            if(xVelActual_mmPerSec > xVelocityTarget) xVelActual_mmPerSec = xVelocityTarget;
        }else if(xVelActual_mmPerSec > xVelocityTarget){
            xVelActual_mmPerSec -= deltaVTranslation_millimetersPerSecond;
            if(xVelActual_mmPerSec < xVelocityTarget) xVelActual_mmPerSec = xVelocityTarget;
        }
        if(yVelActual_mmPerSec < yVelocityTarget){
            yVelActual_mmPerSec += deltaVTranslation_millimetersPerSecond;
            if(yVelActual_mmPerSec > yVelocityTarget) yVelActual_mmPerSec = yVelocityTarget;
        }else if(yVelActual_mmPerSec > yVelocityTarget){
            yVelActual_mmPerSec -= deltaVTranslation_millimetersPerSecond;
            if(yVelActual_mmPerSec < yVelocityTarget) yVelActual_mmPerSec = yVelocityTarget;
        }
        Vec2f requestedTranslationVelocityVector{xVelActual_mmPerSec, yVelActual_mmPerSec};


        double deltaVRotation_degreesPerSecond = rotAccTarget_degPerSecSq * deltaT_seconds;
        //double rotVelPrev_degreesPerSecond = rotVelActual_degPerSec;
        if(rotVelActual_degPerSec < rotVelTarg_degPerSec){
            rotVelActual_degPerSec += deltaVRotation_degreesPerSecond;
            if(rotVelActual_degPerSec > rotVelTarg_degPerSec) rotVelActual_degPerSec = rotVelTarg_degPerSec;
        }else if(rotVelActual_degPerSec > rotVelTarg_degPerSec){
            rotVelActual_degPerSec -= deltaVRotation_degreesPerSecond;
            if(rotVelActual_degPerSec < rotVelTarg_degPerSec) rotVelActual_degPerSec = rotVelTarg_degPerSec;
        }


/*
        //get the current heading by integrating angular velocity
        double averageRotationVel = (rotVelActual_degPerSec + rotVelPrev_degreesPerSecond) / 2.0;
        double rotPosDelta = averageRotationVel * deltaT_seconds;
        currentHeading_degrees += rotPosDelta * HEADING_CORRECTION_FACTOR; //multipply by a calibrated correction factor 

        //in compass mode we use the current heading to correct the angle of the translation vector
        //this way we can keep moving in a fixed direction
        if(b_compassMode){
            
            //get info about current requested velocity
            //front is 0, right is PI/2, left is -PI/2
            //the value needs to be inverted to match the real movement
            double requestedVelocityHeading_radians = -atan2(requestedTranslationVelocityVector.x, requestedTranslationVelocityVector.y);
            double requestedVelocityMagnitude_mmPerS = sqrt(sq(requestedTranslationVelocityVector.x) + sq(requestedTranslationVelocityVector.y));

            //get the current heading in normalized radians
            double currentHeading_radians = PI * 2.0 * currentHeading_degrees / 360.0;
            while(currentHeading_radians > PI) currentHeading_radians -= 2.0 * PI;
            while(currentHeading_radians < -PI) currentHeading_radians += 2.0 * PI;

            //we need to subtract the current heading angle from the requested velocity vector to keep moving in the same direction as the heading
            //at the same time subtract 90° to match the joystick angle
            double correctedVelocityHeading_radians = requestedVelocityHeading_radians + currentHeading_radians + (PI / 2.0);
            while(correctedVelocityHeading_radians < -PI) correctedVelocityHeading_radians += 2.0 * PI;
            while(correctedVelocityHeading_radians > PI) correctedVelocityHeading_radians -= 2.0 * PI;

            //get unit vector with the correct heading and set its magnitude to equal the original requested velocity
            Vector2 requestedVelocityHeadingCorrectedVector{.x = cos(correctedVelocityHeading_radians), .y = sin(correctedVelocityHeading_radians)};
            requestedVelocityHeadingCorrectedVector.x *= requestedVelocityMagnitude_mmPerS;
            requestedVelocityHeadingCorrectedVector.y *= requestedVelocityMagnitude_mmPerS;

            //copy the corrected vector to the original vector so we can apply it later
            requestedTranslationVelocityVector = requestedVelocityHeadingCorrectedVector;
        }
*/


        double wheelVelocity[4];

        //calculate target velocity for each wheel
        for(int i = 0; i < 4; i++){

            //————————————X & Y component velocity————————————
            wheelVelocity[i] = 
                requestedTranslationVelocityVector.x / Robot::servoMotors[i]->wheelFrictionVector_mmPerRev.x +
                requestedTranslationVelocityVector.y / Robot::servoMotors[i]->wheelFrictionVector_mmPerRev.y;

            //———————Rotational Component velocity————————

            //wheel position relative to rotation center
            Vec2f relativeWheelPosition = Robot::servoMotors[i]->wheelPosition_mm;

            float rotationRadius = sqrt(sq(relativeWheelPosition.x) + sq(relativeWheelPosition.y));
            float rotationCirclePerimeter = 2.0 * PI * rotationRadius;
            float rotationVectorMagnitude = rotationCirclePerimeter * rotVelActual_degPerSec / 360.0;

            //vector perpendicular to radius of wheel position around rotation center vector
            Vec2f wheelRotationVector;
            wheelRotationVector.x = -relativeWheelPosition.y;
            wheelRotationVector.y = relativeWheelPosition.x;

            //normalize the perpendicular vector
            double normalisationMagnitude = sqrt(sq(wheelRotationVector.x) + sq(wheelRotationVector.y));
            if(normalisationMagnitude != 0.0){
                wheelRotationVector.x /= normalisationMagnitude;
                wheelRotationVector.y /= normalisationMagnitude;

                //set the rotation vector magnitude to the rotation speed
                wheelRotationVector.x *= rotationVectorMagnitude;
                wheelRotationVector.y *= rotationVectorMagnitude;
            }else{
                wheelRotationVector.x = 0.0;
                wheelRotationVector.y = 0.0;
            }

            //decompose the rotation vector and add to wheel velocity
            wheelVelocity[i] += wheelRotationVector.x / Robot::servoMotors[i]->wheelFrictionVector_mmPerRev.x;
            wheelVelocity[i] += wheelRotationVector.y / Robot::servoMotors[i]->wheelFrictionVector_mmPerRev.y;
        }

        for(int i = 0; i < 4; i++) Robot::servoMotors[i]->setVelocityTarget(wheelVelocity[i]);

        return true;
    }

}