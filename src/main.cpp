#include <Arduino.h>

#include "Robot.h"

double wheelDiameter_mm = 203.0;
double wheelCircumference_mm = PI * wheelDiameter_mm;
float frictionVectorUnit = sin(45.0) * wheelCircumference_mm;

Robot::Configuration bed_robot = {
  .radioFrequency_MHz = 520.0,
  .radioBandwidth_KHz = 250.0,
  .radioSpreadingFactor = 8,
  .highSetting_translationVelocitLimit = 400.0,
  .mediumSetting_translationVelocityLimit = 200.0,
  .lowSetting_translationVelocityLimit = 100.0,
  .highSetting_rotationVelocityLimit = 30.0,
  .mediumSetting_rotationVelocityLimit = 15.0,
  .lowSetting_rotationVelocityLimit = 7.0,
  .translationAcceleration_mmpss = 200.0,
  .rotationAcceleration_degpss = 15.0,
  .translationEmergencyDeceleration_mmps = 1000.0,
  .rotationEmergencyDeceleration_mmps = 80.0,
  .frontLeft_wheelPosition = Vec2f(-291.5, -917.5),
  .backLeft_wheelPosition = Vec2f(-291.5, 917.5),
  .frontRight_wheelPosition = Vec2f(291.5, -917.5),
  .backRight_wheelPosition = Vec2f(291.5, 917.5),
  .frontLeft_wheelFrictionVector = Vec2f(frictionVectorUnit, frictionVectorUnit),
  .backLeft_wheelFrictionVector = Vec2f(-frictionVectorUnit, frictionVectorUnit),
  .frontRight_wheelFrictionVector = Vec2f(frictionVectorUnit, -frictionVectorUnit),
  .backRight_wheelFrictionVector = Vec2f(-frictionVectorUnit, -frictionVectorUnit),
  .b_swapXandY = false,
  .b_invertX = true,
  .b_invertY = false,
  .b_invertR = false
};

Robot::Configuration desk_robot = {
  .radioFrequency_MHz = 485.0,
  .radioBandwidth_KHz = 125.0,
  .radioSpreadingFactor = 7,
  .highSetting_translationVelocitLimit = 400.0,
  .mediumSetting_translationVelocityLimit = 200.0,
  .lowSetting_translationVelocityLimit = 100.0,
  .highSetting_rotationVelocityLimit = 30.0,
  .mediumSetting_rotationVelocityLimit = 15.0,
  .lowSetting_rotationVelocityLimit = 7.0,
  .translationAcceleration_mmpss = 200.0,
  .rotationAcceleration_degpss = 15.0,
  .translationEmergencyDeceleration_mmps = 1000.0,
  .rotationEmergencyDeceleration_mmps = 80.0,
  .frontLeft_wheelPosition = Vec2f(-137.5, -845.5),
  .backLeft_wheelPosition = Vec2f(-137.5, 845.5),
  .frontRight_wheelPosition = Vec2f(137.5, -845.5),
  .backRight_wheelPosition = Vec2f(137.5, 845.5),
  .frontLeft_wheelFrictionVector = Vec2f(frictionVectorUnit, frictionVectorUnit),
  .backLeft_wheelFrictionVector = Vec2f(-frictionVectorUnit, frictionVectorUnit),
  .frontRight_wheelFrictionVector = Vec2f(frictionVectorUnit, -frictionVectorUnit),
  .backRight_wheelFrictionVector = Vec2f(-frictionVectorUnit, -frictionVectorUnit),
  .b_swapXandY = false,
  .b_invertX = false,
  .b_invertY = false,
  .b_invertR = false
};

Robot::Configuration fridge_robot = {
  .radioFrequency_MHz = 450.0,
  .radioBandwidth_KHz = 250.0,
  .radioSpreadingFactor = 8,
  .highSetting_translationVelocitLimit = 400.0,
  .mediumSetting_translationVelocityLimit = 200.0,
  .lowSetting_translationVelocityLimit = 100.0,
  .highSetting_rotationVelocityLimit = 30.0,
  .mediumSetting_rotationVelocityLimit = 15.0,
  .lowSetting_rotationVelocityLimit = 7.0,
  .translationAcceleration_mmpss = 200.0,
  .rotationAcceleration_degpss = 15.0,
  .translationEmergencyDeceleration_mmps = 1000.0,
  .rotationEmergencyDeceleration_mmps = 80.0,
  .frontLeft_wheelPosition = Vec2f(-137.5, -845.5),
  .backLeft_wheelPosition = Vec2f(-137.5, 845.5),
  .frontRight_wheelPosition = Vec2f(137.5, -845.5),
  .backRight_wheelPosition = Vec2f(137.5, 845.5),
  .frontLeft_wheelFrictionVector = Vec2f(frictionVectorUnit, frictionVectorUnit),
  .backLeft_wheelFrictionVector = Vec2f(-frictionVectorUnit, frictionVectorUnit),
  .frontRight_wheelFrictionVector = Vec2f(frictionVectorUnit, -frictionVectorUnit),
  .backRight_wheelFrictionVector = Vec2f(-frictionVectorUnit, -frictionVectorUnit),
  .b_swapXandY = true,
  .b_invertX = false,
  .b_invertY = false,
  .b_invertR = true
};

Robot::Configuration closet_robot = {
  .radioFrequency_MHz = 415.0,
  .radioBandwidth_KHz = 125.0,
  .radioSpreadingFactor = 7,
  .highSetting_translationVelocitLimit = 400.0,
  .mediumSetting_translationVelocityLimit = 200.0,
  .lowSetting_translationVelocityLimit = 100.0,
  .highSetting_rotationVelocityLimit = 30.0,
  .mediumSetting_rotationVelocityLimit = 15.0,
  .lowSetting_rotationVelocityLimit = 7.0,
  .translationAcceleration_mmpss = 200.0,
  .rotationAcceleration_degpss = 15.0,
  .translationEmergencyDeceleration_mmps = 1000.0,
  .rotationEmergencyDeceleration_mmps = 80.0,
  .frontLeft_wheelPosition = Vec2f(-291.5, -917.5),
  .backLeft_wheelPosition = Vec2f(-291.5, 917.5),
  .frontRight_wheelPosition = Vec2f(291.5, -917.5),
  .backRight_wheelPosition = Vec2f(291.5, 917.5),
  .frontLeft_wheelFrictionVector = Vec2f(frictionVectorUnit, frictionVectorUnit),
  .backLeft_wheelFrictionVector = Vec2f(-frictionVectorUnit, frictionVectorUnit),
  .frontRight_wheelFrictionVector = Vec2f(frictionVectorUnit, -frictionVectorUnit),
  .backRight_wheelFrictionVector = Vec2f(-frictionVectorUnit, -frictionVectorUnit),
  .b_swapXandY = true,
  .b_invertX = true,
  .b_invertY = true,
  .b_invertR = true
};


void setup(){
  //Robot::initialize(bed_robot);
  //Robot::initialize(desk_robot);
  //Robot::initialize(fridge_robot);
  Robot::initialize(closet_robot);
}

void loop(){
  Robot::update();
}
