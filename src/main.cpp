#include <Arduino.h>

#include "Robot.h"

void setup(){
  Robot::initialize();
}

void loop(){
  Robot::update();
}
