#include "vex-tools.h"

int sign(float _input) {
  if (_input > 0)
    return 1;
  else if (_input < 0)
    return -1;
  else
    return 0;
}

float deg2rad(float deg) { return deg / 180.0 * M_PI; }

float rad2deg(float rad) { return rad / M_PI * 180.0; }

float reduce_negetive_180_to_180(float angle){
  while(!(angle >= -180 && angle < 180)){
    if(angle >= 180){
      angle -= 360;
    }else if(angle < -180){
      angle += 360;
    } 
  }
  return angle;
}

float reduce_0_to_360(float angle){
  while(!(angle >= 0 && angle < 360)){
    if(angle >= 360){
      angle -= 360;
    }else if(angle < 0){
      angle += 360;
    } 
  }
  return angle;
}

// @return input if it is larger than min, smaller than max
float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

// @return zero if the value is smaller than the input value
float deadband(float input, float width){
  if (fabs(input)<width){
    return(0);
  }
  return(input);
}

float to_voltage(float input){
  return (input*12.7/100.0);
}

#include "robot-config.h"

MyTimer::MyTimer() { startTime = Brain.Timer.value(); }

MyTimer::MyTimer(float init) { startTime = Brain.Timer.value() + init / 1000; }

void MyTimer::reset() { startTime = Brain.Timer.value(); }

/**
 * @return time (msec) from startTime
 */
int MyTimer::getTime() const { return floor((Brain.Timer.value() - startTime) * 1000); }