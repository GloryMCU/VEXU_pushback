#ifndef VEX_TOOLS_H
#define VEX_TOOLS_H
#include <cmath>

int sign(float);
float deg2rad(float);
float rad2deg(float);
float reduce_negetive_180_to_180(float angle);
float reduce_0_to_360(float angle);
float clamp(float input, float min, float max);
float deadband(float input, float width);
float to_voltage(float input);

class MyTimer {
 private:
  float startTime;

 public:
  MyTimer();
  MyTimer(float);
  void reset();
  int getTime() const;
};

#endif