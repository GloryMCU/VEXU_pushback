#ifndef BASIC_SENSORS_SENSORS_H_
#define BASIC_SENSORS_SENSORS_H_

#include "vex.h"

namespace basic {
namespace sensors {

constexpr int kLedPower = 100;
constexpr int kLoopDelay = 50;

enum class ColorName { red, green, blue, yellow, unknown };

ColorName classifyHue(double hue);
void sensorInit();
void runsensor();

double IMUHeading();
void resetHeading();

extern int Accelerate;

}  // namespace sensors
}  // namespace basic

#endif
