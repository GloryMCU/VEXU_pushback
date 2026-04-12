#ifndef BASIC_HARDWARE_SENSORS_H_
#define BASIC_HARDWARE_SENSORS_H_

#include "vex.h"

namespace basic::hardware {

constexpr int kLedPower = 100;
constexpr int kSensorLoopDelay = 50;

enum class ColorName { red, green, blue, yellow, unknown };

ColorName classify_hue(double hue);
void initialize_sensors();
void run_sensor_thread();

double imu_heading();
void reset_heading();
void show_calibrated();
void show_sensor_color(char color_code);

extern int Accelerate;
extern char CurrentColorCode;

}  // namespace basic::hardware

#endif
