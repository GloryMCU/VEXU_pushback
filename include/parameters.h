#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "vex.h"

#define RED_COLOR_HUE 15
#define BLUE_COLOR_HUE 215
#define COLOR_HUE_OFFSET 20

const float JOYSTICK_DEADZONE = 5;

#ifdef ROBOT_01
#define IMU_10 3600.0
const float BASE_FORWARD_PID[3] = {0.28, 0.05, 0.1};
const float BASE_ROTATE_PID[3] = {0.1,0.0005,0};
#endif

#endif