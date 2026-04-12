#ifndef BASIC_HARDWARE_ROBOT_CONFIG_H_
#define BASIC_HARDWARE_ROBOT_CONFIG_H_

#include "vex.h"

namespace basic::hardware {

constexpr bool kIsBlue = false;
constexpr int kRefreshTime = 10;
constexpr int kDeadZone = 10;
constexpr double kImuModCoefficient = 3599.9;
constexpr double kWheelTransitionCoefficient = 40.0;

extern vex::inertial Inertial;
extern vex::brain Brain;
extern vex::motor serial_sensor;
extern vex::controller Controller;

extern vex::motor motor_fl1;
extern vex::motor motor_fl2;
extern vex::motor motor_fr1;
extern vex::motor motor_fr2;
extern vex::motor motor_bl1;
extern vex::motor motor_bl2;
extern vex::motor motor_br1;
extern vex::motor motor_br2;

extern vex::motor middle_motor1;
extern vex::motor under_motor1;

extern vex::motor trans_motor1;
extern vex::motor trans_motor2;
extern vex::motor trans_motor3;

extern vex::motor under_overhang_motor;
extern vex::motor middle_overhang_motor;
extern vex::motor up_overhang_motor;

extern vex::motor up_motor1;

}  // namespace basic::hardware

#endif
