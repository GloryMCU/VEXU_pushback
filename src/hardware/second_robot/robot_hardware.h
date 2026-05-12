#ifndef BASIC_SRC_HARDWARE_SECOND_ROBOT_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_SECOND_ROBOT_ROBOT_HARDWARE_H_

#include "vex.h"

namespace basic::hardware::second_robot {

inline constexpr int kRefreshTime = 10;
inline constexpr int kDeadZone = 10;
inline constexpr int kSensorLoopDelay = 50;

struct RobotHardware {
  vex::motor left_front_motor{vex::PORT1, vex::ratio6_1, false};
  vex::motor left_middle_motor{vex::PORT2, vex::ratio6_1, false};
  vex::motor left_back_motor{vex::PORT3, vex::ratio6_1, false};

  vex::motor right_front_motor{vex::PORT6, vex::ratio6_1, true};
  vex::motor right_middle_motor{vex::PORT13, vex::ratio6_1, true};
  vex::motor right_back_motor{vex::PORT17, vex::ratio6_1, true};

  vex::motor roller_lower_motor{vex::PORT14, vex::ratio6_1, true};
  vex::motor roller_middle_motor{vex::PORT15, vex::ratio6_1, false};
  vex::motor roller_upper_motor{vex::PORT19, vex::ratio6_1, true};

  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  vex::inertial inertial{vex::PORT7};
  vex::digital_out descore{brain.ThreeWirePort.F};
  vex::digital_out hook{brain.ThreeWirePort.E};
  vex::digital_out store{brain.ThreeWirePort.H};

  void calibrate_inertial_sensor() {
    inertial.calibrate();
    while (inertial.isCalibrating()) {
      vex::wait(5, vex::msec);
    }

    inertial.resetHeading();
    inertial.resetRotation();
  }

  void show_calibrated() {
    controller.Screen.setCursor(5, 1);
    controller.Screen.print("      calibrated!");
  }
};

}  // namespace basic::hardware::second_robot

#endif
