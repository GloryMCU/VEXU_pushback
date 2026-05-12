#ifndef BASIC_SRC_HARDWARE_SECOND_ROBOT_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_SECOND_ROBOT_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"

namespace basic::hardware::second_robot {

enum class ShooterMode {
  kOff,
  kRoller,
  kMiddleShot,
  kLongShot,
};

struct ChassisState {
  double left_pct{0.0};
  double right_pct{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct MechanismState {
  ShooterMode shooter_mode{ShooterMode::kOff};
  double shooter_speed_pct{0.0};
  bool descore_open{false};
  bool hook_open{false};
  bool store_open{false};
};

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::hardware::shared::SensorState sensors;
  ChassisState chassis;
  MechanismState mechanism;
  basic::hardware::shared::AutonomousState autonomous;
};

}  // namespace basic::hardware::second_robot

#endif
