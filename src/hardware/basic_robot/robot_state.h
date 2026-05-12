#ifndef BASIC_SRC_HARDWARE_BASIC_ROBOT_ROBOT_STATE_H_
#define BASIC_SRC_HARDWARE_BASIC_ROBOT_ROBOT_STATE_H_

#include "hardware/shared/state_types.h"

namespace basic::hardware::basic_robot {

enum class IndexedMechanismMode {
  kOff,
  kLegacyIntake,
  kUnderTrow,
  kMiddleThrow,
  kUpperThrow,
};

enum class OverhangMode {
  Collapse,
  Expansion,
};

struct ChassisState {
  double fl{0.0};
  double fr{0.0};
  double bl{0.0};
  double br{0.0};
  vex::brakeType stop_brake_type{vex::coast};
};

struct MechanismState {
  IndexedMechanismMode indexed_mode{IndexedMechanismMode::kOff};
};

struct OverhangState {
  OverhangMode upper_overhang_mode{OverhangMode::Collapse};
  OverhangMode middle_overhang_mode{OverhangMode::Collapse};
  OverhangMode under_overhang_mode{OverhangMode::Collapse};
};

struct RobotState {
  basic::hardware::shared::ControllerInputState controller;
  basic::hardware::shared::SensorState sensors;
  ChassisState chassis;
  MechanismState mechanism;
  OverhangState overhang;
  basic::hardware::shared::AutonomousState autonomous;
};

}  // namespace basic::hardware::basic_robot

#endif
