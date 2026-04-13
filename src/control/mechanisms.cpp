#include "control/mechanisms.h"
#include "control/motor_control.h"

#include <array>

namespace basic::hardware::robots {

namespace {

constexpr std::size_t kIndexedMotorCount = 6;
constexpr std::array<int, kIndexedMotorCount> kOffSpeeds{{0, 0, 0, 0, 0, 0}};
constexpr std::array<int, kIndexedMotorCount> kLegacyIntakeSpeeds{{0, 45, 30, 100, 45, 30}};
constexpr std::array<int, kIndexedMotorCount> kMiddleThrowSpeeds{{45, 45, 45, 100, -60, 0}};
constexpr std::array<int, kIndexedMotorCount> kUpperThrowSpeeds{{45, 45, 45, 100, 45, -50}};

void set_motor_power(vex::motor& motor, double speed, vex::brakeType type = vex::coast) {
  if (speed) {
    velocitycontrol(motor, speed, vex::pct);
  } else {
    stopcontrol(motor, type);
  }
}

void toggle_indexed_mode(MechanismState& mechanism, IndexedMechanismMode requested_mode) {
  if (mechanism.indexed_mode == requested_mode) {
    mechanism.indexed_mode = IndexedMechanismMode::kOff;
  } else {
    mechanism.indexed_mode = requested_mode;
  }
}

const std::array<int, kIndexedMotorCount>& indexed_motor_speeds(const MechanismState& mechanism) {
  switch (mechanism.indexed_mode) {
    case IndexedMechanismMode::kLegacyIntake:
      return kLegacyIntakeSpeeds;
    case IndexedMechanismMode::kMiddleThrow:
      return kMiddleThrowSpeeds;
    case IndexedMechanismMode::kUpperThrow:
      return kUpperThrowSpeeds;
    case IndexedMechanismMode::kOff:
    default:
      return kOffSpeeds;
  }
}

void apply_indexed_mode(RobotHardware& hardware, const MechanismState& mechanism) {
  const std::array<vex::motor*, kIndexedMotorCount> motors{{
      &hardware.trans_motor1,
      &hardware.trans_motor2,
      &hardware.trans_motor3,
      &hardware.under_motor1,
      &hardware.middle_motor1,
      &hardware.up_motor1,
  }};
  const std::array<int, kIndexedMotorCount>& speeds = indexed_motor_speeds(mechanism);
  for (std::size_t index = 0; index < kIndexedMotorCount; ++index) {
    set_motor_power(*motors[index], speeds[index]);
  }
}

void update_under_overhang(RobotHardware& hardware, const ControllerInputState& input) {
  if (input.up) {
    velocitycontrol(hardware.under_overhang_motor, -20, vex::pct);
  }
  if (input.down) {
    velocitycontrol(hardware.under_overhang_motor, 20, vex::pct);
  }
  if (!input.up && !input.down) {
    stopcontrol(hardware.under_overhang_motor, vex::hold);
  }
}

void update_middle_overhang(RobotHardware& hardware, const ControllerInputState& input) {
  if (input.left) {
    velocitycontrol(hardware.middle_overhang_motor, 20, vex::pct);
  }
  if (input.right) {
    velocitycontrol(hardware.middle_overhang_motor, -20, vex::pct);
  }
  if (!input.left && !input.right) {
    stopcontrol(hardware.middle_overhang_motor, vex::hold);
  }
}

void update_upper_overhang(RobotHardware& hardware, const ControllerInputState& input) {
  if (input.x) {
    velocitycontrol(hardware.up_overhang_motor, 50, vex::pct);
  }
  if (input.b) {
    velocitycontrol(hardware.up_overhang_motor, -50, vex::pct);
  }
  if (!input.x && !input.b) {
    stopcontrol(hardware.up_overhang_motor, vex::hold);
  }
}

}  // namespace

void mechanism_update(RobotHardware& hardware, RobotState& state) {
  const ControllerInputState& input = state.controller;
  MechanismState& mechanism = state.mechanism;

  update_upper_overhang(hardware, input);
  update_middle_overhang(hardware, input);
  update_under_overhang(hardware, input);

  if (input.press_a) {
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kLegacyIntake);
  }
  if (input.press_l1) {
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kMiddleThrow);
  }
  if (input.press_r1) {
    toggle_indexed_mode(mechanism, IndexedMechanismMode::kUpperThrow);
  }

  apply_indexed_mode(hardware, mechanism);
}

}  // namespace basic::hardware::robots
