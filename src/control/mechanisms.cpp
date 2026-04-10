#include "control/chassis.h"

#include "hardware/robot_config.h"
#include "input/controller.h"

#include <array>

namespace basic {
namespace control {

namespace {

enum class IndexedMechanismMode {
  kOff,
  kLegacyIntake,
  kMiddleThrow,
  kUpperThrow,
};

constexpr std::size_t kIndexedMotorCount = 6;

const std::array<vex::motor*, kIndexedMotorCount> kIndexedMotors = {{
    &hardware::trans_motor1,
    &hardware::trans_motor2,
    &hardware::trans_motor3,
    &hardware::under_motor1,
    &hardware::middle_motor1,
    &hardware::up_motor1,
}};

const std::array<int, kIndexedMotorCount> kOffSpeeds = {{0, 0, 0, 0, 0, 0}};
const std::array<int, kIndexedMotorCount> kLegacyIntakeSpeeds = {{0, 45, 30, 100, 45, 30}};
const std::array<int, kIndexedMotorCount> kMiddleThrowSpeeds = {{45, 45, 45, 100, -60, 0}};
const std::array<int, kIndexedMotorCount> kUpperThrowSpeeds = {{45, 45, 45, 100, 45, -50}};

IndexedMechanismMode indexed_mode = IndexedMechanismMode::kOff;

void set_motor_power(vex::motor& base, double speed, vex::brakeType type = vex::coast) {
  if (speed) {
    base.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
  } else {
    base.stop(type);
  }
}

void toggle_indexed_mode(IndexedMechanismMode requested_mode) {
  if (indexed_mode == requested_mode) {
    indexed_mode = IndexedMechanismMode::kOff;
  } else {
    indexed_mode = requested_mode;
  }
}

const std::array<int, kIndexedMotorCount>& get_indexed_motor_speeds() {
  switch (indexed_mode) {
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

void apply_indexed_mode() {
  const std::array<int, kIndexedMotorCount>& speeds = get_indexed_motor_speeds();
  for (std::size_t index = 0; index < kIndexedMotorCount; ++index) {
    set_motor_power(*kIndexedMotors[index], speeds[index]);
  }
}

void update_under_overhang(const input::ControllerState& controls) {
  if (controls.up) {
    hardware::under_overhang_motor.spin(vex::fwd, -20, vex::pct);
  }
  if (controls.down) {
    hardware::under_overhang_motor.spin(vex::fwd, 20, vex::pct);
  }
  if (!controls.up && !controls.down) {
    hardware::under_overhang_motor.stop(vex::hold);
  }
}

void update_middle_overhang(const input::ControllerState& controls) {
  if (controls.left) {
    hardware::middle_overhang_motor.spin(vex::fwd, 20, vex::pct);
  }
  if (controls.right) {
    hardware::middle_overhang_motor.spin(vex::fwd, -20, vex::pct);
  }
  if (!controls.left && !controls.right) {
    hardware::middle_overhang_motor.stop(vex::hold);
  }
}

void update_upper_overhang(const input::ControllerState& controls) {
  if (controls.x) {
    hardware::up_overhang_motor.spin(vex::fwd, 50, vex::pct);
  }
  if (controls.b) {
    hardware::up_overhang_motor.spin(vex::fwd, -50, vex::pct);
  }
  if (!controls.x && !controls.b) {
    hardware::up_overhang_motor.stop(vex::hold);
  }
}

void update_indexed_mechanisms(const input::ControllerState& controls) {
  if (controls.press_a) {
    toggle_indexed_mode(IndexedMechanismMode::kLegacyIntake);
  }
  if (controls.press_l1) {
    toggle_indexed_mode(IndexedMechanismMode::kMiddleThrow);
  }
  if (controls.press_r1) {
    toggle_indexed_mode(IndexedMechanismMode::kUpperThrow);
  }

  apply_indexed_mode();
}

}  // namespace

void update_driver_mechanisms() {
  const input::ControllerState controls = input::get_controls_snapshot();

  update_upper_overhang(controls);
  update_middle_overhang(controls);
  update_under_overhang(controls);
  update_indexed_mechanisms(controls);
}

void run_driver_control_loop() {
  while (true) {
    Chassis::get_instance()->run_driver_control();
    update_driver_mechanisms();

    vex::this_thread::sleep_for(hardware::kRefreshTime);
  }
}

void start_driver_control() {
  static vex::thread user_control_task(run_driver_control_loop);
}

}  // namespace control
}  // namespace basic
