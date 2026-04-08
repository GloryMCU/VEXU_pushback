#include "subsystems/mechanisms.h"

#include "common/parameters.h"
#include "config/robot_config.h"
#include "input/controller_input.h"

#include <cstdlib>
#include <vector>

namespace basic {
namespace subsystems {

namespace {

void set_motor_power(vex::motor base, double speed, vex::brakeType type = vex::coast) {
  if (speed) {
    base.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
  } else {
    base.stop(type);
  }
}

void enable_motor_analog(vex::motor base, int input_value, int deadzone, int speed = 100) {
  const int dir = (input_value < 0) ? -1 : 1;
  const bool active = (std::abs(input_value) < deadzone) ? 0 : 1;
  if (active) {
    set_motor_power(base, dir * speed);
  } else {
    base.stop();
  }
}

void enable_motor_discrete(vex::motor base, bool& input_value, int speed = 100) {
  static bool motor_start = false;
  if (input_value) {
    if (!motor_start) {
      base.spin(vex::fwd, speed, vex::pct);
      motor_start = true;
    } else {
      base.stop();
      motor_start = false;
    }
    input_value = false;
  }
}

void enable_motor_discrete(std::vector<vex::motor> motors, bool& input_value, std::vector<int> speeds) {
  static bool motor_start = false;
  if (input_value) {
    if (!motor_start) {
      int count = 0;
      for (auto motor : motors) {
        motor.spin(vex::fwd, speeds[count], vex::pct);
        count += 1;
      }
      motor_start = true;
    } else {
      for (auto motor : motors) {
        motor.stop();
      }
      motor_start = false;
    }
    input_value = false;
  }
}

void update_under_overhang() {
  if (input::controls.up) {
    config::under_overhang_motor.spin(vex::fwd, -20, vex::pct);
  }
  if (input::controls.down) {
    config::under_overhang_motor.spin(vex::fwd, 20, vex::pct);
  }
  if (!input::controls.up && !input::controls.down) {
    config::under_overhang_motor.stop(vex::hold);
  }
}

void update_middle_overhang() {
  if (input::controls.left) {
    config::middle_overhang_motor.spin(vex::fwd, 20, vex::pct);
  }
  if (input::controls.right) {
    config::middle_overhang_motor.spin(vex::fwd, -20, vex::pct);
  }
  if (!input::controls.left && !input::controls.right) {
    config::middle_overhang_motor.stop(vex::hold);
  }
}

void update_upper_overhang() {
  if (input::controls.x) {
    config::up_overhang_motor.spin(vex::fwd, 50, vex::pct);
  }
  if (input::controls.b) {
    config::up_overhang_motor.spin(vex::fwd, -50, vex::pct);
  }
  if (!input::controls.x && !input::controls.b) {
    config::up_overhang_motor.stop(vex::hold);
  }
}

void update_auto_intake() {
  /*
  int k = sensors::Accelerate == 1 ? 100 : (sensors::Accelerate == -1 ? 0 : 0);
  std::vector<vex::motor> motors = {
      config::trans_motor2, config::trans_motor3, config::under_motor1, config::middle_motor1, config::up_motor1};
  std::vector<int> speeds = {
      45, 45, 100, 45, sensors::Accelerate == 1 ? 100 : (sensors::Accelerate == -1 ? 0 : 0)};
  enable_motor_discrete(motors, input::controls.press_r2, speeds);
  */
}

void update_legacy_intake() {
  std::vector<vex::motor> motors = {
      config::trans_motor2, config::trans_motor3, config::under_motor1, config::middle_motor1, config::up_motor1};
  std::vector<int> speeds = {45, 30, 100, 45, 30};
  enable_motor_discrete(motors, input::controls.press_a, speeds);
}

void update_upper_throw() {
  static std::vector<vex::motor> motors = {
      config::trans_motor1, config::trans_motor2, config::trans_motor3, config::under_motor1, config::middle_motor1,
      config::up_motor1};
  static std::vector<int> speeds = {45, 45, 45, 100, 45, -50};

  enable_motor_discrete(motors, input::controls.press_r1, speeds);
}

void update_middle_throw() {
  static std::vector<vex::motor> motors = {
      config::trans_motor1, config::trans_motor2, config::trans_motor3, config::under_motor1, config::middle_motor1};
  static std::vector<int> speeds = {45, 45, 45, 100, -60};

  enable_motor_discrete(motors, input::controls.press_l1, speeds);
}

}  // namespace

void update_driver_mechanisms() {
  update_upper_overhang();
  update_middle_overhang();
  update_under_overhang();

  update_legacy_intake();
  update_auto_intake();
  update_middle_throw();
  update_upper_throw();
}

}  // namespace subsystems
}  // namespace basic
