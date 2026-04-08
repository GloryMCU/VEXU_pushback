#include "subsystems/drive.h"

#include "common/parameters.h"
#include "config/robot_config.h"
#include "input/controller_input.h"

#include <algorithm>
#include <cmath>

namespace basic {
namespace subsystems {

namespace {

void set_motor_power(vex::motor base, double speed, vex::brakeType type) {
  if (speed) {
    base.spin(vex::directionType::fwd, speed, vex::percentUnits::pct);
  } else {
    base.stop(type);
  }
}

double shape_input(double input) {
  const bool negative = input < 0;
  const double normalized = std::abs(input) * 0.01;
  const double shaped = normalized * normalized * (3 - 2 * normalized) * 100.0;
  return negative ? -shaped : shaped;
}

}  // namespace

void Chassis::Brake(vex::brakeType type) {
  fl = 0;
  fr = 0;
  bl = 0;
  br = 0;

  config::motor_bl1.stop(type);
  config::motor_bl2.stop(type);
  config::motor_br1.stop(type);
  config::motor_br2.stop(type);
  config::motor_fl1.stop(type);
  config::motor_fl2.stop(type);
  config::motor_fr1.stop(type);
  config::motor_fr2.stop(type);
}

void Chassis::SetMotorPower() {
  set_motor_power(config::motor_bl1, bl, stop_brake_type);
  set_motor_power(config::motor_bl2, bl, stop_brake_type);

  set_motor_power(config::motor_br1, br, stop_brake_type);
  set_motor_power(config::motor_br2, br, stop_brake_type);

  set_motor_power(config::motor_fl1, fl, stop_brake_type);
  set_motor_power(config::motor_fl2, fl, stop_brake_type);

  set_motor_power(config::motor_fr1, fr, stop_brake_type);
  set_motor_power(config::motor_fr2, fr, stop_brake_type);
}

double Chassis::dynamicSmooth(int now, int last, double rating) {
  if (std::abs(now) > common::kDeadZone) {
    const double k = 0.4 + 0.5 * rating;
    return now * k + last * (1 - k);
  }

  const double k = 0.7 + 0.2 * rating;
  return last * (1 - k);
}

void Chassis::OmniChassisControl() {
  input_cache[1] = dynamicSmooth(input::controls.axis1, input::controls.last_axis1, input::controls.rating[0]);
  input_cache[2] = dynamicSmooth(input::controls.axis2, input::controls.last_axis2, input::controls.rating[1]);
  input_cache[3] = dynamicSmooth(input::controls.axis3, input::controls.last_axis3, input::controls.rating[2]);
  input_cache[4] = dynamicSmooth(input::controls.axis4, input::controls.last_axis4, input::controls.rating[3]);

  front_left = input_cache[2] + input_cache[1];
  front_right = input_cache[2] - input_cache[1];
  back_left = input_cache[2] + input_cache[1];
  back_right = input_cache[2] - input_cache[1];

  double maxpct = std::max({std::fabs(front_left), std::fabs(front_right), std::fabs(back_left), std::fabs(back_right)});
  if (maxpct > 100) {
    const double k = 100.0 / maxpct;
    front_left *= k;
    front_right *= k;
    back_left *= k;
    back_right *= k;
  }

  fl = shape_input(front_left);
  fr = shape_input(front_right);
  bl = shape_input(back_left);
  br = shape_input(back_right);
}

void chassis_updating_thread() {
  while (true) {
    Chassis::getInstance()->SetMotorPower();
    vex::this_thread::sleep_for(common::kRefreshTime);
  }
}

}  // namespace subsystems
}  // namespace basic
