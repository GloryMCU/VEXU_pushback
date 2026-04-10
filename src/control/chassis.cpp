#include "control/chassis.h"

#include "hardware/robot_config.h"
#include "input/controller.h"

#include <algorithm>
#include <cmath>

namespace basic {
namespace control {

namespace {

void apply_motor_power(vex::motor& base, double speed, vex::brakeType type) {
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

void Chassis::brake(vex::brakeType type) {
  state_mutex.lock();
  fl = 0;
  fr = 0;
  bl = 0;
  br = 0;
  stop_brake_type = type;
  state_mutex.unlock();

  set_motor_power();
}

void Chassis::set_motor_power() {
  state_mutex.lock();
  const double current_bl = bl;
  const double current_br = br;
  const double current_fl = fl;
  const double current_fr = fr;
  const vex::brakeType current_stop_brake_type = stop_brake_type;
  state_mutex.unlock();

  apply_motor_power(hardware::motor_bl1, current_bl, current_stop_brake_type);
  apply_motor_power(hardware::motor_bl2, current_bl, current_stop_brake_type);

  apply_motor_power(hardware::motor_br1, current_br, current_stop_brake_type);
  apply_motor_power(hardware::motor_br2, current_br, current_stop_brake_type);

  apply_motor_power(hardware::motor_fl1, current_fl, current_stop_brake_type);
  apply_motor_power(hardware::motor_fl2, current_fl, current_stop_brake_type);

  apply_motor_power(hardware::motor_fr1, current_fr, current_stop_brake_type);
  apply_motor_power(hardware::motor_fr2, current_fr, current_stop_brake_type);
}

double Chassis::dynamic_smooth(int now, int last, double rating) {
  if (std::abs(now) > hardware::kDeadZone) {
    const double k = 0.4 + 0.5 * rating;
    return now * k + last * (1 - k);
  }

  const double k = 0.7 + 0.2 * rating;
  return last * (1 - k);
}

void Chassis::run_driver_control() {
  const input::ControllerState controls = input::get_controls_snapshot();

  const double axis1 = dynamic_smooth(controls.axis1, controls.last_axis1, controls.rating[0]);
  const double axis2 = dynamic_smooth(controls.axis2, controls.last_axis2, controls.rating[1]);
  double front_left = axis2 + axis1;
  double front_right = axis2 - axis1;
  double back_left = axis2 + axis1;
  double back_right = axis2 - axis1;

  double maxpct = std::max({std::fabs(front_left), std::fabs(front_right), std::fabs(back_left), std::fabs(back_right)});
  if (maxpct > 100) {
    const double k = 100.0 / maxpct;
    front_left *= k;
    front_right *= k;
    back_left *= k;
    back_right *= k;
  }

  const double next_fl = shape_input(front_left);
  const double next_fr = shape_input(front_right);
  const double next_bl = shape_input(back_left);
  const double next_br = shape_input(back_right);

  state_mutex.lock();
  fl = next_fl;
  fr = next_fr;
  bl = next_bl;
  br = next_br;
  state_mutex.unlock();
}

void run_chassis_thread() {
  while (true) {
    Chassis::get_instance()->set_motor_power();
    vex::this_thread::sleep_for(hardware::kRefreshTime);
  }
}

}  // namespace control
}  // namespace basic
