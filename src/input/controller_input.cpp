#include "input/controller_input.h"

#include "common/parameters.h"
#include "config/robot_config.h"

#include <cmath>

namespace basic {
namespace input {

namespace {

void calculate_button_rating() {
  controls.rating[0] = std::abs(controls.axis1 - controls.last_axis1) * 0.005;
  controls.rating[1] = std::abs(controls.axis2 - controls.last_axis2) * 0.005;
  controls.rating[2] = std::abs(controls.axis3 - controls.last_axis3) * 0.005;
  controls.rating[3] = std::abs(controls.axis4 - controls.last_axis4) * 0.005;
}

}  // namespace

ControllerState controls;

void input_updating_thread() {
  while (true) {
    controls.last_axis1 = controls.axis1;
    controls.last_axis2 = controls.axis2;
    controls.last_axis3 = controls.axis3;
    controls.last_axis4 = controls.axis4;

    controls.last_l1 = controls.l1;
    controls.last_l2 = controls.l2;
    controls.last_r1 = controls.r1;
    controls.last_r2 = controls.r2;
    controls.last_x = controls.x;
    controls.last_y = controls.y;
    controls.last_a = controls.a;
    controls.last_b = controls.b;
    controls.last_left = controls.left;
    controls.last_right = controls.right;
    controls.last_up = controls.up;
    controls.last_down = controls.down;

    controls.time_ms = config::Brain.timer(vex::timeUnits::msec);

    controls.axis1 = config::Controller.Axis1.position(vex::percentUnits::pct);
    controls.axis2 = config::Controller.Axis2.position(vex::percentUnits::pct);
    controls.axis3 = config::Controller.Axis3.position(vex::percentUnits::pct);
    controls.axis4 = config::Controller.Axis4.position(vex::percentUnits::pct);

    calculate_button_rating();

    controls.l1 = config::Controller.ButtonL1.pressing();
    controls.l2 = config::Controller.ButtonL2.pressing();
    controls.r1 = config::Controller.ButtonR1.pressing();
    controls.r2 = config::Controller.ButtonR2.pressing();
    controls.x = config::Controller.ButtonX.pressing();
    controls.y = config::Controller.ButtonY.pressing();
    controls.a = config::Controller.ButtonA.pressing();
    controls.b = config::Controller.ButtonB.pressing();
    controls.left = config::Controller.ButtonLeft.pressing();
    controls.right = config::Controller.ButtonRight.pressing();
    controls.up = config::Controller.ButtonUp.pressing();
    controls.down = config::Controller.ButtonDown.pressing();

    if (controls.x && !controls.last_x) controls.press_x = true;
    if (controls.a && !controls.last_a) controls.press_a = true;
    if (controls.b && !controls.last_b) controls.press_b = true;
    if (controls.y && !controls.last_y) controls.press_y = true;
    if (controls.up && !controls.last_up) controls.press_up = true;
    if (controls.down && !controls.last_down) controls.press_down = true;
    if (controls.right && !controls.last_right) controls.press_right = true;
    if (controls.left && !controls.last_left) controls.press_left = true;
    if (controls.l1 && !controls.last_l1) controls.press_l1 = true;
    if (controls.l2 && !controls.last_l2) controls.press_l2 = true;
    if (controls.r1 && !controls.last_r1) controls.press_r1 = true;
    if (controls.r2 && !controls.last_r2) controls.press_r2 = true;

    vex::this_thread::sleep_for(common::kRefreshTime);
  }
}

}  // namespace input
}  // namespace basic
