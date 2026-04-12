#include "input/controller.h"

#include "hardware/robot_config.h"

#include <cmath>

namespace basic::input {

namespace {

vex::mutex controls_mutex;
ControllerState controls_state;

void calculate_button_rating(ControllerState& state) {
  state.rating[0] = std::abs(state.axis1 - state.last_axis1) * 0.005;
  state.rating[1] = std::abs(state.axis2 - state.last_axis2) * 0.005;
  state.rating[2] = std::abs(state.axis3 - state.last_axis3) * 0.005;
  state.rating[3] = std::abs(state.axis4 - state.last_axis4) * 0.005;
}

void clear_press_events(ControllerState& state) {
  state.press_x = false;
  state.press_y = false;
  state.press_a = false;
  state.press_b = false;
  state.press_up = false;
  state.press_down = false;
  state.press_left = false;
  state.press_right = false;
  state.press_l1 = false;
  state.press_l2 = false;
  state.press_r1 = false;
  state.press_r2 = false;
}

void update_press_events(ControllerState& state) {
  state.press_x = state.x && !state.last_x;
  state.press_a = state.a && !state.last_a;
  state.press_b = state.b && !state.last_b;
  state.press_y = state.y && !state.last_y;
  state.press_up = state.up && !state.last_up;
  state.press_down = state.down && !state.last_down;
  state.press_right = state.right && !state.last_right;
  state.press_left = state.left && !state.last_left;
  state.press_l1 = state.l1 && !state.last_l1;
  state.press_l2 = state.l2 && !state.last_l2;
  state.press_r1 = state.r1 && !state.last_r1;
  state.press_r2 = state.r2 && !state.last_r2;
}

}  // namespace

ControllerState get_controls_snapshot() {
  controls_mutex.lock();
  const ControllerState snapshot = controls_state;
  controls_mutex.unlock();
  return snapshot;
}

void run_input_thread() {
  ControllerState next_state;
  while (true) {
    next_state.last_axis1 = next_state.axis1;
    next_state.last_axis2 = next_state.axis2;
    next_state.last_axis3 = next_state.axis3;
    next_state.last_axis4 = next_state.axis4;

    next_state.last_l1 = next_state.l1;
    next_state.last_l2 = next_state.l2;
    next_state.last_r1 = next_state.r1;
    next_state.last_r2 = next_state.r2;
    next_state.last_x = next_state.x;
    next_state.last_y = next_state.y;
    next_state.last_a = next_state.a;
    next_state.last_b = next_state.b;
    next_state.last_left = next_state.left;
    next_state.last_right = next_state.right;
    next_state.last_up = next_state.up;
    next_state.last_down = next_state.down;

    next_state.time_ms = hardware::Brain.timer(vex::timeUnits::msec);

    next_state.axis1 = hardware::Controller.Axis1.position(vex::percentUnits::pct);
    next_state.axis2 = hardware::Controller.Axis2.position(vex::percentUnits::pct);
    next_state.axis3 = hardware::Controller.Axis3.position(vex::percentUnits::pct);
    next_state.axis4 = hardware::Controller.Axis4.position(vex::percentUnits::pct);

    next_state.l1 = hardware::Controller.ButtonL1.pressing();
    next_state.l2 = hardware::Controller.ButtonL2.pressing();
    next_state.r1 = hardware::Controller.ButtonR1.pressing();
    next_state.r2 = hardware::Controller.ButtonR2.pressing();
    next_state.x = hardware::Controller.ButtonX.pressing();
    next_state.y = hardware::Controller.ButtonY.pressing();
    next_state.a = hardware::Controller.ButtonA.pressing();
    next_state.b = hardware::Controller.ButtonB.pressing();
    next_state.left = hardware::Controller.ButtonLeft.pressing();
    next_state.right = hardware::Controller.ButtonRight.pressing();
    next_state.up = hardware::Controller.ButtonUp.pressing();
    next_state.down = hardware::Controller.ButtonDown.pressing();

    clear_press_events(next_state);
    calculate_button_rating(next_state);
    update_press_events(next_state);

    controls_mutex.lock();
    controls_state = next_state;
    controls_mutex.unlock();

    vex::this_thread::sleep_for(hardware::kRefreshTime);
  }
}

}  // namespace basic::input
