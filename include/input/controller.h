#ifndef BASIC_INPUT_CONTROLLER_H_
#define BASIC_INPUT_CONTROLLER_H_

namespace basic::input {

struct ControllerState {
  int time_ms{0};

  int axis1{0};
  int axis2{0};
  int axis3{0};
  int axis4{0};

  int last_axis1{0};
  int last_axis2{0};
  int last_axis3{0};
  int last_axis4{0};

  bool l1{false};
  bool l2{false};
  bool r1{false};
  bool r2{false};
  bool x{false};
  bool y{false};
  bool a{false};
  bool b{false};
  bool left{false};
  bool right{false};
  bool up{false};
  bool down{false};

  bool last_l1{false};
  bool last_l2{false};
  bool last_r1{false};
  bool last_r2{false};
  bool last_x{false};
  bool last_y{false};
  bool last_a{false};
  bool last_b{false};
  bool last_left{false};
  bool last_right{false};
  bool last_up{false};
  bool last_down{false};

  bool press_x{false};
  bool press_y{false};
  bool press_a{false};
  bool press_b{false};
  bool press_up{false};
  bool press_down{false};
  bool press_left{false};
  bool press_right{false};
  bool press_l1{false};
  bool press_l2{false};
  bool press_r1{false};
  bool press_r2{false};

  double rating[4]{0, 0, 0, 0};
};

ControllerState get_controls_snapshot();

void run_input_thread();

}  // namespace basic::input

#endif
