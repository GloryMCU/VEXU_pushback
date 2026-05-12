#ifndef BASIC_SRC_INPUT_CONTROLLER_H_
#define BASIC_SRC_INPUT_CONTROLLER_H_

#include "hardware/shared/state_types.h"

namespace basic::input {

void controller_update(
    vex::brain& brain,
    vex::controller& controller,
    basic::hardware::shared::ControllerInputState& state);

}  // namespace basic::input

#endif
