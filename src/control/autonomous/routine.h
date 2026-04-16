#ifndef BASIC_SRC_CONTROL_AUTONOMOUS_ROUTINE_H_
#define BASIC_SRC_CONTROL_AUTONOMOUS_ROUTINE_H_

#include "hardware/robot_hardware.h"
#include "hardware/robots/robot_state.h"

namespace basic::hardware::robots::autonomous {

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition);

}  // namespace basic::hardware::robots::autonomous

#endif
