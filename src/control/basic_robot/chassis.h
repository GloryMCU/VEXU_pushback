#ifndef BASIC_SRC_CONTROL_BASIC_ROBOT_CHASSIS_H_
#define BASIC_SRC_CONTROL_BASIC_ROBOT_CHASSIS_H_

#include "hardware/basic_robot/robot_hardware.h"
#include "hardware/basic_robot/robot_state.h"

namespace basic::control::basic_robot {

void chassis_update(
    basic::hardware::basic_robot::RobotHardware& hardware,
    basic::hardware::basic_robot::RobotState& state);

}  // namespace basic::control::basic_robot

#endif
