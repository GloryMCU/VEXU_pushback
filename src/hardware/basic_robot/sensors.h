#ifndef BASIC_SRC_HARDWARE_BASIC_ROBOT_SENSORS_H_
#define BASIC_SRC_HARDWARE_BASIC_ROBOT_SENSORS_H_

#include "hardware/basic_robot/robot_hardware.h"
#include "hardware/basic_robot/robot_state.h"

namespace basic::hardware::basic_robot {

void sensor_update(RobotHardware& hardware, RobotState& state, vex::color target);

}  // namespace basic::hardware::basic_robot

#endif
