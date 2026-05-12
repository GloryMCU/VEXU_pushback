#ifndef BASIC_SRC_HARDWARE_SECOND_ROBOT_SENSORS_H_
#define BASIC_SRC_HARDWARE_SECOND_ROBOT_SENSORS_H_

#include "hardware/second_robot/robot_hardware.h"
#include "hardware/second_robot/robot_state.h"

namespace basic::hardware::second_robot {

void sensor_update(RobotHardware& hardware, RobotState& state);

}  // namespace basic::hardware::second_robot

#endif
