#include "hardware/basic_robot/sensors.h"

namespace basic::hardware::basic_robot {

void sensor_update(RobotHardware& hardware, RobotState& state, vex::color target) {
  (void)hardware;
  (void)target;
  if (state.mechanism.indexed_mode == IndexedMechanismMode::kLegacyIntake) {
  }
}

}  // namespace basic::hardware::basic_robot
