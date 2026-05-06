#include "hardware/sensors.h"
#include "hardware/robots/robot_state.h"

#include "v5_apiuser.h"

namespace basic::hardware::robots {

namespace {
  
}

void sensor_update(RobotHardware& hardware, RobotState& state,vex::color target){
  if(state.mechanism.indexed_mode==IndexedMechanismMode::kLegacyIntake){
  }
}
}  // namespace basic::hardware::robots
