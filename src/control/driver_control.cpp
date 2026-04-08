#include "control/driver_control.h"

#include "common/parameters.h"
#include "subsystems/drive.h"
#include "subsystems/mechanisms.h"

namespace basic {
namespace control {

void user_control_thread() {
  while (true) {
    subsystems::Chassis::getInstance()->OmniChassisControl();
    subsystems::update_driver_mechanisms();

    vex::this_thread::sleep_for(common::kRefreshTime);
  }
}

void user_control() {
  static vex::thread user_control_task(user_control_thread);
}

}  // namespace control
}  // namespace basic
