#include "hardware/robot_selector.h"

#include "hardware/robots/basic_robot.h"

namespace basic::hardware {

basic::app::Robot& get_current_robot() {
  static robots::BasicRobot robot;
  return robot;
}

}  // namespace basic::hardware
