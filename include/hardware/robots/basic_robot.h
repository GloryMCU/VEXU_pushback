#ifndef BASIC_HARDWARE_ROBOTS_BASIC_ROBOT_H_
#define BASIC_HARDWARE_ROBOTS_BASIC_ROBOT_H_

#include "app/robot.h"

namespace basic::hardware::robots {

class BasicRobot : public basic::app::Robot {
 public:
  void initialize() override;
  bool enter_test_mode_if_enabled() override;
  void start_background_tasks() override;
  void bind_competition(vex::competition& competition) override;
};

}  // namespace basic::hardware::robots

#endif
