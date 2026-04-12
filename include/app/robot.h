#ifndef BASIC_APP_ROBOT_H_
#define BASIC_APP_ROBOT_H_

#include "vex.h"

namespace basic::app {

class Robot {
 public:
  virtual ~Robot() = default;

  virtual void initialize() = 0;
  virtual bool enter_test_mode_if_enabled() = 0;
  virtual void start_background_tasks() = 0;
  virtual void bind_competition(vex::competition& competition) = 0;
};

}  // namespace basic::app

#endif
