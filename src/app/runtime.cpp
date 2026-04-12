#include "app/runtime.h"

namespace basic::app {

namespace {

void wait_forever() {
  while (true) {
    vex::this_thread::sleep_for(1000);
  }
}

}  // namespace

void run(Robot& robot) {
  robot.initialize();

  if (robot.enter_test_mode_if_enabled()) {
    wait_forever();
    return;
  }

  robot.start_background_tasks();

#ifdef COMPETITION
  static vex::competition competition;
  robot.bind_competition(competition);
#endif
}

}  // namespace basic::app
