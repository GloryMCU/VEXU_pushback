#include "hardware/robot_selector.h"

#include "control/chassis.h"
#include "control/mechanisms.h"
#include "hardware/robot_hardware.h"
#include "hardware/sensors.h"
#include "hardware/robots/robot_state.h"
#include "input/controller.h"

namespace basic::hardware {

namespace {

class BasicRobot;
BasicRobot& current_basic_robot();

class BasicRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    hardware_.calibrate_inertial_sensor();
    hardware_.show_calibrated();
  }

  void start_background_tasks() override {}

  void bind_competition(vex::competition& competition) override {
    competition.drivercontrol(start_driver_control_entry);
  }

 private:
  static void start_driver_control_entry() {
    static vex::thread user_control_thread(run_driver_control_thread_entry);
  }

  static void run_driver_control_thread_entry() {
    current_basic_robot().run_driver_control_loop();
  }

  void run_driver_control_loop() {
    while (true) {
      robots::controller_update(hardware_, state_);
      robots::sensor_update(hardware_, state_);
      robots::chassis_update(hardware_, state_);
      robots::mechanism_update(hardware_, state_);
      vex::this_thread::sleep_for(robots::kRefreshTime);
    }
  }

  robots::RobotHardware hardware_;
  robots::RobotState state_;

  friend BasicRobot& current_basic_robot();
};

BasicRobot& current_basic_robot() {
  static BasicRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_current_robot() {
  return current_basic_robot();
}

}  // namespace basic::hardware
