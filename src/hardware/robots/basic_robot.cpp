#include "hardware/robots/basic_robot.h"

#include "control/chassis.h"
#include "hardware/robot_config.h"
#include "hardware/sensors.h"
#include "input/controller.h"
#include "test/performance_tests.h"

namespace basic::hardware::robots {

namespace {

void calibrate_inertial_sensor() {
  basic::hardware::Inertial.calibrate();
  while (basic::hardware::Inertial.isCalibrating()) {
    vex::wait(5, vex::msec);
  }

  basic::hardware::Inertial.resetHeading();
  basic::hardware::Inertial.resetRotation();
}

}  // namespace

void BasicRobot::initialize() {
  calibrate_inertial_sensor();
  basic::hardware::show_calibrated();
}

bool BasicRobot::enter_test_mode_if_enabled() {
  return basic::test::start_test_mode_if_enabled();
}

void BasicRobot::start_background_tasks() {
  static vex::thread controller_task(basic::input::run_input_thread);
  static vex::thread chassis_task(basic::control::run_chassis_thread);
  static vex::thread sensor_task(basic::hardware::run_sensor_thread);
}

void BasicRobot::bind_competition(vex::competition& competition) {
  competition.drivercontrol(basic::control::start_driver_control);
}

}  // namespace basic::hardware::robots
