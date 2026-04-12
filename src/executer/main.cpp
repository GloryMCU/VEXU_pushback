/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       86135                                                     */
/*    Created:      2025/9/10 17:33:24                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

#include "control/chassis.h"
#include "hardware/robot_config.h"
#include "hardware/sensors.h"
#include "input/controller.h"
#include "test/performance_tests.h"

#ifdef COMPETITION
vex::competition Competition;
#endif

int main() {
  vex::timer time_begin;
  time_begin.system();

  basic::hardware::Inertial.calibrate();
  while (basic::hardware::Inertial.isCalibrating()) {
    vex::wait(5, vex::msec);
  }
  basic::hardware::Inertial.resetHeading();
  basic::hardware::Inertial.resetRotation();

  basic::hardware::show_calibrated();

  if (basic::test::start_test_mode_if_enabled()) {
    while (true) {
      vex::this_thread::sleep_for(1000);
    }
  }

  vex::thread controller_task(basic::input::run_input_thread);
  vex::thread chassis_task(basic::control::run_chassis_thread);
  vex::thread sensor_task(basic::hardware::run_sensor_thread);

#ifdef COMPETITION
  Competition.drivercontrol(basic::control::start_driver_control);
#endif
  return 0;
}
