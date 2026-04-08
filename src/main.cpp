/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       86135                                                     */
/*    Created:      2025/9/10 17:33:24                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

#include "config/robot_config.h"
#include "control/driver_control.h"
#include "input/controller_input.h"
#include "subsystems/drive.h"
#include "sensors/sensors.h"

#ifdef COMPETITION
vex::competition Competition;
#endif

int main() {
  vex::timer time_begin;
  time_begin.system();

  basic::config::Inertial.calibrate();
  while (basic::config::Inertial.isCalibrating()) {
    vex::wait(5, vex::msec);
  }
  basic::config::Inertial.resetHeading();
  basic::config::Inertial.resetRotation();

  basic::config::Controller.Screen.setCursor(5, 1);
  basic::config::Controller.Screen.print("      calibrated!");

  vex::thread controller_task(basic::input::input_updating_thread);
  vex::thread chassis_task(basic::subsystems::chassis_updating_thread);
  vex::thread sensor_task(basic::sensors::runsensor);

#ifdef COMPETITION
  Competition.drivercontrol(basic::control::user_control);
#endif
  return 0;
}
