#ifndef BASIC_CONTROL_CHASSIS_H_
#define BASIC_CONTROL_CHASSIS_H_

#include "vex.h"

namespace basic {
namespace control {

class Chassis {
 private:
  Chassis() = default;

  vex::mutex state_mutex;

  double fl = 0;
  double fr = 0;
  double bl = 0;
  double br = 0;

  vex::brakeType stop_brake_type = vex::coast;

 public:
  static Chassis* get_instance() {
    static Chassis instance;
    return &instance;
  }

  static void delete_instance() {}

  void brake(vex::brakeType type);
  void set_motor_power();
  double dynamic_smooth(int now, int last, double rating);
  void run_driver_control();
};

void update_driver_mechanisms();
void run_driver_control_loop();
void start_driver_control();
void run_chassis_thread();

}  // namespace control
}  // namespace basic

#endif
