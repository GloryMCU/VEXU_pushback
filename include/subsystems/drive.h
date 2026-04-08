#ifndef BASIC_SUBSYSTEMS_DRIVE_H_
#define BASIC_SUBSYSTEMS_DRIVE_H_

#include "vex.h"

namespace basic {
namespace subsystems {

class Chassis {
 private:
  Chassis() = default;

  double fl = 0;
  double fr = 0;
  double bl = 0;
  double br = 0;

  double input_cache[5] = {0, 0, 0, 0, 0};

  double front_left = 0;
  double front_right = 0;
  double back_left = 0;
  double back_right = 0;

  vex::brakeType stop_brake_type = vex::coast;

 public:
  static Chassis* getInstance() {
    static Chassis instance;
    return &instance;
  }

  static void deleteInstance() {}

  void Brake(vex::brakeType type);
  void SetMotorPower();
  double dynamicSmooth(int now, int last, double rating);
  void OmniChassisControl();
};

void chassis_updating_thread();

}  // namespace subsystems
}  // namespace basic

#endif
