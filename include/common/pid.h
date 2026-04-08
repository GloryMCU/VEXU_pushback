#ifndef BASIC_COMMON_PID_H_
#define BASIC_COMMON_PID_H_

namespace basic {
namespace common {

class PID {
 private:
  double kp{0};
  double ki{0};
  double kd{0};
};

}  // namespace common
}  // namespace basic

#endif
