#include "hardware/robot_selector.h"

#include "control/autonomous/routine.h"
#include "control/chassis.h"
#include "control/mechanisms.h"
#include "control/motor_control.h"
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
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    competition.drivercontrol(start_driver_control_entry);
  }

 private:
  static void start_driver_control_entry() {
    current_basic_robot().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_basic_robot().run_autonomous_routine();
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      robots::controller_update(hardware_, state_);
      robots::sensor_update(hardware_, state_);
      robots::chassis_update(hardware_, state_);
      robots::mechanism_update(hardware_, state_);
      vex::this_thread::sleep_for(robots::kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    robots::autonomous::run_routine(hardware_, state_, *competition_);
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    state_.controller = robots::ControllerInputState{};
    state_.chassis = robots::ChassisState{};
    state_.chassis.stop_brake_type = drive_brake_type;
    state_.mechanism = robots::MechanismState{};

    robots::stopcontrol(hardware_.motor_fl1, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_fl2, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_fr1, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_fr2, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_bl1, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_bl2, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_br1, state_.chassis.stop_brake_type);
    robots::stopcontrol(hardware_.motor_br2, state_.chassis.stop_brake_type);

    robots::stopcontrol(hardware_.trans_motor1);
    robots::stopcontrol(hardware_.trans_motor2);
    robots::stopcontrol(hardware_.trans_motor3);
    robots::stopcontrol(hardware_.under_motor1);
    robots::stopcontrol(hardware_.middle_motor1);
    robots::stopcontrol(hardware_.up_motor1);

    robots::stopcontrol(hardware_.under_overhang_motor, vex::hold);
    robots::stopcontrol(hardware_.middle_overhang_motor, vex::hold);
    robots::stopcontrol(hardware_.up_overhang_motor, vex::hold);
  }

  robots::RobotHardware hardware_;
  robots::RobotState state_;
  vex::competition* competition_{nullptr};

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
