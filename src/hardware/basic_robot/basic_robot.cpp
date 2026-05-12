#include "hardware/robot_selector.h"

#include "control/basic_robot/autonomous/routine.h"
#include "control/basic_robot/chassis.h"
#include "control/basic_robot/mechanisms.h"
#include "control/motor_control.h"
#include "hardware/basic_robot/robot_hardware.h"
#include "hardware/basic_robot/robot_state.h"
#include "hardware/basic_robot/sensors.h"
#include "input/controller.h"

namespace basic::hardware::basic_robot {

namespace {

class BasicRobot;
BasicRobot& current_basic_robot();

class BasicRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    hardware_.calibrate_inertial_sensor();
    hardware_.show_calibrated();
  }

  void bind_background_tasks() override {
    vex::thread background(start_background_tasks);
  }

  void bind_competition(vex::competition& competition) override {
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    competition.drivercontrol(start_driver_control_entry);
  }

 private:
  static void start_background_tasks() {
    current_basic_robot().run_background_tasks();
  }

  static void start_driver_control_entry() {
    current_basic_robot().run_driver_control_loop();
  }

  static void start_autonomous_entry() {
    current_basic_robot().run_autonomous_routine();
  }

  void run_background_tasks() {
    while (true) {
      sensor_update(hardware_, state_, vex::color::red);
      vex::this_thread::sleep_for(kSensorLoopDelay);
    }
  }

  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      basic::input::controller_update(hardware_.brain, hardware_.controller, state_.controller);
      basic::control::basic_robot::chassis_update(hardware_, state_);
      basic::control::basic_robot::mechanism_update(hardware_, state_);
      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    basic::control::basic_robot::autonomous::run_routine(hardware_, state_, *competition_);
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType drive_brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    state_.chassis = ChassisState{};
    state_.chassis.stop_brake_type = drive_brake_type;
    state_.mechanism = MechanismState{};

    basic::control::stopcontrol(hardware_.motor_fl1, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_fl2, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_fr1, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_fr2, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_bl1, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_bl2, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_br1, state_.chassis.stop_brake_type);
    basic::control::stopcontrol(hardware_.motor_br2, state_.chassis.stop_brake_type);

    basic::control::stopcontrol(hardware_.trans_motor1);
    basic::control::stopcontrol(hardware_.trans_motor2);
    basic::control::stopcontrol(hardware_.trans_motor3);
    basic::control::stopcontrol(hardware_.under_motor1);
    basic::control::stopcontrol(hardware_.middle_motor1);
    basic::control::stopcontrol(hardware_.upper_motor1);

    basic::control::stopcontrol(hardware_.under_overhang_motor, vex::hold);
    basic::control::stopcontrol(hardware_.middle_overhang_motor, vex::hold);
    basic::control::stopcontrol(hardware_.upper_overhang_motor, vex::hold);
  }

  RobotHardware hardware_;
  RobotState state_;
  vex::competition* competition_{nullptr};

  friend BasicRobot& current_basic_robot();
};

BasicRobot& current_basic_robot() {
  static BasicRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_basic_robot();
}

}  // namespace basic::hardware::basic_robot
