#include "control/autonomous/routine.h"

#include "control/motor_control.h"

#include <array>
#include <cmath>

namespace basic::hardware::robots::autonomous {

namespace {

constexpr double kMillimetersPerWheelRevolution = 212.8;
constexpr int kAutonomousLoopDelayMs = 10;
constexpr int kAutonomousSettleDelayMs = 150;
constexpr double kDriveToleranceRevolutions = 0.03;
constexpr double kTurnToleranceDegrees = 1.5;
constexpr double kDriveProportionalGain = 30.0;
constexpr double kDriveMinSpeedPct = 12.0;
constexpr double kDriveMaxSpeedPct = 45.0;
constexpr double kTurnProportionalGain = 0.6;
constexpr double kTurnMinSpeedPct = 10.0;
constexpr double kTurnMaxSpeedPct = 30.0;

using DriveMotorArray = std::array<vex::motor*, 8>;
using DriveSample = std::array<double, 8>;
using SideMotorArray = std::array<vex::motor*, 4>;

DriveMotorArray drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.motor_fl1,
      &hardware.motor_fl2,
      &hardware.motor_fr1,
      &hardware.motor_fr2,
      &hardware.motor_bl1,
      &hardware.motor_bl2,
      &hardware.motor_br1,
      &hardware.motor_br2,
  }};
}

SideMotorArray left_drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.motor_fl1,
      &hardware.motor_fl2,
      &hardware.motor_bl1,
      &hardware.motor_bl2,
  }};
}

SideMotorArray right_drive_motors(RobotHardware& hardware) {
  return {{
      &hardware.motor_fr1,
      &hardware.motor_fr2,
      &hardware.motor_br1,
      &hardware.motor_br2,
  }};
}

bool should_run_autonomous(vex::competition& competition) {
  return competition.isEnabled() && competition.isAutonomous();
}

DriveSample sample_drive_revolutions(RobotHardware& hardware) {
  const DriveMotorArray motors = drive_motors(hardware);
  DriveSample sample{};
  for (std::size_t index = 0; index < motors.size(); ++index) {
    sample[index] = motors[index]->position(vex::rev);
  }
  return sample;
}

double average_drive_delta_revolutions(RobotHardware& hardware, const DriveSample& start_sample) {
  const DriveSample current_sample = sample_drive_revolutions(hardware);
  double total = 0.0;
  for (std::size_t index = 0; index < current_sample.size(); ++index) {
    total += std::fabs(current_sample[index] - start_sample[index]);
  }
  return total / static_cast<double>(current_sample.size());
}

void set_drive_power(RobotHardware& hardware, double left_pct, double right_pct) {
  for (vex::motor* motor : left_drive_motors(hardware)) {
    velocitycontrol(*motor, left_pct, vex::pct);
  }
  for (vex::motor* motor : right_drive_motors(hardware)) {
    velocitycontrol(*motor, right_pct, vex::pct);
  }
}

void stop_drive(RobotHardware& hardware, vex::brakeType brake_type) {
  for (vex::motor* motor : drive_motors(hardware)) {
    stopcontrol(*motor, brake_type);
  }
}

double clamp_speed(double speed_pct, double min_speed_pct, double max_speed_pct) {
  return std::min(std::max(speed_pct, min_speed_pct), max_speed_pct);
}

void settle_after_motion() {
  vex::this_thread::sleep_for(kAutonomousSettleDelayMs);
}

void drive_distance_mm(RobotHardware& hardware, vex::competition& competition, double distance_mm) {
  if (!should_run_autonomous(competition) || distance_mm == 0.0) {
    return;
  }

  const double direction = distance_mm > 0.0 ? 1.0 : -1.0;
  const double target_revolutions = std::fabs(distance_mm) / kMillimetersPerWheelRevolution;
  const DriveSample start_sample = sample_drive_revolutions(hardware);

  while (should_run_autonomous(competition)) {
    const double traveled_revolutions = average_drive_delta_revolutions(hardware, start_sample);
    const double remaining_revolutions = target_revolutions - traveled_revolutions;
    if (remaining_revolutions <= kDriveToleranceRevolutions) {
      break;
    }

    const double speed_pct = clamp_speed(
        remaining_revolutions * kDriveProportionalGain,
        kDriveMinSpeedPct,
        kDriveMaxSpeedPct);
    set_drive_power(hardware, direction * speed_pct, direction * speed_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  settle_after_motion();
}

void turn_left_deg(RobotHardware& hardware, vex::competition& competition, double target_degrees) {
  if (!should_run_autonomous(competition) || target_degrees <= 0.0) {
    return;
  }

  hardware.inertial.resetRotation();
  while (should_run_autonomous(competition)) {
    const double turned_degrees = std::fabs(hardware.inertial.rotation(vex::deg));
    const double remaining_degrees = target_degrees - turned_degrees;
    if (remaining_degrees <= kTurnToleranceDegrees) {
      break;
    }

    const double speed_pct = clamp_speed(
        remaining_degrees * kTurnProportionalGain,
        kTurnMinSpeedPct,
        kTurnMaxSpeedPct);
    set_drive_power(hardware, -speed_pct, speed_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  settle_after_motion();
}

}  // namespace

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {
  state.chassis.stop_brake_type = vex::hold;

  drive_distance_mm(hardware, competition, 600.0);
  turn_left_deg(hardware, competition, 90.0);
  drive_distance_mm(hardware, competition, 467.0);
  drive_distance_mm(hardware, competition, -1027.0);

  stop_drive(hardware, vex::hold);
}

}  // namespace basic::hardware::robots::autonomous
