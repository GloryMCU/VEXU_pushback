#include "control/autonomous/routine.h"

#include "control/motor_control.h"

#include <array>
#include <cmath>

namespace basic::hardware::robots::autonomous {

namespace {

constexpr double kMillimetersPerWheelRevolution = 212.8;
constexpr double kPi = 3.14159265358979323846;
constexpr int kAutonomousLoopDelayMs = 10;
constexpr int kAutonomousSettleDelayMs = 150;
constexpr double kDriveToleranceRevolutions = 0.03;
constexpr double kTurnToleranceDegrees = 1.5;
constexpr double kDriveProportionalGain = 30.0;
constexpr double kDriveMinSpeedPct = 12.0;
constexpr double kDriveMaxSpeedPct = 22.5;
constexpr double kDriveHeadingProportionalGain = 1.0;
constexpr double kDriveHeadingCorrectionMaxPct = 6.0;
constexpr double kDriveHeadingDeadbandDegrees = 1.0;
constexpr double kTurnProportionalGain = 0.6;
constexpr double kTurnMinSpeedPct = 10.0;
constexpr double kTurnApproachMinSpeedPct = 4.0;
constexpr double kTurnMaxSpeedPct = 30.0;
constexpr double kTurnApproachWindowDegrees = 12.0;
constexpr int kTurnBaseTimeoutMs = 1000;
constexpr int kTurnTimeoutPerDegreeMs = 50;

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

double clamp_correction(double correction_pct, double max_abs_correction_pct) {
  return std::min(std::max(correction_pct, -max_abs_correction_pct), max_abs_correction_pct);
}

double drive_heading_correction_pct(double heading_error_degrees) {
  if (std::fabs(heading_error_degrees) <= kDriveHeadingDeadbandDegrees) {
    return 0.0;
  }

  return clamp_correction(
      heading_error_degrees * kDriveHeadingProportionalGain,
      kDriveHeadingCorrectionMaxPct);
}

double deg_to_rad(double angle_deg) {
  return angle_deg * kPi / 180.0;
}

double normalize_angle_deg(double angle_deg) {
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg <= -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double current_heading_deg(RobotHardware& hardware) {
  return normalize_angle_deg(hardware.inertial.rotation(vex::deg));
}

void reset_autonomous_frame(RobotHardware& hardware, RobotState& state) {
  hardware.inertial.resetRotation();
  state.autonomous = AutonomousState{};
  state.autonomous.initialized = true;
}

void ensure_autonomous_frame(RobotHardware& hardware, RobotState& state) {
  if (!state.autonomous.initialized) {
    reset_autonomous_frame(hardware, state);
  }
}

void update_autonomous_pose(RobotState& state, double delta_mm, double heading_deg) {
  state.autonomous.estimated_heading_deg = heading_deg;
  state.autonomous.estimated_x_mm += delta_mm * std::cos(deg_to_rad(heading_deg));
  state.autonomous.estimated_y_mm += delta_mm * std::sin(deg_to_rad(heading_deg));
}

double turn_timeout_ms(double target_degrees) {
  return kTurnBaseTimeoutMs +
         std::ceil(std::fabs(target_degrees) * static_cast<double>(kTurnTimeoutPerDegreeMs));
}

double turn_speed_pct(double error_degrees) {
  const double abs_error_degrees = std::fabs(error_degrees);
  const double min_speed_pct =
      abs_error_degrees > kTurnApproachWindowDegrees ? kTurnMinSpeedPct : kTurnApproachMinSpeedPct;
  return clamp_speed(abs_error_degrees * kTurnProportionalGain, min_speed_pct, kTurnMaxSpeedPct);
}

void settle_after_motion() {
  vex::this_thread::sleep_for(kAutonomousSettleDelayMs);
}

void drive_distance_mm(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double distance_mm) {
  if (!should_run_autonomous(competition) || distance_mm == 0.0) {
    return;
  }

  ensure_autonomous_frame(hardware, state);
  const double direction = distance_mm > 0.0 ? 1.0 : -1.0;
  const double target_revolutions = std::fabs(distance_mm) / kMillimetersPerWheelRevolution;
  const DriveSample start_sample = sample_drive_revolutions(hardware);
  double previous_traveled_revolutions = 0.0;

  while (should_run_autonomous(competition)) {
    const double traveled_revolutions = average_drive_delta_revolutions(hardware, start_sample);
    const double current_heading_degrees = current_heading_deg(hardware);
    const double traveled_delta_revolutions = traveled_revolutions - previous_traveled_revolutions;
    if (traveled_delta_revolutions > 0.0) {
      update_autonomous_pose(
          state,
          direction * traveled_delta_revolutions * kMillimetersPerWheelRevolution,
          current_heading_degrees);
      previous_traveled_revolutions = traveled_revolutions;
    }

    const double remaining_revolutions = target_revolutions - traveled_revolutions;
    if (remaining_revolutions <= kDriveToleranceRevolutions) {
      break;
    }

    const double speed_pct = clamp_speed(
        remaining_revolutions * kDriveProportionalGain,
        kDriveMinSpeedPct,
        kDriveMaxSpeedPct);
    const double heading_error_degrees = normalize_angle_deg(
        state.autonomous.target_heading_deg - current_heading_degrees);
    const double heading_correction_pct = drive_heading_correction_pct(heading_error_degrees);
    const double drive_speed_pct = direction * speed_pct;
    set_drive_power(
        hardware,
        drive_speed_pct + heading_correction_pct,
        drive_speed_pct - heading_correction_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  settle_after_motion();
}

void turn_deg(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_degrees) {
  if (!should_run_autonomous(competition) || target_degrees == 0.0) {
    return;
  }

  ensure_autonomous_frame(hardware, state);
  state.autonomous.target_heading_deg =
      normalize_angle_deg(state.autonomous.target_heading_deg + target_degrees);
  const int turn_start_ms = hardware.brain.timer(vex::msec);
  const double timeout_ms = turn_timeout_ms(target_degrees);
  while (should_run_autonomous(competition)) {
    const int elapsed_ms = hardware.brain.timer(vex::msec) - turn_start_ms;
    if (elapsed_ms >= timeout_ms) {
      break;
    }

    const double current_heading_degrees = current_heading_deg(hardware);
    state.autonomous.estimated_heading_deg = current_heading_degrees;
    const double error_degrees = normalize_angle_deg(
        state.autonomous.target_heading_deg - current_heading_degrees);
    if (std::fabs(error_degrees) <= kTurnToleranceDegrees) {
      break;
    }

    const double speed_pct = turn_speed_pct(error_degrees);
    const double direction = error_degrees > 0.0 ? 1.0 : -1.0;
    set_drive_power(hardware, direction * speed_pct, -direction * speed_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  settle_after_motion();
}

}  // namespace

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {
  state.chassis.stop_brake_type = vex::hold;
  reset_autonomous_frame(hardware, state);

  drive_distance_mm(hardware, state, competition, 770.0);
  turn_deg(hardware, state, competition, -90.0);
  drive_distance_mm(hardware, state, competition, 397.0);
  drive_distance_mm(hardware, state, competition, -320.0);
  turn_deg(hardware, state, competition, 180.0);
  drive_distance_mm(hardware, state, competition, -342.0);

  stop_drive(hardware, vex::hold);
}

}  // namespace basic::hardware::robots::autonomous
