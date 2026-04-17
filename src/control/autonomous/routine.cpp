#include "control/autonomous/routine.h"

#include "control/motor_control.h"

#include <algorithm>
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
constexpr double kDriveMinSpeedPct = 12.0;
constexpr double kDriveMaxSpeedPct = 22.5;
constexpr double kDriveAccelerationWindowMm = 180.0;
constexpr double kDriveDecelerationWindowMm = 260.0;
constexpr double kDriveHeadingProportionalGain = 0.6;
constexpr double kDriveHeadingCorrectionMaxPct = 4.0;
constexpr double kDriveHeadingCorrectionSpeedRatio = 0.2;
constexpr double kDriveHeadingDeadbandDegrees = 1.0;
constexpr double kLaserDistanceToleranceMm = 10.0;
constexpr double kLaserDistanceMinSpeedPct = 6.0;
constexpr double kLaserDistanceMaxSpeedPct = 18.0;
constexpr double kLaserDistanceAccelerationWindowMm = 120.0;
constexpr double kLaserDistanceDecelerationWindowMm = 180.0;
constexpr int kLaserDistanceBaseTimeoutMs = 1200;
constexpr int kLaserDistanceTimeoutPerMm = 4;
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

double clamp_unit_interval(double value) {
  return std::min(std::max(value, 0.0), 1.0);
}

double clamp_correction(double correction_pct, double max_abs_correction_pct) {
  return std::min(std::max(correction_pct, -max_abs_correction_pct), max_abs_correction_pct);
}

double smoothstep01(double value) {
  const double clamped = clamp_unit_interval(value);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

double planned_linear_speed_pct(
    double traveled_mm,
    double remaining_mm,
    double min_speed_pct,
    double max_speed_pct,
    double acceleration_window_mm,
    double deceleration_window_mm) {
  const double accel_ratio = acceleration_window_mm > 0.0
                                 ? smoothstep01(traveled_mm / acceleration_window_mm)
                                 : 1.0;
  const double decel_ratio = deceleration_window_mm > 0.0
                                 ? smoothstep01(remaining_mm / deceleration_window_mm)
                                 : 1.0;
  const double accel_cap_pct = min_speed_pct + (max_speed_pct - min_speed_pct) * accel_ratio;
  const double decel_cap_pct = min_speed_pct + (max_speed_pct - min_speed_pct) * decel_ratio;
  return std::min(accel_cap_pct, decel_cap_pct);
}

double drive_heading_correction_pct(double heading_error_degrees, double drive_speed_pct) {
  if (std::fabs(heading_error_degrees) <= kDriveHeadingDeadbandDegrees) {
    return 0.0;
  }

  const double scaled_max_correction_pct = std::min(
      kDriveHeadingCorrectionMaxPct,
      std::fabs(drive_speed_pct) * kDriveHeadingCorrectionSpeedRatio);
  return clamp_correction(
      heading_error_degrees * kDriveHeadingProportionalGain,
      scaled_max_correction_pct);
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

double laser_distance_timeout_ms(double distance_error_mm) {
  return kLaserDistanceBaseTimeoutMs +
         std::ceil(std::fabs(distance_error_mm) * static_cast<double>(kLaserDistanceTimeoutPerMm));
}

bool try_read_laser_distance_mm(RobotHardware& hardware, double& measured_distance_mm) {
  if (!hardware.laser_rangefinder.installed() || !hardware.laser_rangefinder.isObjectDetected()) {
    return false;
  }

  measured_distance_mm = hardware.laser_rangefinder.objectDistance(vex::distanceUnits::mm);
  return std::isfinite(measured_distance_mm) && measured_distance_mm > 0.0;
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

    const double traveled_mm = traveled_revolutions * kMillimetersPerWheelRevolution;
    const double remaining_mm = remaining_revolutions * kMillimetersPerWheelRevolution;
    const double speed_pct = planned_linear_speed_pct(
        traveled_mm,
        remaining_mm,
        kDriveMinSpeedPct,
        kDriveMaxSpeedPct,
        kDriveAccelerationWindowMm,
        kDriveDecelerationWindowMm);
    const double drive_speed_pct = direction * speed_pct;
    const double heading_error_degrees = normalize_angle_deg(
        state.autonomous.target_heading_deg - current_heading_degrees);
    const double heading_correction_pct =
        drive_heading_correction_pct(heading_error_degrees, drive_speed_pct);
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

void drive_to_laser_distance_mm(
    RobotHardware& hardware,
    RobotState& state,
    vex::competition& competition,
    double target_distance_mm) {
  if (!should_run_autonomous(competition) || target_distance_mm <= 0.0) {
    return;
  }

  ensure_autonomous_frame(hardware, state);

  double measured_distance_mm = 0.0;
  if (!try_read_laser_distance_mm(hardware, measured_distance_mm)) {
    stop_drive(hardware, vex::hold);
    return;
  }

  const DriveSample start_sample = sample_drive_revolutions(hardware);
  double previous_traveled_revolutions = 0.0;
  const int motion_start_ms = hardware.brain.timer(vex::msec);
  const double initial_distance_error_mm = std::fabs(measured_distance_mm - target_distance_mm);
  const double timeout_ms = laser_distance_timeout_ms(measured_distance_mm - target_distance_mm);

  while (should_run_autonomous(competition)) {
    const int elapsed_ms = hardware.brain.timer(vex::msec) - motion_start_ms;
    if (elapsed_ms >= timeout_ms) {
      break;
    }

    if (!try_read_laser_distance_mm(hardware, measured_distance_mm)) {
      break;
    }

    const double current_heading_degrees = current_heading_deg(hardware);
    const double traveled_revolutions = average_drive_delta_revolutions(hardware, start_sample);
    const double traveled_delta_revolutions = traveled_revolutions - previous_traveled_revolutions;
    if (traveled_delta_revolutions > 0.0) {
      const double drive_direction = measured_distance_mm > target_distance_mm ? 1.0 : -1.0;
      update_autonomous_pose(
          state,
          drive_direction * traveled_delta_revolutions * kMillimetersPerWheelRevolution,
          current_heading_degrees);
      previous_traveled_revolutions = traveled_revolutions;
    }

    const double distance_error_mm = measured_distance_mm - target_distance_mm;
    if (std::fabs(distance_error_mm) <= kLaserDistanceToleranceMm) {
      break;
    }

    const double traveled_toward_target_mm =
        std::max(0.0, initial_distance_error_mm - std::fabs(distance_error_mm));
    const double drive_speed_pct = distance_error_mm > 0.0 ? 1.0 : -1.0;
    const double commanded_speed_pct = drive_speed_pct * planned_linear_speed_pct(
        traveled_toward_target_mm,
        std::fabs(distance_error_mm),
        kLaserDistanceMinSpeedPct,
        kLaserDistanceMaxSpeedPct,
        kLaserDistanceAccelerationWindowMm,
        kLaserDistanceDecelerationWindowMm);
    const double heading_error_degrees = normalize_angle_deg(
        state.autonomous.target_heading_deg - current_heading_degrees);
    const double heading_correction_pct =
        drive_heading_correction_pct(heading_error_degrees, commanded_speed_pct);
    set_drive_power(
        hardware,
        commanded_speed_pct + heading_correction_pct,
        commanded_speed_pct - heading_correction_pct);
    vex::this_thread::sleep_for(kAutonomousLoopDelayMs);
  }

  stop_drive(hardware, vex::hold);
  settle_after_motion();
}

void run_routine(RobotHardware& hardware, RobotState& state, vex::competition& competition) {
  state.chassis.stop_brake_type = vex::hold;
  reset_autonomous_frame(hardware, state);

  drive_distance_mm(hardware, state, competition, 730.0);
  turn_deg(hardware, state, competition, -90.0);
  drive_distance_mm(hardware, state, competition, 397.0);
  drive_distance_mm(hardware, state, competition, -320.0);
  turn_deg(hardware, state, competition, 180.0);
  drive_distance_mm(hardware, state, competition, -342.0);

  stop_drive(hardware, vex::hold);
}

}  // namespace basic::hardware::robots::autonomous
