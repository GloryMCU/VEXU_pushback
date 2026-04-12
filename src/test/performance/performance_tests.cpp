#include "test/performance_tests.h"

#include "hardware/robot_config.h"

#include <cstdint>
#include <cstdio>
#include <limits>

namespace basic {
namespace test {

namespace {

constexpr bool kEnableTestMode = false;
constexpr bool kEnableMotorTorqueRampTest = true;
constexpr bool kEnableSchedulerJitterTest = true;

struct MotorTorqueRampConfig {
  const char* name;
  vex::motor* motor;
  vex::directionType direction;
  double drive_velocity_pct;
  double min_torque_nm;
  double max_torque_nm;
  int settle_time_ms;
  int ramp_up_time_ms;
  int ramp_down_time_ms;
  int sample_period_ms;
  int cycle_count;
};

struct SchedulerJitterConfig {
  const char* name;
  int expected_period_ms;
  int sample_count;
};

MotorTorqueRampConfig default_motor_torque_ramp_config() {
  MotorTorqueRampConfig config;
  config.name = "motor_fl1";
  config.motor = &hardware::motor_fl1;
  config.direction = vex::directionType::fwd;
  config.drive_velocity_pct = 100.0;
  config.min_torque_nm = 0.0;
  config.max_torque_nm = 0.60;
  config.settle_time_ms = 200;
  config.ramp_up_time_ms = 3000;
  config.ramp_down_time_ms = 3000;
  config.sample_period_ms = 20;
  config.cycle_count = 1;
  return config;
}

SchedulerJitterConfig default_scheduler_jitter_config() {
  SchedulerJitterConfig config;
  config.name = "refresh_thread";
  config.expected_period_ms = hardware::kRefreshTime;
  config.sample_count = 500;
  return config;
}

double compute_torque_command(const MotorTorqueRampConfig& config, double elapsed_ms) {
  if (elapsed_ms <= 0.0) {
    return config.min_torque_nm;
  }

  if (elapsed_ms < config.ramp_up_time_ms) {
    const double ratio = elapsed_ms / config.ramp_up_time_ms;
    return config.min_torque_nm
         + (config.max_torque_nm - config.min_torque_nm) * ratio;
  }

  if (elapsed_ms < config.ramp_up_time_ms + config.ramp_down_time_ms) {
    const double ratio =
        (elapsed_ms - config.ramp_up_time_ms) / config.ramp_down_time_ms;
    return config.max_torque_nm
         - (config.max_torque_nm - config.min_torque_nm) * ratio;
  }

  return config.min_torque_nm;
}

void log_motor_torque_ramp_header(const MotorTorqueRampConfig& config) {
  std::printf(
      "[motor_test] mode=velocity_with_torque_limit motor=%s velocity_pct=%.1f "
      "min_torque_nm=%.3f max_torque_nm=%.3f settle_ms=%d ramp_up_ms=%d "
      "ramp_down_ms=%d sample_period_ms=%d cycles=%d\n",
      config.name,
      config.drive_velocity_pct,
      config.min_torque_nm,
      config.max_torque_nm,
      config.settle_time_ms,
      config.ramp_up_time_ms,
      config.ramp_down_time_ms,
      config.sample_period_ms,
      config.cycle_count);
  std::printf(
      "motor_test_csv,sample,cycle,elapsed_ms,command_torque_nm,"
      "measured_torque_nm,velocity_rpm,current_amp,position_deg,"
      "temperature_pct\n");
  std::fflush(stdout);
}

void run_motor_torque_ramp_test() {
  const MotorTorqueRampConfig config = default_motor_torque_ramp_config();
  vex::motor& motor = *config.motor;
  const int cycle_time_ms = config.ramp_up_time_ms + config.ramp_down_time_ms;

  log_motor_torque_ramp_header(config);

  motor.stop(vex::brakeType::coast);
  motor.resetPosition();
  motor.setStopping(vex::brakeType::coast);
  motor.setVelocity(config.drive_velocity_pct, vex::percentUnits::pct);
  motor.setMaxTorque(config.min_torque_nm, vex::torqueUnits::Nm);
  motor.spin(config.direction);

  if (config.settle_time_ms > 0) {
    vex::this_thread::sleep_for(config.settle_time_ms);
  }

  int sample = 0;
  for (int cycle = 0; cycle < config.cycle_count; ++cycle) {
    const std::uint64_t cycle_start_us = vex::timer::systemHighResolution();

    while (true) {
      const std::uint64_t now_us = vex::timer::systemHighResolution();
      const double elapsed_ms =
          static_cast<double>(now_us - cycle_start_us) * 0.001;

      if (elapsed_ms > cycle_time_ms) {
        break;
      }

      const double command_torque_nm = compute_torque_command(config, elapsed_ms);
      motor.setMaxTorque(command_torque_nm, vex::torqueUnits::Nm);

      std::printf(
          "motor_test_csv,%d,%d,%.3f,%.4f,%.4f,%.3f,%.4f,%.3f,%.3f\n",
          sample,
          cycle,
          elapsed_ms,
          command_torque_nm,
          motor.torque(vex::torqueUnits::Nm),
          motor.velocity(vex::velocityUnits::rpm),
          motor.current(vex::currentUnits::amp),
          motor.position(vex::rotationUnits::deg),
          motor.temperature(vex::percentUnits::pct));
      std::fflush(stdout);

      ++sample;
      vex::this_thread::sleep_for(config.sample_period_ms);
    }
  }

  motor.setMaxTorque(100.0, vex::percentUnits::pct);
  motor.stop(vex::brakeType::coast);

  std::printf("[motor_test] completed motor=%s samples=%d\n", config.name, sample);
  std::fflush(stdout);
}

void log_scheduler_jitter_header(const SchedulerJitterConfig& config) {
  std::printf(
      "[scheduler_test] name=%s expected_period_ms=%d samples=%d\n",
      config.name,
      config.expected_period_ms,
      config.sample_count);
  std::printf(
      "scheduler_test_csv,sample,elapsed_us,actual_period_us,error_us,"
      "min_error_us,max_error_us\n");
  std::fflush(stdout);
}

void run_scheduler_jitter_test() {
  const SchedulerJitterConfig config = default_scheduler_jitter_config();
  const std::uint64_t expected_period_us =
      static_cast<std::uint64_t>(config.expected_period_ms) * 1000ULL;
  std::uint64_t last_time_us = vex::timer::systemHighResolution();
  const std::uint64_t start_time_us = last_time_us;

  std::int64_t min_error_us = std::numeric_limits<std::int64_t>::max();
  std::int64_t max_error_us = std::numeric_limits<std::int64_t>::min();

  log_scheduler_jitter_header(config);

  for (int sample = 0; sample < config.sample_count; ++sample) {
    vex::this_thread::sleep_for(config.expected_period_ms);

    const std::uint64_t now_us = vex::timer::systemHighResolution();
    const std::uint64_t actual_period_us = now_us - last_time_us;
    const std::uint64_t elapsed_us = now_us - start_time_us;
    const std::int64_t error_us = static_cast<std::int64_t>(actual_period_us)
                                - static_cast<std::int64_t>(expected_period_us);

    if (error_us < min_error_us) {
      min_error_us = error_us;
    }
    if (error_us > max_error_us) {
      max_error_us = error_us;
    }

    std::printf(
        "scheduler_test_csv,%d,%llu,%llu,%lld,%lld,%lld\n",
        sample,
        static_cast<unsigned long long>(elapsed_us),
        static_cast<unsigned long long>(actual_period_us),
        static_cast<long long>(error_us),
        static_cast<long long>(min_error_us),
        static_cast<long long>(max_error_us));
    std::fflush(stdout);

    last_time_us = now_us;
  }

  std::printf(
      "[scheduler_test] completed name=%s min_error_us=%lld max_error_us=%lld\n",
      config.name,
      static_cast<long long>(min_error_us),
      static_cast<long long>(max_error_us));
  std::fflush(stdout);
}

}  // namespace

void start_enabled_tests() {
  static bool started = false;
  if (started) {
    return;
  }
  started = true;

  if (kEnableSchedulerJitterTest) {
    static vex::thread scheduler_jitter_thread(run_scheduler_jitter_test);
    scheduler_jitter_thread.setPriority(vex::thread::threadPriorityNormal);
  }

  if (kEnableMotorTorqueRampTest) {
    static vex::thread motor_torque_ramp_thread(run_motor_torque_ramp_test);
    motor_torque_ramp_thread.setPriority(vex::thread::threadPriorityNormal);
  }
}

bool start_test_mode_if_enabled() {
  if (!kEnableTestMode) {
    return false;
  }

  hardware::Brain.Screen.clearScreen();
  hardware::Brain.Screen.setCursor(1, 1);
  hardware::Brain.Screen.print("TEST MODE");

  std::printf("[test] starting hardware performance tests\n");
  std::fflush(stdout);

  start_enabled_tests();
  return true;
}

}  // namespace test
}  // namespace basic
