#include "localization/odometry.h"

#include "common/parameters.h"
#include "config/robot_config.h"
#include "sensors/sensors.h"

#include <cmath>

namespace basic {
namespace localization {

namespace {

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double deg) { return deg * kPi / 180.0; }

}  // namespace

Timer::Timer() : start_time_ms(static_cast<int>(std::floor(1000 * config::Brain.Timer.value()))) {}

Timer::Timer(int time_ms) : start_time_ms(time_ms) {}

int Timer::getTimeMs() const { return static_cast<int>(std::floor(1000 * config::Brain.Timer.value())) - start_time_ms; }

void Timer::reset() { start_time_ms = static_cast<int>(std::floor(1000 * config::Brain.Timer.value())); }

void Position::updateInertialHeading() { current_imu_heading = deg_to_rad(sensors::IMUHeading()); }

void Position::updateLeftMileage() {
  last_left_mileage = current_left_mileage;
  current_left_mileage =
      -deg_to_rad(-config::motor_bl1.position(vex::degrees)) * common::kWheelTransitionCoefficient;
}

void Position::updateRightMileage() {
  last_right_mileage = current_right_mileage;
  current_right_mileage =
      -deg_to_rad(-config::motor_br1.position(vex::degrees)) * common::kWheelTransitionCoefficient;
}

void Position::updateLeftSpeed() {
  last_left_speed = current_left_speed;
  double value = (current_left_mileage - last_left_mileage) * 1000 / sample_time;
  if (std::abs(value) > 1000 || std::abs(value) < 0.001) {
    value = 0;
  }
  current_left_speed = value;
}

void Position::updateRightSpeed() {
  last_right_speed = current_right_speed;
  double value = (current_right_mileage - last_right_mileage) * 1000 / sample_time;
  if (std::abs(value) > 1000 || std::abs(value) < 0.001) {
    value = 0;
  }
  current_right_speed = value;
}

void Position::updateSelfSpeed() { self_speed = (current_left_speed + current_right_speed) / 2; }

void Position::updateGlobalYSpeed() {
  last_global_y_speed = global_y_speed;
  global_y_speed = self_speed * std::cos(current_imu_heading);
  if (std::abs(global_y_speed) < 0.01) {
    global_y_speed = 0;
  }
}

void Position::updateGlobalXSpeed() {
  last_global_x_speed = global_x_speed;
  global_x_speed = self_speed * std::sin(current_imu_heading);
  if (std::abs(global_x_speed) < 0.01) {
    global_x_speed = 0;
  }
}

void Position::updateGlobalY() {
  const double delta = (global_y_speed + last_global_y_speed) * sample_time / 1000 / 2;
  if (std::abs(delta) < 0.001) {
    return;
  }
  global_y = global_y + delta;
}

void Position::updateGlobalX() {
  const double delta = (global_x_speed + last_global_x_speed) * sample_time / 1000 / 2;
  if (std::abs(delta) < 0.001) {
    return;
  }
  global_x = global_x + delta;
}

void Position::update() {
  const double time_current = timer.getTimeMs();
  sample_time = time_current - last_time;
  last_time = time_current;

  if (sample_time < 0.001) {
    sample_time = common::kRefreshTime;
  }

  updateInertialHeading();
  updateLeftMileage();
  updateRightMileage();
  updateLeftSpeed();
  updateRightSpeed();
  updateSelfSpeed();
  updateGlobalYSpeed();
  updateGlobalXSpeed();
  updateGlobalY();
  updateGlobalX();
}

Point Position::getPosition() const { return Point(global_x, global_y); }

double Position::getXSpeed() const { return global_x_speed; }

double Position::getYSpeed() const { return global_y_speed; }

double Position::getLeftMileage() const { return current_left_mileage; }

double Position::getRightMileage() const { return current_right_mileage; }

void Position::resetXPosition() { global_x = 0; }

void Position::resetYPosition() { global_y = 0; }

void Position::setGlobalPosition(double x_value, double y_value) {
  global_x = x_value;
  global_y = y_value;
}

void update_position_thread() {
  while (true) {
    Position::getInstance()->update();
    vex::this_thread::sleep_for(common::kRefreshTime);
  }
}

void Position::reset() {
  global_x = 0;
  global_y = 0;
}

}  // namespace localization
}  // namespace basic
