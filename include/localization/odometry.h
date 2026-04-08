#ifndef BASIC_LOCALIZATION_ODOMETRY_H_
#define BASIC_LOCALIZATION_ODOMETRY_H_

#include <cmath>

namespace basic {
namespace localization {

class Vector;

class Point {
 public:
  double x;
  double y;

  Point() : x(0), y(0) {}
  Point(double x_value, double y_value) : x(x_value), y(y_value) {}
  Point(const Point& other) : x(other.x), y(other.y) {}

  Point& operator=(const Point& other) {
    if (this == &other) {
      return *this;
    }
    x = other.x;
    y = other.y;
    return *this;
  }

  Point operator+(const Vector& vector) const;
  Point operator-(const Vector& vector) const;
  Vector operator-(const Point& point) const;

  void set(double x_value, double y_value) {
    x = x_value;
    y = y_value;
  }

  void reset() {
    x = 0;
    y = 0;
  }
};

class Vector {
 public:
  double x;
  double y;

  Vector() : x(0), y(0) {}
  Vector(double x_value, double y_value) : x(x_value), y(y_value) {}
  Vector(const Vector& other) : x(other.x), y(other.y) {}
  Vector(const Point& start, const Point& end) : x(end.x - start.x), y(end.y - start.y) {}

  Vector& operator=(const Vector& other) {
    if (this == &other) {
      return *this;
    }
    x = other.x;
    y = other.y;
    return *this;
  }

  Vector operator+(const Vector& other) const { return Vector(x + other.x, y + other.y); }
  Vector operator-(const Vector& other) const { return Vector(x - other.x, y - other.y); }
  double operator*(const Vector& other) const { return x * other.x + y * other.y; }
  Vector operator*(double scalar) const { return Vector(x * scalar, y * scalar); }
  Vector operator/(double scalar) const { return Vector(x / scalar, y / scalar); }
  double magnitude() const { return std::sqrt(x * x + y * y); }

  void set(double x_value, double y_value) {
    x = x_value;
    y = y_value;
  }

  void reset() {
    x = 0;
    y = 0;
  }
};

inline Point Point::operator+(const Vector& vector) const { return Point(x + vector.x, y + vector.y); }
inline Point Point::operator-(const Vector& vector) const { return Point(x - vector.x, y - vector.y); }
inline Vector Point::operator-(const Point& point) const { return Vector(x - point.x, y - point.y); }
inline Point operator+(const Vector& vector, const Point& point) { return Point(point.x + vector.x, point.y + vector.y); }
inline Vector operator*(double scalar, const Vector& vector) { return Vector(vector.x * scalar, vector.y * scalar); }

class Timer {
 private:
  int start_time_ms;

 public:
  Timer();
  explicit Timer(int time_ms);

  void reset();
  int getTimeMs() const;
};

class Position {
 private:
  Position()
      : current_imu_heading(0),
        current_left_mileage(0),
        current_right_mileage(0),
        last_left_mileage(0),
        last_right_mileage(0),
        current_left_speed(0),
        current_right_speed(0),
        last_left_speed(0),
        last_right_speed(0),
        self_speed(0),
        global_x_speed(0),
        global_y_speed(0),
        last_global_x_speed(0),
        last_global_y_speed(0),
        global_x(0),
        global_y(0),
        last_time(0),
        sample_time(0) {}

  double current_imu_heading;
  double current_left_mileage;
  double current_right_mileage;
  double last_left_mileage;
  double last_right_mileage;
  double current_left_speed;
  double current_right_speed;
  double last_left_speed;
  double last_right_speed;
  double self_speed;
  double global_x_speed;
  double global_y_speed;
  double last_global_x_speed;
  double last_global_y_speed;
  double global_x;
  double global_y;
  double last_time;
  double sample_time;
  Timer timer;

  void updateInertialHeading();
  void updateLeftMileage();
  void updateRightMileage();
  void updateLeftSpeed();
  void updateRightSpeed();
  void updateSelfSpeed();
  void updateGlobalYSpeed();
  void updateGlobalXSpeed();
  void updateGlobalY();
  void updateGlobalX();

 public:
  static Position* getInstance() {
    static Position instance;
    return &instance;
  }

  static void deleteInstance() {}

  void update();
  Point getPosition() const;
  double getXSpeed() const;
  double getYSpeed() const;
  double getLeftMileage() const;
  double getRightMileage() const;
  void resetYPosition();
  void resetXPosition();
  void setGlobalPosition(double x_value, double y_value);
  void reset();
};

void update_position_thread();

}  // namespace localization
}  // namespace basic

#endif
