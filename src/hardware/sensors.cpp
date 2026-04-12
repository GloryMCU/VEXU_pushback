#include "hardware/sensors.h"

#include "hardware/robot_config.h"
#include "v5_apiuser.h"

namespace basic::hardware {

int Accelerate = 0;
char CurrentColorCode = 'N';

ColorName classify_hue(double hue) {
  if ((hue >= 330) || (hue < 30)) return ColorName::red;
  if (hue >= 30 && hue < 90) return ColorName::yellow;
  if (hue >= 90 && hue < 180) return ColorName::green;
  if (hue >= 180 && hue < 270) return ColorName::blue;
  return ColorName::unknown;
}

void initialize_sensors() {
  // colorSensor.setLight(ledState::on);
  // colorSensor.setLightPower(kLedPower);
}

double imu_heading() {
  double heading = Inertial.rotation(vex::rotationUnits::deg);
  heading = heading / kImuModCoefficient * 3600;

  while (heading < 0) heading += 360;
  while (heading >= 360) heading -= 360;
  return heading;
}

void reset_heading() { Inertial.resetRotation(); }

void show_calibrated() {
  Controller.Screen.setCursor(5, 1);
  Controller.Screen.print("      calibrated!");
}

void show_sensor_color(char color_code) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("%c", color_code);
  Brain.Screen.newLine();

  switch (color_code) {
    case 'R':
      Brain.Screen.print(">> RED <<");
      break;
    case 'G':
      Brain.Screen.print(">> GREEN <<");
      break;
    case 'B':
      Brain.Screen.print(">> BLUE <<");
      break;
    default:
      Brain.Screen.print("?? UNKNOWN ??");
      break;
  }
}

void run_sensor_thread() {
  static bool inited = false;
  if (!inited) {
    vexGenericSerialEnable(serial_sensor.index(), 9);
    vexGenericSerialBaudrate(serial_sensor.index(), 115200);
    initialize_sensors();
    inited = true;
  }

  char previous_color = 'N';

  while (true) {
    int bytes_avail = vexGenericSerialReceiveAvail(serial_sensor.index());
    while (bytes_avail-- > 0) {
      int character = vexGenericSerialReadChar(serial_sensor.index());
      if (character != -1) {
        CurrentColorCode = static_cast<char>(character);
      }
    }

    switch (CurrentColorCode) {
      case 'R':
        Accelerate = kIsBlue ? 1 : 0;
        break;
      case 'G':
        Accelerate = 0;
        break;
      case 'B':
        Accelerate = kIsBlue ? 0 : 1;
        break;
      default:
        Accelerate = 0;
        break;
    }

    if (kIsBlue) {
      if (previous_color == 'R' && CurrentColorCode != 'R') {
        Accelerate = -1;
        vex::this_thread::sleep_for(500);
      }
    } else if (previous_color == 'B' && CurrentColorCode != 'B') {
      Accelerate = -1;
      vex::this_thread::sleep_for(500);
    }

    show_sensor_color(CurrentColorCode);
    previous_color = CurrentColorCode;
    vex::this_thread::sleep_for(80);
  }
}

}  // namespace basic::hardware
