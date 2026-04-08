#include "sensors/sensors.h"

#include "common/parameters.h"
#include "config/robot_config.h"
#include "v5_apiuser.h"

namespace basic {
namespace sensors {

int Accelerate = 0;

ColorName classifyHue(double hue) {
  if ((hue >= 330) || (hue < 30)) return ColorName::red;
  if (hue >= 30 && hue < 90) return ColorName::yellow;
  if (hue >= 90 && hue < 180) return ColorName::green;
  if (hue >= 180 && hue < 270) return ColorName::blue;
  return ColorName::unknown;
}

void sensorInit() {
  // colorSensor.setLight(ledState::on);
  // colorSensor.setLightPower(kLedPower);
}

double IMUHeading() {
  double heading = config::Inertial.rotation(vex::rotationUnits::deg);
  heading = heading / common::kImuModCoefficient * 3600;

  while (heading < 0) heading += 360;
  while (heading >= 360) heading -= 360;
  return heading;
}

void resetHeading() { config::Inertial.resetRotation(); }

void runsensor() {
  static bool inited = false;
  if (!inited) {
    vexGenericSerialEnable(config::serial_sensor.index(), 9);
    vexGenericSerialBaudrate(config::serial_sensor.index(), 115200);
    inited = true;
  }

  char now_color = 'N';
  char pre_color = 'N';

  while (true) {
    int bytes_avail = vexGenericSerialReceiveAvail(config::serial_sensor.index());
    while (bytes_avail-- > 0) {
      int character = vexGenericSerialReadChar(config::serial_sensor.index());
      if (character != -1) {
        now_color = static_cast<char>(character);
      }
    }

    config::Brain.Screen.clearScreen();
    config::Brain.Screen.setCursor(1, 1);
    config::Brain.Screen.print("%c", now_color);
    config::Brain.Screen.newLine();

    switch (now_color) {
      case 'R':
        config::Brain.Screen.print(">> RED <<");
        Accelerate = config::kIsBlue ? 1 : 0;
        break;
      case 'G':
        config::Brain.Screen.print(">> GREEN <<");
        Accelerate = 0;
        break;
      case 'B':
        config::Brain.Screen.print(">> BLUE <<");
        Accelerate = config::kIsBlue ? 0 : 1;
        break;
      default:
        config::Brain.Screen.print("?? UNKNOWN ??");
        Accelerate = 0;
        break;
    }

    if (config::kIsBlue) {
      if (pre_color == 'R' && now_color != 'R') {
        Accelerate = -1;
        vex::this_thread::sleep_for(500);
      }
    } else {
      if (pre_color == 'B' && now_color != 'B') {
        Accelerate = -1;
        vex::this_thread::sleep_for(500);
      }
    }

    pre_color = now_color;
    vex::this_thread::sleep_for(80);
  }
}

}  // namespace sensors
}  // namespace basic
