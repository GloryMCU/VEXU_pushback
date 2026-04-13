#include "hardware/sensors.h"

#include "v5_apiuser.h"

namespace basic::hardware::robots {

namespace {

void update_acceleration_state(SensorState& sensors) {
  switch (sensors.current_color_code) {
    case 'R':
      sensors.accelerate = kIsBlue ? 1 : 0;
      break;
    case 'G':
      sensors.accelerate = 0;
      break;
    case 'B':
      sensors.accelerate = kIsBlue ? 0 : 1;
      break;
    default:
      sensors.accelerate = 0;
      break;
  }
}

}  // namespace

void sensor_update(RobotHardware& hardware, RobotState& state) {
  SensorState& sensors = state.sensors;
  const int now = hardware.brain.timer(vex::timeUnits::msec);

  if (!sensors.initialized) {
    vexGenericSerialEnable(hardware.serial_sensor.index(), 9);
    vexGenericSerialBaudrate(hardware.serial_sensor.index(), 115200);
    sensors.initialized = true;
  }

  if (now - sensors.last_update_ms < kSensorLoopDelay) {
    if (now < sensors.hold_until_ms) {
      sensors.accelerate = -1;
    }
    return;
  }
  sensors.last_update_ms = now;

  int bytes_avail = vexGenericSerialReceiveAvail(hardware.serial_sensor.index());
  while (bytes_avail-- > 0) {
    const int character = vexGenericSerialReadChar(hardware.serial_sensor.index());
    if (character != -1) {
      sensors.current_color_code = static_cast<char>(character);
    }
  }

  update_acceleration_state(sensors);

  if (kIsBlue) {
    if (sensors.previous_color_code == 'R' && sensors.current_color_code != 'R') {
      sensors.hold_until_ms = now + 500;
    }
  } else if (sensors.previous_color_code == 'B' && sensors.current_color_code != 'B') {
    sensors.hold_until_ms = now + 500;
  }

  if (now < sensors.hold_until_ms) {
    sensors.accelerate = -1;
  }

  hardware.show_sensor_color(sensors.current_color_code);
  sensors.previous_color_code = sensors.current_color_code;
}

}  // namespace basic::hardware::robots
