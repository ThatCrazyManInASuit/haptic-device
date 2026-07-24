// Standalone sensor-only debug sketch - no driver, no motor, no binary
// protocol. Just prints the raw AS5048A angle in plain text so it can be
// read straight from the Arduino Serial Monitor while turning the shaft by
// hand. Use this to isolate a sensor/wiring problem from the FOC/firmware
// logic in haptic_motor_controller.ino.
//
// Same sensor wiring as haptic_motor_controller.ino: AS5048A over SPI, CS on
// D10, clock dropped to 100kHz for margin against a flaky solder joint.
//
// Reflash haptic_motor_controller.ino once you're done debugging - this
// sketch does not speak the PC-side binary protocol, so chai3d_visualizer/
// chai3d_device_test will not work while this is flashed.

#include <SimpleFOC.h>

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10, 100000);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  sensor.init();
  Serial.println("sensor_debug: sensor.init() done, printing angle (rad) and velocity (rad/s)");
}

void loop() {
  sensor.update();
  Serial.print("angle=");
  Serial.print(sensor.getAngle(), 4);
  Serial.print("  velocity=");
  Serial.println(sensor.getVelocity(), 4);
  delay(50);
}
