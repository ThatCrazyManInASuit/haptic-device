// Diagnostic copy of haptic_motor_controller.ino - identical motor/driver/
// sensor setup, but instead of the binary PC protocol it prints
// sensor.getAngle() (raw sensor, bypassing the motor object entirely),
// motor.shaft_angle (what the real firmware sends to the PC), and
// motor.motor_status (SimpleFOC's init/calibration state) side by side in
// plain text. This isolates whether the FOC motor object itself is failing
// to pick up sensor motion, independent of the PC-side binary protocol.
//
// Reflash haptic_motor_controller.ino once done - this sketch does not
// speak the PC-side binary protocol.

#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10, 100000);

unsigned long lastPrintMicros = 0;
const unsigned long PRINT_INTERVAL_US = 100000; // 10 Hz, readable

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_limit = 5;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.init();
  Serial.print("motor.init() done, motor_status=");
  Serial.println((int)motor.motor_status);

  motor.initFOC();
  Serial.print("motor.initFOC() done, motor_status=");
  Serial.println((int)motor.motor_status);

  Serial.println("format: raw_sensor=... shaft_angle=... motor_status=...");
}

void loop() {
  motor.loopFOC();
  motor.move(0.0f); // zero commanded torque, just want shaft_angle updated

  unsigned long now = micros();
  if (now - lastPrintMicros >= PRINT_INTERVAL_US) {
    lastPrintMicros = now;
    Serial.print("raw_sensor=");
    Serial.print(sensor.getAngle(), 4);
    Serial.print("  shaft_angle=");
    Serial.print(motor.shaft_angle, 4);
    Serial.print("  motor_status=");
    Serial.println((int)motor.motor_status);
  }
}
