#include <SimpleFOC.h>
#include "protocol.h"

// GM3506, 11 pole pairs.
BLDCMotor motor = BLDCMotor(11);
// SimpleFOC Shield v2 pins (Arduino Uno/R4 header): PWM 9/5/6, enable 8.
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// AS5048A over SPI, CS on D10. Clock dropped from SimpleFOC's 1MHz default to
// 100kHz for more timing margin against the still-flaky solder joint - raise
// this back toward 1MHz once the connection is solid.
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10, 100000);

// PC <-> Arduino tick rate. Both StateFrame (out) and CommandFrame (in) are
// keyed to this - see protocol.h.
const unsigned long LOOP_INTERVAL_US = 1000;   // 1 kHz
// If no CommandFrame arrives for this long, torque is zeroed. Guards against a
// dropped USB connection leaving the capstan drum spinning on a stale command.
const unsigned long COMMAND_TIMEOUT_US = 100000; // 100 ms

float targetTorque = 0.0f;
unsigned long lastCommandMicros = 0;
unsigned long lastTickMicros = 0;

uint8_t rxBuf[sizeof(CommandFrame)];
size_t rxIndex = 0;

void pollCommands() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (rxIndex == 0) {
      if (b == COMMAND_START_BYTE) {
        rxBuf[rxIndex++] = b;
      }
      continue;
    }

    rxBuf[rxIndex++] = b;
    if (rxIndex == sizeof(CommandFrame)) {
      uint8_t checksum = computeChecksum(rxBuf, sizeof(CommandFrame) - 1);
      if (checksum == rxBuf[sizeof(CommandFrame) - 1]) {
        CommandFrame frame;
        memcpy(&frame, rxBuf, sizeof(frame));
        targetTorque = frame.torque;
        lastCommandMicros = micros();
      }
      rxIndex = 0;
    }
  }
}

void sendState() {
  StateFrame frame;
  frame.start = STATE_START_BYTE;
  frame.angle = motor.shaft_angle;
  frame.velocity = motor.shaft_velocity;
  frame.checksum = computeChecksum(reinterpret_cast<uint8_t *>(&frame), sizeof(frame) - 1);
  Serial.write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
}

void setup() {
  Serial.begin(115200);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Conservative cap - no cable/capstan load attached yet. Raise once the
  // mechanism is connected and behavior at low torque is confirmed sane.
  motor.voltage_limit = 5;

  // Direct voltage-mode torque control: motor.move(x) sets Uq straight from
  // the PC's CommandFrame, bypassing SimpleFOC's velocity/angle PID loops -
  // the haptic render loop on the PC owns the control law, not the firmware.
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.init();
  motor.initFOC(); // keep the motor free to move during sensor alignment

  lastCommandMicros = micros();
  lastTickMicros = lastCommandMicros;
}

void loop() {
  motor.loopFOC(); // run as fast as possible
  pollCommands();

  unsigned long now = micros();
  if (now - lastTickMicros >= LOOP_INTERVAL_US) {
    lastTickMicros = now;

    if (now - lastCommandMicros > COMMAND_TIMEOUT_US) {
      targetTorque = 0.0f;
    }

    motor.move(targetTorque);
    sendState();
  }
}
