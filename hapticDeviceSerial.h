#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "protocol.h"

// Serial link to the custom 1-DOF haptic device firmware
// (firmware/haptic_motor_controller). POSIX-only (termios) - this class does
// not depend on CHAI3D, so it can be exercised standalone before being wired
// into the simulation's haptic device path.
class HapticDeviceSerial {
public:
  HapticDeviceSerial();
  ~HapticDeviceSerial();

  // Opens and configures the serial port. baudRate must match Serial.begin()
  // in the firmware (115200).
  bool open(const std::string &devicePath, int baudRate = 115200);
  void close();
  bool isOpen() const { return m_fd >= 0; }

  // Sends a torque command (volts) to the device.
  bool sendTorque(float torqueVolts);

  // Reads any bytes currently available and decodes complete StateFrames.
  // Non-blocking; call frequently (e.g. once per haptic loop tick).
  void poll();

  // Most recent StateFrame successfully decoded. hasState() is false until
  // the first valid frame arrives.
  bool hasState() const { return m_hasState; }
  float shaftAngle() const { return m_lastState.angle; }
  float shaftVelocity() const { return m_lastState.velocity; }

private:
  int m_fd;
  uint8_t m_rxBuf[sizeof(StateFrame)];
  size_t m_rxIndex;
  StateFrame m_lastState;
  bool m_hasState;
};
