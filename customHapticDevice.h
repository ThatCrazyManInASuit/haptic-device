#pragma once

// termios-based serial I/O (hapticDeviceSerial.cpp) is POSIX-only.
#ifndef _WIN32

#include <string>

#include "chai3d.h"
#include "hapticDeviceSerial.h"

// Adapts the 1-DOF capstan-driven serial device (see hapticDeviceSerial.h and
// firmware/haptic_motor_controller) to CHAI3D's cGenericHapticDevice
// interface, so it can be assigned directly to LJ.cpp's `hapticDevice` in
// place of an auto-detected device.
//
// The single rotational DOF is mapped onto the X axis: shaft angle becomes X
// position, the X component of commanded force becomes torque. metersPerRadian
// and newtonsPerVolt are placeholders until the cable/capstan geometry is
// finalized - retune them against the actual mechanism.
class CustomHapticDevice : public chai3d::cGenericHapticDevice {
public:
  explicit CustomHapticDevice(const std::string &serialPort);
  ~CustomHapticDevice() override;

  bool open() override;
  bool close() override;
  bool calibrate(bool a_forceCalibration = false) override;

  bool getPosition(chai3d::cVector3d &a_position) override;
  bool getLinearVelocity(chai3d::cVector3d &a_linearVelocity) override;

  bool setForceAndTorqueAndGripperForce(const chai3d::cVector3d &a_force,
                                         const chai3d::cVector3d &a_torque,
                                         double a_gripperForce) override;

  bool enableForces(bool a_value) override;

private:
  std::string m_serialPort;
  HapticDeviceSerial m_serial;
  bool m_forcesEnabled;

  // Shaft angle captured at calibrate() time - motor.shaft_angle accumulates
  // continuously in the firmware (never wraps to +/-pi), so getPosition()
  // reports motion relative to this reference rather than the raw angle.
  float m_referenceAngle;

  // Capstan/cable calibration - PLACEHOLDERS until the mechanism is attached
  // and its real range of motion / force transmission is measured.
  static constexpr double metersPerRadian = 0.02;
  static constexpr double newtonsPerVolt = 1.0;
};

#endif // _WIN32
