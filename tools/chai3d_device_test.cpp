// Standalone check that CustomHapticDevice correctly feeds real chai3d types
// (cVector3d position/velocity) - no GLFW window, no LJ.cpp, just the device
// adapter talking to CHAI3D's own vector types.
//
// Usage: chai3d_device_test /dev/cu.usbmodemXXXX

#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

#include "../customHapticDevice.h"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void handleSigint(int) { g_stop = 1; }
}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <serial-device>\n", argv[0]);
    return 1;
  }

  CustomHapticDevice device(argv[1]);
  if (!device.open()) {
    std::fprintf(stderr, "failed to open device on %s\n", argv[1]);
    return 1;
  }
  device.calibrate();

  std::signal(SIGINT, handleSigint);

  chai3d::cVector3d position, velocity;
  while (!g_stop) {
    device.getPosition(position);
    device.getLinearVelocity(velocity);
    std::printf("\rx=%+8.4f m   vx=%+8.4f m/s   ", position.x(), velocity.x());
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  device.setForceAndTorqueAndGripperForce(chai3d::cVector3d(0, 0, 0),
                                           chai3d::cVector3d(0, 0, 0), 0.0);
  device.close();
  std::printf("\nstopped.\n");
  return 0;
}
