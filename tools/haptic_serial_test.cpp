// Standalone smoke test for the firmware <-> PC serial link, independent of
// CHAI3D. Prints decoded angle/velocity and optionally drives a constant
// torque for a fixed duration.
//
// Build:
//   g++ -std=c++11 tools/haptic_serial_test.cpp hapticDeviceSerial.cpp -o haptic_serial_test
//
// Usage:
//   ./haptic_serial_test /dev/tty.usbmodemXXXX             # monitor only, runs until Ctrl+C
//   ./haptic_serial_test /dev/tty.usbmodemXXXX 1.5 3        # 1.5V torque for 3 seconds, then stop

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>

#include "../hapticDeviceSerial.h"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void handleSigint(int) { g_stop = 1; }
}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                  "usage: %s <serial-device> [torque_volts] [duration_seconds]\n",
                  argv[0]);
    return 1;
  }

  std::string port = argv[1];
  float torque = argc > 2 ? std::atof(argv[2]) : 0.0f;
  double durationSec = argc > 3 ? std::atof(argv[3]) : -1.0;  // -1 = run until Ctrl+C

  HapticDeviceSerial device;
  if (!device.open(port)) {
    std::fprintf(stderr, "failed to open %s\n", port.c_str());
    return 1;
  }

  std::signal(SIGINT, handleSigint);

  auto start = std::chrono::steady_clock::now();
  while (!g_stop) {
    if (durationSec >= 0.0) {
      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= durationSec) {
        break;
      }
    }

    device.sendTorque(torque);
    device.poll();
    if (device.hasState()) {
      std::printf("\rangle=%+8.4f rad   velocity=%+8.4f rad/s   ", device.shaftAngle(),
                  device.shaftVelocity());
      std::fflush(stdout);
    }

    // ~500 Hz command rate - well under the firmware's 100ms watchdog timeout.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  device.sendTorque(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  device.poll();
  std::printf("\nstopped.\n");
  return 0;
}
