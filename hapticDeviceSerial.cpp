#include "hapticDeviceSerial.h"

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

HapticDeviceSerial::HapticDeviceSerial()
    : m_fd(-1), m_rxIndex(0), m_lastState(), m_hasState(false) {}

HapticDeviceSerial::~HapticDeviceSerial() { close(); }

bool HapticDeviceSerial::open(const std::string &devicePath, int baudRate) {
  close();

  m_fd = ::open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (m_fd < 0) {
    return false;
  }

  termios options{};
  if (tcgetattr(m_fd, &options) != 0) {
    close();
    return false;
  }

  speed_t speed;
  switch (baudRate) {
    case 9600:   speed = B9600;   break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:     speed = B115200; break;
  }
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  cfmakeraw(&options);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(m_fd, TCSANOW, &options) != 0) {
    close();
    return false;
  }

  tcflush(m_fd, TCIOFLUSH);
  m_rxIndex = 0;
  m_hasState = false;
  return true;
}

void HapticDeviceSerial::close() {
  if (m_fd >= 0) {
    ::close(m_fd);
    m_fd = -1;
  }
}

bool HapticDeviceSerial::sendTorque(float torqueVolts) {
  if (m_fd < 0) {
    return false;
  }

  CommandFrame frame{};
  frame.start = COMMAND_START_BYTE;
  frame.torque = torqueVolts;
  frame.checksum = computeChecksum(reinterpret_cast<uint8_t *>(&frame), sizeof(frame) - 1);

  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&frame);
  ssize_t written = ::write(m_fd, bytes, sizeof(frame));
  return written == static_cast<ssize_t>(sizeof(frame));
}

void HapticDeviceSerial::poll() {
  if (m_fd < 0) {
    return;
  }

  uint8_t byte;
  ssize_t n;
  while ((n = ::read(m_fd, &byte, 1)) > 0) {
    if (m_rxIndex == 0) {
      if (byte == STATE_START_BYTE) {
        m_rxBuf[m_rxIndex++] = byte;
      }
      continue;
    }

    m_rxBuf[m_rxIndex++] = byte;
    if (m_rxIndex == sizeof(StateFrame)) {
      uint8_t checksum = computeChecksum(m_rxBuf, sizeof(StateFrame) - 1);
      if (checksum == m_rxBuf[sizeof(StateFrame) - 1]) {
        std::memcpy(&m_lastState, m_rxBuf, sizeof(StateFrame));
        m_hasState = true;
      }
      m_rxIndex = 0;
    }
  }
}
