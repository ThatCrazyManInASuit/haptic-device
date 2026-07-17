// Shared wire-format definition for the PC <-> Arduino haptic link.
// Keep this file identical on both sides (firmware/haptic_motor_controller/protocol.h
// and the copy included by the PC-side custom haptic device class). It's small and
// should change rarely, so manual sync between the two copies is fine.
#pragma once
#include <stdint.h>
#include <stddef.h>

// Arbitrary, distinct start bytes so a receiver can resync mid-stream if a byte
// gets dropped or corrupted - unlikely to appear by chance in the float payload.
static const uint8_t STATE_START_BYTE   = 0xB5; // Arduino -> PC
static const uint8_t COMMAND_START_BYTE = 0xC3; // PC -> Arduino

#pragma pack(push, 1)

// Arduino -> PC: current shaft state, sent once per loop tick.
struct StateFrame {
    uint8_t start;     // STATE_START_BYTE
    float   angle;     // radians, motor.shaft_angle
    float   velocity;  // rad/s, motor.shaft_velocity
    uint8_t checksum;  // XOR of all preceding bytes
};

// PC -> Arduino: target torque, sent once per loop tick.
struct CommandFrame {
    uint8_t start;     // COMMAND_START_BYTE
    float   torque;    // volts (voltage-mode torque target)
    uint8_t checksum;  // XOR of all preceding bytes
};

#pragma pack(pop)

inline uint8_t computeChecksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum ^= data[i];
    return sum;
}
