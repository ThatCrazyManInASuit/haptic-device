// Shared wire-format definition for the PC <-> Arduino haptic link.
// Keep this file identical on both sides (firmware/haptic_motor_controller/protocol.h
// and the copy included by the PC-side custom haptic device class). It's small and
// should change rarely, so manual sync between the two copies is fine.
#pragma once
#include <stdint.h>
#include <stddef.h>

// Two-byte sync markers, distinct per direction, plus a CRC8 integrity check.
// A single sync byte with an XOR checksum can false-resync: after any dropped
// byte on the wire, the reader hunts for the next occurrence of the marker
// byte, which can also turn up by chance inside a normal float payload
// (~1/256 per frame). Once mis-locked, an 8-bit XOR checksum validates
// unrelated bytes by coincidence about 1/256 of the time too, so a single
// noise event (e.g. motor driver PWM switching near the serial line) could
// produce a run of accepted garbage frames. Two sync bytes cut the false-lock
// rate to ~1/65536, and CRC8 avoids XOR's blind spots against symmetric or
// swapped-byte corruption.
static const uint8_t STATE_SYNC0   = 0xB5; // Arduino -> PC
static const uint8_t STATE_SYNC1   = 0x5A;
static const uint8_t COMMAND_SYNC0 = 0xC3; // PC -> Arduino
static const uint8_t COMMAND_SYNC1 = 0x3C;

#pragma pack(push, 1)

// Arduino -> PC: current shaft state, sent once per loop tick.
struct StateFrame {
    uint8_t sync0;     // STATE_SYNC0
    uint8_t sync1;     // STATE_SYNC1
    float   angle;     // radians, motor.shaft_angle
    float   velocity;  // rad/s, motor.shaft_velocity
    uint8_t crc;        // CRC8 of all preceding bytes
};

// PC -> Arduino: target torque, sent once per loop tick.
struct CommandFrame {
    uint8_t sync0;     // COMMAND_SYNC0
    uint8_t sync1;     // COMMAND_SYNC1
    float   torque;    // volts (voltage-mode torque target)
    uint8_t crc;        // CRC8 of all preceding bytes
};

#pragma pack(pop)

// CRC-8/SMBUS (poly 0x07, init 0x00, no reflect) - simple bit-banged form,
// cheap enough to run per-frame on an Arduino Uno-class MCU.
inline uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
